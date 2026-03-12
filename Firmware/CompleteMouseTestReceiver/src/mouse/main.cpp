// ─────────────────────────────────────────────────────────────────────────────
//  Mouse firmware — position PID with trapezoidal profile + ESP-NOW telemetry
//  Author: chuck  (3/11)
//  Targets: ESP32 Arduino core 2.x  (ledcSetup / ledcAttachPin / ledcWrite-by-channel)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/pcnt.h"
#include "../shared/mouse_packet.h"   // shared struct — must match receiver include

// ─── ESP-NOW: receiver MAC address ───────────────────────────────────────────
// HOW TO GET THIS:
//   1. Flash the receiver firmware first.
//   2. Open its serial monitor (115200 baud).
//   3. It will print its MAC on startup — copy it here.
uint8_t RECEIVER_MAC[] = {0x38, 0x18, 0x2B, 0x8A, 0x1D, 0x0C};

// Global telemetry packet — fields are set during the control loop then sent
MousePacket pkt;

// Send the current pkt over ESP-NOW to the receiver
void espnowSend() {
    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

// Initialise ESP-NOW in STA mode and register the receiver as a peer
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed — check that receiver is powered on");
        return;
    }
    esp_now_peer_info_t peer = {};           // zero-initialise
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("ESP-NOW ready.");
}

// ─── Pin definitions ──────────────────────────────────────────────────────────
// Left motor  (physically mounted on the left side of the robot)
#define EN_L       25   // PWM enable pin for left motor
#define IN_L1      13   // direction pin A — HIGH + IN_L2 LOW  = forward
#define IN_L2      14   // direction pin B
#define ENC_L      22   // encoder pulse input for left wheel

// Right motor (physically mounted on the right side)
#define EN_R       18   // PWM enable pin for right motor
#define IN_R1      19   // direction pin A
#define IN_R2      21   // direction pin B
#define ENC_R      26   // encoder pulse input for right wheel

// LEDC channels — Arduino ESP32 2.x assigns one channel per PWM output.
// Channels 0 and 1 are free; do not reuse these numbers elsewhere.
#define LEDC_CH_L  0    // channel for left  motor EN_L
#define LEDC_CH_R  1    // channel for right motor EN_R

#define LEDC_FREQ  1000  // PWM frequency in Hz
#define LEDC_RES   8     // PWM resolution in bits (0-255 range)

// ─── IR sensor pins (unused in this firmware, kept for reference) ─────────────
#define IR_L_PIN   34
#define IR_R_PIN   35
#define IR_F_PIN   27

// ═════════════════════════════════════════════════════════════════════════════
//  TUNING — derived from impulse characterization data
// ═════════════════════════════════════════════════════════════════════════════
//
//  Impulse data summary (counts accumulated in 1 second at fixed PWM):
//    PWM   Left    Right
//     40   1105    1688      ← left deadband ~40, right ~30
//     60   3706    3587
//    100   6097    6324
//    150   7185    7417
//    200   7743    8241
//    255   8443    8674
//
//  kFF = PWM / (counts_per_sec / 100) = PWM / counts_per_10ms
//    Left:  ~2.0 PWM per count/10ms → scaled to 1ms loop: ×10 = 20.0
//    Right: ~1.9 PWM per count/10ms → scaled: 19.0
//
//  Usable cruise ≈ 60 counts/10ms at ~PWM 100 (leaves room for PID corrections)

// ─── Target distance ──────────────────────────────────────────────────────────
// Can be overridden at runtime by sending a number over USB serial.
int TARGET_COUNTS = 1500;

// ─── Control loop rate ────────────────────────────────────────────────────────
// Uses micros() so the period can be set below 1 ms.
// Change LOOP_US to adjust the rate — 500 = 2 kHz, 250 = 4 kHz, etc.
const int LOOP_US = 125;   // loop period in microseconds (500 µs = 2 kHz)

// ─── Trapezoidal velocity profile ────────────────────────────────────────────
// Units are counts/STEP. Step size = LOOP_US µs.
// To keep the same physical speed and acceleration as the original 1 ms loop,
// scale both values by (LOOP_US / 1000.0):
//   CRUISE_VEL: 6.0 counts/ms × (500µs / 1000) = 3.0 counts/step
//   ACCEL_RATE: 0.015 counts/ms² × (500µs / 1000) = 0.0075 counts/step²
// If you change LOOP_US, multiply both constants by (LOOP_US / 1000.0).
const float CRUISE_VEL = 0.75f;     // peak speed  in counts/step  (6.0 × 125/1000)
const float ACCEL_RATE = 0.001875f; // ramp rate   in counts/step² (0.015 × 125/1000)

// ─── Feedforward gains ───────────────────────────────────────────────────────
// FF gives each motor a base PWM proportional to the profile velocity,
// so the motors track the reference without a large steady-state error.
// Scaled from characterization (2.x counts/10ms) to the 1ms loop (×10).
const float kFF_L  = 20.0f;   // PWM per (count/ms) for left  motor
const float kFF_R  = 20.0f;   // PWM per (count/ms) for right motor
const float DEAD_L = 50.0f;   // minimum PWM to overcome left  motor deadband
const float DEAD_R = 50.0f;   // minimum PWM to overcome right motor deadband

// ─── Position PID gains ───────────────────────────────────────────────────────
// These correct the residual error between the reference position and
// the actual encoder count.
const float kP = 1.2f;    // proportional: direct error → PWM
const float kI = 0.001f;  // integral:     accumulated error → PWM
const float kD = 1.5f;    // derivative:   rate of change → PWM

// ─── Inter-motor synchronisation ─────────────────────────────────────────────
// If one motor runs ahead, this gain injects a correction into both PID loops
// to push them toward each other — reduces steering drift.
const float kSync = 0.175f;

// ─── Settle detection ────────────────────────────────────────────────────────
const int          SETTLE_WIN     = 5;       // both encoders must be within this many counts
const unsigned long SETTLE_TIME_US = 200000; // must stay settled for this long (200 ms in µs)

// ─── Controlled stop ─────────────────────────────────────────────────────────
// Reverse current magnitude during the stop phase.
// Too low → slow stop. Too high → robot bounces backward.
// Start around 60-70 and tune from there.
const int   STOP_PWM_L      = 70;   // reverse PWM for left  motor during stop
const int   STOP_PWM_R      = 70;   // reverse PWM for right motor during stop
const int   STOP_TIMEOUT_MS = 300;  // safety cutoff — force-stop after this many ms
                                    // in case encoders stop reporting (e.g. stall)

// ─── Startup engagement hold ─────────────────────────────────────────────────
// Before the profile begins, both motors are held at STARTUP_PWM for
// STARTUP_HOLD_MS. This pre-engages both motor windings simultaneously so
// neither motor has a head-start over the other. Encoders are re-zeroed
// after the hold, so the profile always begins from a clean zero.
// STARTUP_PWM should be just above the higher of the two deadbands (~45-50).
const int   STARTUP_HOLD_MS = 30;    // ms to hold before profile starts
const float STARTUP_PWM_L   = 50.0f; // pre-engage PWM for left  motor
const float STARTUP_PWM_R   = 50.0f; // pre-engage PWM for right motor

// ═════════════════════════════════════════════════════════════════════════════
//  PCNT (Pulse Counter) setup
//  The ESP32's hardware PCNT peripheral counts encoder pulses without CPU load.
//  We use one unit per motor — UNIT_0 = left, UNIT_1 = right.
// ═════════════════════════════════════════════════════════════════════════════
void setupPCNT(pcnt_unit_t unit, int pin) {
    pcnt_config_t cfg = {
        .pulse_gpio_num = pin,            // encoder signal pin
        .ctrl_gpio_num  = PCNT_PIN_NOT_USED,
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .pos_mode       = PCNT_COUNT_INC, // count rising edges
        .neg_mode       = PCNT_COUNT_INC, // count falling edges (quadrature-like)
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg);
    pcnt_filter_disable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Motor helpers
// ═════════════════════════════════════════════════════════════════════════════

// Controlled stop — flips the H-bridge to reverse and applies STOP_PWM to both
// motors, then polls the encoders every 5 ms until both wheels have stopped
// spinning. Cuts power once motion ceases (or after STOP_TIMEOUT_MS).
// This produces a much shorter, more repeatable stop than coasting or windings-short.
void controlledStop() {
    // Flip both motors to reverse direction
    digitalWrite(IN_L1, HIGH); digitalWrite(IN_L2, LOW);
    digitalWrite(IN_R1, HIGH); digitalWrite(IN_R2, LOW);
    ledcWrite(LEDC_CH_L, STOP_PWM_L);
    ledcWrite(LEDC_CH_R, STOP_PWM_R);

    unsigned long start = millis();

    // Seed the previous encoder snapshot
    int16_t prevL, prevR;
    pcnt_get_counter_value(PCNT_UNIT_0, &prevL);
    pcnt_get_counter_value(PCNT_UNIT_1, &prevR);

    while (millis() - start < STOP_TIMEOUT_MS) {
        delay(5);   // 5 ms sample window — short enough to react quickly

        int16_t curL, curR;
        pcnt_get_counter_value(PCNT_UNIT_0, &curL);
        pcnt_get_counter_value(PCNT_UNIT_1, &curR);

        // Both encoders unchanged → wheels have stopped, exit early
        if (abs((int)curL - (int)prevL) == 0 &&
            abs((int)curR - (int)prevR) == 0) break;

        prevL = curL;
        prevR = curR;
    }

    // Cut all power once stopped (or timed out)
    ledcWrite(LEDC_CH_L, 0); ledcWrite(LEDC_CH_R, 0);
    digitalWrite(IN_L1, LOW); digitalWrite(IN_L2, LOW);
    digitalWrite(IN_R1, LOW); digitalWrite(IN_R2, LOW);
}

// Coast stop — removes power, motors roll to a halt
void stopMotors() {
    ledcWrite(LEDC_CH_L, 0); ledcWrite(LEDC_CH_R, 0);
    digitalWrite(IN_L1, LOW); digitalWrite(IN_L2, LOW);
    digitalWrite(IN_R1, LOW); digitalWrite(IN_R2, LOW);
}

// Active brake — drives both direction pins HIGH simultaneously to short the
// motor windings, creating a resistive braking force
void brakeMotors() {
    digitalWrite(IN_L1, HIGH); digitalWrite(IN_L2, HIGH); ledcWrite(LEDC_CH_L, 255);
    digitalWrite(IN_R1, HIGH); digitalWrite(IN_R2, HIGH); ledcWrite(LEDC_CH_R, 255);
}

// Set both motors to drive forward (direction pins only — PWM is set separately)
void setForward() {
    digitalWrite(IN_L1, LOW); digitalWrite(IN_L2, HIGH);  // left  forward
    digitalWrite(IN_R1, LOW); digitalWrite(IN_R2, HIGH);  // right forward
}

// Drive each motor with a signed PWM value.
// Positive → forward, negative → reverse (flips H-bridge direction pins).
// This is used in the braking zone to allow active deceleration via reverse current.
void setMotorPWM(float pwmL, float pwmR) {
    // Left motor
    if (pwmL >= 0) {
        digitalWrite(IN_L1, LOW);  digitalWrite(IN_L2, HIGH); // forward
        ledcWrite(LEDC_CH_L, (int)pwmL);
    } else {
        digitalWrite(IN_L1, HIGH); digitalWrite(IN_L2, LOW);  // reverse
        ledcWrite(LEDC_CH_L, (int)(-pwmL));
    }
    // Right motor
    if (pwmR >= 0) {
        digitalWrite(IN_R1, LOW);  digitalWrite(IN_R2, HIGH); // forward
        ledcWrite(LEDC_CH_R, (int)pwmR);
    } else {
        digitalWrite(IN_R1, HIGH); digitalWrite(IN_R2, LOW);  // reverse
        ledcWrite(LEDC_CH_R, (int)(-pwmR));
    }
}

// Read both encoder counts. abs() is used because PCNT counts both edges,
// and the sign depends on direction. We always drive forward here.
void readEncoders(int32_t &eL, int32_t &eR) {
    int16_t rL, rR;
    pcnt_get_counter_value(PCNT_UNIT_0, &rL);
    pcnt_get_counter_value(PCNT_UNIT_1, &rR);
    eL = abs(rL);
    eR = abs(rR);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Trapezoidal profile generator
//  Returns the next reference position (counts) each time it is called.
//  Accelerates up to CRUISE_VEL, then decelerates so the robot stops exactly
//  at the target without overshooting.
// ═════════════════════════════════════════════════════════════════════════════
static float profPos = 0.0f;  // reference position accumulator (counts)
static float profVel = 0.0f;  // reference velocity            (counts/ms)

float profileStep(int target) {
    float remaining = (float)target - profPos;
    // Distance needed to stop from current speed: v² / (2a)
    float stopDist  = (profVel * profVel) / (2.0f * ACCEL_RATE);

    if (remaining <= stopDist) {
        // Close enough that we must start braking now
        profVel -= ACCEL_RATE;
        if (profVel < 0.0f) profVel = 0.0f;
    } else if (profVel < CRUISE_VEL) {
        // Still accelerating toward cruise speed
        profVel += ACCEL_RATE;
        if (profVel > CRUISE_VEL) profVel = CRUISE_VEL;
    }
    // Advance reference position by current velocity
    profPos += profVel;
    if (profPos > (float)target) profPos = (float)target;  // clamp at target
    return profPos;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main drive routine — runs a full move to a target encoder count
//  Sends telemetry every 50 ms over USB serial AND over ESP-NOW to receiver
// ═════════════════════════════════════════════════════════════════════════════
void driveToCount(int target) {
    // ── Reset all state before starting ──────────────────────────────────────
    profPos = 0.0f;
    profVel = 0.0f;
    float intL = 0.0f, intR = 0.0f;       // PID integrators
    float lastErrL = 0.0f, lastErrR = 0.0f; // previous errors for derivative

    // ── Pre-engagement hold ───────────────────────────────────────────────────
    // Drive both motors at STARTUP_PWM for STARTUP_HOLD_MS before the profile
    // starts. This ensures both motors are physically spinning before the
    // reference position begins advancing — eliminating the race condition where
    // one motor engages a few ms before the other and builds an uncorrectable
    // head-start. Encoders are re-cleared after the hold so the profile sees
    // a true zero starting point.
    setForward();
    ledcWrite(LEDC_CH_L, (int)STARTUP_PWM_L);
    ledcWrite(LEDC_CH_R, (int)STARTUP_PWM_R);
    delay(STARTUP_HOLD_MS);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    unsigned long startTime    = micros(); // µs — run start time
    unsigned long lastStep     = micros(); // µs — loop rate control
    unsigned long settledSince = 0;        // µs — settle timer
    bool settled = false;

    // NOTE: CSV header below — uncomment if you want to paste output into a spreadsheet.
    // Serial.println("timestamp_ms,ref,enc_L,enc_R,diff,pwm_L,pwm_R,vel_ref");

    while (true) {
        // ── Enforce loop rate using micros() for sub-ms precision ────────────
        if (micros() - lastStep < (unsigned long)LOOP_US) continue;
        lastStep = micros();
        unsigned long now = micros();   // µs — used for telemetry, settle, and timeout

        // ── Step the velocity profile forward by one ms ───────────────────────
        float ref = profileStep(target);

        // ── Read actual encoder positions ─────────────────────────────────────
        int32_t eL, eR;
        readEncoders(eL, eR);
        float cntL = (float)eL;
        float cntR = (float)eR;

        // ── Sync correction ───────────────────────────────────────────────────
        // If left is ahead of right, sync > 0.
        // We subtract sync from left's error (slowing it down) and add to
        // right's error (speeding it up), keeping both wheels tracking together.
        float sync = kSync * (cntL - cntR);

        // ── Position errors with sync injected ───────────────────────────────
        float errL = (ref - cntL) - sync;   // reduce left  if it leads
        float errR = (ref - cntR) + sync;   // boost  right if left leads

        // ── PID: accumulate integral, compute derivative ───────────────────────
        intL = constrain(intL + errL, -500.0f, 500.0f);
        intR = constrain(intR + errR, -500.0f, 500.0f);

        float dL = errL - lastErrL; lastErrL = errL;
        float dR = errR - lastErrR; lastErrR = errR;

        // ── Feedforward + PID → final PWM ─────────────────────────────────────
        // FF: scales profile velocity to a base PWM + adds deadband offset so
        //     the motor actually starts moving at low speeds.
        // PID: corrects position error that FF leaves behind.
        // Apply deadband offset as soon as the profile starts moving (profVel > 0).
        // Previously this threshold was 0.1, which left a ~7ms window at startup
        // where PWM was too low to move the motors — causing integrator windup
        // and a jerky, inconsistent launch. Using > 0 ensures the motors receive
        // a valid PWM from step 1, matching the profile from the very beginning.
        float ffL = kFF_L * profVel + (profVel > 0.0f ? DEAD_L : 0.0f);
        float ffR = kFF_R * profVel + (profVel > 0.0f ? DEAD_R : 0.0f);

        // ── Braking zone ──────────────────────────────────────────────────────
        // Once the average encoder position passes 75% of the target, unlock
        // negative PWM so the PID can apply reverse current through the H-bridge.
        // Before 75%: clamp to [0, 255] — forward only.
        // After  75%: clamp to [-255, 255] — PID can actively reverse to prevent overshoot.
        float avgPos = (float)(eL + eR) / 2.0f;
        float minPWM = (avgPos >= 0.75f * target) ? -255.0f : 0.0f;

        float pwmL = constrain(ffL + kP*errL + kI*intL + kD*dL, minPWM, 255.0f);
        float pwmR = constrain(ffR + kP*errR + kI*intR + kD*dR, minPWM, 255.0f);

        // setMotorPWM handles direction pins — positive = forward, negative = reverse
        setMotorPWM(pwmL, pwmR);

        // ── Telemetry — every 12.5 ms / 100 steps at 8 kHz (decoupled from control loop) ─
        static int logDiv = 0;
        if (++logDiv >= 100) {
            logDiv = 0;

            // NOTE: CSV format below — uncomment and comment out the readable line
            // if you want to paste output into a spreadsheet.
            // Serial.printf("%lu,%.0f,%ld,%ld,%ld,%.0f,%.0f,%.2f\n",
            //               now - startTime, ref, eL, eR, eL - eR, pwmL, pwmR, profVel);

            // Human-readable serial monitor output
            // diff = enc_L - enc_R  (+ means left ahead, - means right ahead)
            Serial.printf("t=%luus  ref=%.0f  L=%ld  R=%ld  diff=%ld  pwmL=%.0f  pwmR=%.0f  vel=%.2f\n",
                          now - startTime, ref, eL, eR, eL - eR, pwmL, pwmR, profVel);

            // ESP-NOW: fill shared packet and send to receiver
            // m1_count / m1_pwm = left  motor (L)
            // m2_count / m2_pwm = right motor (R)
            pkt.m1_count = (int32_t)eL;
            pkt.m2_count = (int32_t)eR;
            pkt.m1_pwm   = (int16_t)pwmL;
            pkt.m2_pwm   = (int16_t)pwmR;
            // Repurpose distance fields to carry profile state for the receiver
            pkt.dist_left_cm  = ref;      // reference position (counts)
            pkt.dist_right_cm = profVel;  // profile velocity   (counts/ms)
            strncpy(pkt.label, "DRIVE", sizeof(pkt.label));
            espnowSend();
        }

        // ── Settle detection ──────────────────────────────────────────────────
        // Both encoders must be within SETTLE_WIN counts of target AND the
        // profile must be finished (vel == 0, pos == target).
        bool inWindow = (abs(eL - target) <= SETTLE_WIN) &&
                        (abs(eR - target) <= SETTLE_WIN) &&
                        (profVel == 0.0f) &&
                        (profPos >= (float)target);

        if (inWindow) {
            if (!settled) { settled = true; settledSince = now; }
            if (now - settledSince >= SETTLE_TIME_US) break; // done!
        } else {
            settled = false; // reset timer if we drift out of window
        }

        // ── Safety timeout (30 s = 30,000,000 µs) ────────────────────────────
        if (now - startTime > 30000000UL) {
            Serial.println("TIMEOUT — motor or encoder issue, stopping.");
            break;
        }
    }

    // Apply reverse current to bring the wheels to a controlled stop
    controlledStop();

    // Report final result
    int32_t finalL, finalR;
    readEncoders(finalL, finalR);
    Serial.printf("TARGET=%d  FINAL: L=%ld R=%ld  ERR: L=%ld R=%ld\n",
                  target, finalL, finalR, finalL - target, finalR - target);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // ── Left motor ────────────────────────────────────────────────────────────
    // ledcSetup: configure channel frequency and resolution (ESP32 Arduino 2.x)
    // ledcAttachPin: bind the physical EN pin to that channel
    ledcSetup(LEDC_CH_L, LEDC_FREQ, LEDC_RES);
    ledcAttachPin(EN_L, LEDC_CH_L);
    pinMode(IN_L1, OUTPUT);
    pinMode(IN_L2, OUTPUT);
    pinMode(ENC_L, INPUT_PULLUP);          // encoder needs pull-up to read cleanly
    setupPCNT(PCNT_UNIT_0, ENC_L);        // UNIT_0 counts left encoder pulses

    // ── Right motor ───────────────────────────────────────────────────────────
    ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
    ledcAttachPin(EN_R, LEDC_CH_R);
    pinMode(IN_R1, OUTPUT);
    pinMode(IN_R2, OUTPUT);
    pinMode(ENC_R, INPUT_PULLUP);
    setupPCNT(PCNT_UNIT_1, ENC_R);        // UNIT_1 counts right encoder pulses

    stopMotors();  // ensure motors are off before anything runs

    // ── ESP-NOW ───────────────────────────────────────────────────────────────
    setupEspNow();

    // ── Ready prompt ──────────────────────────────────────────────────────────
    // Press EN to reset the ESP32 and start a new trial — this separator will
    // appear in the serial monitor each time so runs are easy to tell apart.
    Serial.println("\n════════════════════ NEW TRIAL ════════════════════");
    Serial.println("=== POSITION PID CONTROLLER ===");
    Serial.println("Send target encoder count over serial (e.g. '3000')");
    Serial.println("Press Enter to use default target.");
    Serial.printf("Default target: %d counts\n\n", TARGET_COUNTS);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Loop — waits for a target count from the USB serial monitor
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
    // ── Serial: accept target count ───────────────────────────────────────────
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Parse target — use default if the input is empty or non-numeric
        int target = input.length() > 0 ? input.toInt() : TARGET_COUNTS;
        if (target <= 0) target = TARGET_COUNTS;

        Serial.printf("\n--- Driving to %d counts ---\n", target);
        delay(500);         // short pause so you can place the robot before it moves
        driveToCount(target);
        Serial.println("--- Done. Send a new target to run again. ---\n");
    }
}
