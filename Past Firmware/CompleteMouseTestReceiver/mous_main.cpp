#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/pcnt.h"
#include "../shared/mouse_packet.h"

// ─────────────────────────────────────────────────────────────────────────────
//  TEST MODE — uncomment exactly ONE
// ─────────────────────────────────────────────────────────────────────────────
// #define TEST_STRAIGHT      // PID straight-line drive
// #define TEST_TURN           // encoder-based turn
// #define TEST_SENSORS        // IR sensor readings only
// #define TEST_ESPNOW         // ESP-NOW link check (no motors/sensors)
 #define TEST_PROFILE        // unified PID + trapezoidal profile (straight or turn)
// #define TEST_CHARACTERIZE   // motor deadband + speed curve (Phase 1 calibration)
// #define TEST_FORWARD        // open-loop forward: verify both motors spin fwd + encoders count
// ─────────────────────────────────────────────────────────────────────────────

// ─── ESP-NOW ─────────────────────────────────────────────────────────────────
// SETUP STEP: flash receiver first, note its MAC from serial monitor, paste here
uint8_t RECEIVER_MAC[] = {0x38, 0x18, 0x2B, 0x8A, 0x1D, 0x0C}; //  38:18:2B:8A:1D:0C

MousePacket pkt;

void espnowSend() {
    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

// ─── Pin definitions ─────────────────────────────────────────────────────────
#define ENA        25
#define IN1        13
#define IN2        14
#define ENCODER_A1 26

#define ENB        18
#define IN3        19
#define IN4        21
#define ENCODER_A2 22

#define ENA_CH     0   // LEDC channel for ENA (Arduino ESP32 2.x)
#define ENB_CH     1   // LEDC channel for ENB

#define IR_L_PIN   34
#define IR_R_PIN   35
#define IR_F_PIN   27   // digital front sensor

#define LEDC_FREQ  1000
#define LEDC_RES   8

// ─── PCNT setup ──────────────────────────────────────────────────────────────
void setupPCNT(pcnt_unit_t unit, int pin) {
    pcnt_config_t cfg = {
        .pulse_gpio_num = pin,
        .ctrl_gpio_num  = PCNT_PIN_NOT_USED,
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_INC,
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

// ─── Motor helpers ───────────────────────────────────────────────────────────
void stopMotors() {
    ledcWrite(ENA_CH, 0); ledcWrite(ENB_CH, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setForward() {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

// ─── Sensor helpers ──────────────────────────────────────────────────────────
int readAvg(int pin) {
    long sum = 0;
    for (int i = 0; i < 15; i++) { sum += analogRead(pin); delayMicroseconds(250); }
    return (int)(sum / 15);
}

float adcToVoltage(int adc)          { return adc * 3.3f / 4095.0f; }

float voltageToDistanceCm(float v) {
    float denom = (v * 200.0f - 11.0f);
    if (denom <= 1.0f) return -1.0f;
    return 2076.0f / denom;
}

void readSensors() {
    int adcL = readAvg(IR_L_PIN);
    int adcR = readAvg(IR_R_PIN);
    pkt.adc_left      = adcL;
    pkt.adc_right     = adcR;
    pkt.dist_left_cm  = voltageToDistanceCm(adcToVoltage(adcL));
    pkt.dist_right_cm = voltageToDistanceCm(adcToVoltage(adcR));
    pkt.front_blocked = (digitalRead(IR_F_PIN) == LOW) ? 1 : 0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: STRAIGHT LINE (velocity PID)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_STRAIGHT

float kP = 0.5, kI = 0.005, kD = 0.0;
const float M1_TARGET = 13, M2_TARGET = 13;
float m1_integral=0, m1_lastError=0, m1_pwm=55;
float m2_integral=0, m2_lastError=0, m2_pwm=55;
int16_t m1_lastCount=0, m2_lastCount=0;
unsigned long lastPIDTime = 0;

void updatePID() {
    int16_t m1_count, m2_count;
    pcnt_get_counter_value(PCNT_UNIT_0, &m1_count);
    pcnt_get_counter_value(PCNT_UNIT_1, &m2_count);

    int16_t m1_delta = m1_count - m1_lastCount;
    if (abs(m1_count) > 30000) { pcnt_counter_clear(PCNT_UNIT_0); m1_lastCount = 0; }
    else m1_lastCount = m1_count;

    int16_t m2_delta = m2_count - m2_lastCount;
    if (abs(m2_count) > 30000) { pcnt_counter_clear(PCNT_UNIT_1); m2_lastCount = 0; }
    else m2_lastCount = m2_count;

    float m1_error = M1_TARGET - abs(m1_delta);
    float m2_error = M2_TARGET - abs(m2_delta);

    m1_integral = constrain(m1_integral + m1_error, -50, 50);
    m2_integral = constrain(m2_integral + m2_error, -50, 50);

    m1_pwm = constrain(m1_pwm + kP*m1_error + kI*m1_integral, 30, 255);
    m2_pwm = constrain(m2_pwm + kP*m2_error + kI*m2_integral, 30, 255);

    ledcWrite(ENA_CH, (int)m1_pwm);
    ledcWrite(ENB_CH, (int)m2_pwm);

    pkt.m1_count = m1_count; pkt.m2_count = m2_count;
    pkt.m1_pwm   = m1_pwm;   pkt.m2_pwm   = m2_pwm;
    strncpy(pkt.label, "STRAIGHT", sizeof(pkt.label));
    espnowSend();
}

void runStraightTest() {
    setForward();
    unsigned long start = millis();
    while (millis() - start < 2000) {
        if (millis() - lastPIDTime >= 10) {
            lastPIDTime = millis();
            updatePID();
        }
    }
    stopMotors();
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: TURN (left in-place turn, clean baseline)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_TURN

// --- Tune these ---
const int COUNTS_PER_90  = 150;
const int COUNTS_PER_180 = 300;
const int M1_TURN_PWM    = 100;
const int M2_TURN_PWM    = 100;

// Left turn: M1 backward, M2 forward
// Both motors stop independently when they reach targetCounts
void turn(int targetCounts) {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // M1 backward
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // M2 forward
    ledcWrite(ENA_CH, M1_TURN_PWM);
    ledcWrite(ENB_CH, M2_TURN_PWM);

    bool m1_done = false, m2_done = false;

    while (!m1_done || !m2_done) {
        int16_t r1, r2;
        pcnt_get_counter_value(PCNT_UNIT_0, &r1);
        pcnt_get_counter_value(PCNT_UNIT_1, &r2);
        int c1 = abs(r1), c2 = abs(r2);

        if (c1 >= targetCounts && !m1_done) {
            ledcWrite(ENA_CH, 0);
            digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
            m1_done = true;
        }
        if (c2 >= targetCounts && !m2_done) {
            ledcWrite(ENB_CH, 0);
            digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
            m2_done = true;
        }

        pkt.m1_count = c1; pkt.m2_count = c2;
        pkt.m1_pwm   = m1_done ? 0 : M1_TURN_PWM;
        pkt.m2_pwm   = m2_done ? 0 : M2_TURN_PWM;
        strncpy(pkt.label, "TURN", sizeof(pkt.label));
        espnowSend();
    }
}

void turn90()  { turn(COUNTS_PER_90);  }
void turn180() { turn(COUNTS_PER_180); }
void turn270() { turn(COUNTS_PER_90 * 3); }
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: SENSORS only
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_SENSORS
void runSensorTest() {
    readSensors();
    strncpy(pkt.label, "SENSORS", sizeof(pkt.label));

    // Also print locally for USB debugging
    Serial.print("L: "); Serial.print(pkt.dist_left_cm, 1);
    Serial.print("cm | R: "); Serial.print(pkt.dist_right_cm, 1);
    Serial.print("cm | Front: "); Serial.println(pkt.front_blocked ? "WALL" : "CLEAR");

    espnowSend();
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: ESP-NOW link check (no motors or sensors)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_ESPNOW
int espnow_counter = 0;
void runEspNowTest() {
    pkt.m1_count = espnow_counter++;
    strncpy(pkt.label, "ESPNOW", sizeof(pkt.label));
    espnowSend();
    Serial.print("Sent packet #"); Serial.println(espnow_counter);
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: FORWARD VERIFICATION  (open-loop — check motors + encoders before PID)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_FORWARD

// Fixed PWM applied to both motors.  Start low (80–100) so the robot doesn't
// shoot off the table.  Raise only if wheels don't move.
const int FWD_PWM    = 90;
const int FWD_RUN_MS = 2500;   // how long to drive before stopping

void runForwardTest() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    setForward();
    ledcWrite(ENA_CH, FWD_PWM);
    ledcWrite(ENB_CH, FWD_PWM);

    Serial.println("=== FORWARD TEST START ===");
    Serial.println("t(ms), M1_counts, M2_counts");

    unsigned long start = millis();
    unsigned long lastPrint = 0;

    while (millis() - start < FWD_RUN_MS) {
        if (millis() - lastPrint >= 100) {
            lastPrint = millis();

            int16_t r1, r2;
            pcnt_get_counter_value(PCNT_UNIT_0, &r1);
            pcnt_get_counter_value(PCNT_UNIT_1, &r2);

            Serial.print(millis() - start);
            Serial.print(", ");
            Serial.print(r1);
            Serial.print(", ");
            Serial.println(r2);

            // Also send via ESP-NOW so the receiver sees it
            pkt.m1_count = r1;
            pkt.m2_count = r2;
            pkt.m1_pwm   = FWD_PWM;
            pkt.m2_pwm   = FWD_PWM;
            strncpy(pkt.label, "FORWARD", sizeof(pkt.label));
            espnowSend();
        }
    }

    stopMotors();

    int16_t final1, final2;
    pcnt_get_counter_value(PCNT_UNIT_0, &final1);
    pcnt_get_counter_value(PCNT_UNIT_1, &final2);

    Serial.println("=== FORWARD TEST DONE ===");
    Serial.print("Final M1: "); Serial.println(final1);
    Serial.print("Final M2: "); Serial.println(final2);
    Serial.println();
    Serial.println("INTERPRET RESULTS:");
    Serial.println("  Both positive → both motors wired correctly for forward");
    Serial.println("  One negative  → that motor is wired backwards (swap IN pins or motor wires)");
    Serial.println("  One near zero → encoder not counting (check encoder pin / wiring)");
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: MOTOR CHARACTERIZATION  (Phase 1 calibration — run before tuning PID)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_CHARACTERIZE

// PWM levels to sweep. Start low to find deadband, then cover the usable range.
const int CHAR_PWM_STEPS[] = {20, 30, 40, 50, 60, 70, 80, 100, 120, 150, 180, 210};
const int CHAR_NUM_STEPS   = 12;
const int CHAR_SETTLE_MS   = 200;  // wait for motor to reach steady-state speed
const int CHAR_MEASURE_MS  = 800;  // measure window — longer = more accurate average

// Test one motor at fixed PWM. The other motor is fully disabled.
// Returns measured speed in counts per 10ms step.
float charMotorSingle(int motorNum, int testPWM) {
    // Apply PWM to the target motor, kill the other
    if (motorNum == 1) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // M1 forward
        ledcWrite(ENA_CH, testPWM);
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);  // M2 coast
        ledcWrite(ENB_CH, 0);
    } else {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // M2 forward
        ledcWrite(ENB_CH, testPWM);
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);  // M1 coast
        ledcWrite(ENA_CH, 0);
    }

    delay(CHAR_SETTLE_MS);           // let speed stabilise

    pcnt_counter_clear(PCNT_UNIT_0); // start a clean count window
    pcnt_counter_clear(PCNT_UNIT_1);

    delay(CHAR_MEASURE_MS);

    int16_t cnt1, cnt2;
    pcnt_get_counter_value(PCNT_UNIT_0, &cnt1);
    pcnt_get_counter_value(PCNT_UNIT_1, &cnt2);

    // Speed = total counts over window / number of 10ms steps in window
    int raw = (motorNum == 1) ? abs(cnt1) : abs(cnt2);
    return (float)raw / (CHAR_MEASURE_MS / 10.0f);
}

void runCharacterizeTest() {
    Serial.println();
    Serial.println("==============================================");
    Serial.println("  MOTOR CHARACTERIZATION");
    Serial.println("  PWM | M1 speed (cts/step) | M2 speed (cts/step)");
    Serial.println("------+----------------------+--------------------");

    for (int i = 0; i < CHAR_NUM_STEPS; i++) {
        int pwm = CHAR_PWM_STEPS[i];

        float speed1 = charMotorSingle(1, pwm);
        stopMotors();
        delay(300);  // brief pause between motors so inertia clears

        float speed2 = charMotorSingle(2, pwm);
        stopMotors();
        delay(300);

        // Print as a table row
        Serial.print("  "); Serial.print(pwm);
        if (pwm < 100) Serial.print(" ");   // align columns
        if (pwm < 10)  Serial.print(" ");
        Serial.print(" | ");
        Serial.print(speed1, 2);
        Serial.print("                | ");
        Serial.println(speed2, 2);

        // Also send via ESP-NOW so receiver logs it
        pkt.m1_count = pwm;
        pkt.m1_pwm   = (int16_t)(speed1 * 100);  // speed × 100 for 2 dp as integer
        pkt.m2_pwm   = (int16_t)(speed2 * 100);
        strncpy(pkt.label, "CHAR", sizeof(pkt.label));
        espnowSend();
    }

    stopMotors();
    Serial.println("==============================================");
    Serial.println();
    Serial.println("HOW TO READ THESE RESULTS:");
    Serial.println("  Deadband  = lowest PWM where speed > 0.1 counts/step");
    Serial.println("  kFF (M1)  = PWM / speed  at cruise (e.g. 150/6.0 = 25.0)");
    Serial.println("  kFF (M2)  = PWM / speed  at cruise");
    Serial.println("  M2_BIAS   = kFF_M2*cruise - kFF_M1*cruise  (extra PWM M2 needs)");
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  TEST: UNIFIED PID + TRAPEZOIDAL PROFILE (straight or left turn)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef TEST_PROFILE

// ── Direction: uncomment ONE ──────────────────────────────────────────────────
#define PROFILE_STRAIGHT      // both motors forward
// #define PROFILE_TURN       // left in-place turn (M1 backward, M2 forward)

// ── Motion profile tuning ────────────────────────────────────────────────────
// All count/step values are relative to PROFILE_STEP_MS (currently 1ms).
// Physical cruise speed = PROFILE_CRUISE × (1000 / PROFILE_STEP_MS) counts/sec
//   → 0.5 × 1000 = 500 counts/sec
const int   PROFILE_TARGET = 300;      // total encoder counts to travel
const float PROFILE_ACCEL  = 0.0025f; // counts/step² — ramp rate (accel & decel)
const float PROFILE_CRUISE = 0.5f;    // peak speed in counts/step

// ── PID tuning ────────────────────────────────────────────────────────────────
// P_kFF = PWM per (count/step). Derived from characterisation:
//   on-bench kFF_physical = PWM/speed = 90/5 = 18 counts/(counts/10ms)
//   scaled for 1ms steps: 18 × 10 = 180
const float P_kFF   = 180.0f; // velocity feedforward gain
const float M2_BIAS =   0.0f; // static PWM offset for M2 — motors matched per characterisation

const float P_kP = 1.5f;   // proportional gain
const float P_kI = 0.01f;  // integral gain
const float P_kD = 0.05f;  // derivative gain
const float P_Ks = 0.75f;  // inter-motor sync gain

// ── Supervisor ───────────────────────────────────────────────────────────────
const int   PROFILE_STEP_MS    = 1;  // control loop period (ms)
const int   PROFILE_SETTLE_WIN = 8;   // encoder counts within target to declare done

// ── Profile state ─────────────────────────────────────────────────────────────
static float p_ref = 0.0f;   // current reference position (counts)
static float p_vel = 0.0f;   // current reference velocity (counts/step)

// ── PID state ─────────────────────────────────────────────────────────────────
static float p_m1_int = 0.0f, p_m1_lastErr = 0.0f;
static float p_m2_int = 0.0f, p_m2_lastErr = 0.0f;

// Advance the trapezoidal profile by one step. Returns the new reference position.
// Decelerates when remaining distance equals the stopping distance at current speed.
float profileStep() {
    float remaining  = (float)PROFILE_TARGET - p_ref;
    float stop_dist  = (p_vel * p_vel) / (2.0f * PROFILE_ACCEL);

    if (remaining <= stop_dist) {
        p_vel -= PROFILE_ACCEL;
        if (p_vel < 0.0f) p_vel = 0.0f;
    } else if (p_vel < PROFILE_CRUISE) {
        p_vel += PROFILE_ACCEL;
        if (p_vel > PROFILE_CRUISE) p_vel = PROFILE_CRUISE;
    }

    p_ref += p_vel;
    if (p_ref > (float)PROFILE_TARGET) p_ref = (float)PROFILE_TARGET;
    return p_ref;
}

void runProfileTest() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    // Set motor directions — same direction pins held throughout the move
#ifdef PROFILE_STRAIGHT
    setForward();
#else  // PROFILE_TURN: left in-place
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // M1 backward
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // M2 forward
#endif

    unsigned long lastStep    = millis();
    unsigned long lastTelemetry = millis();

    while (true) {
        if (millis() - lastStep < PROFILE_STEP_MS) continue;
        lastStep = millis();

        // ── Trapezoidal profile ───────────────────────────────────────────────
        float r = profileStep();

        // ── Read encoders ─────────────────────────────────────────────────────
        int16_t raw1, raw2;
        pcnt_get_counter_value(PCNT_UNIT_0, &raw1);
        pcnt_get_counter_value(PCNT_UNIT_1, &raw2);
        float cnt1 = (float)abs(raw1);
        float cnt2 = (float)abs(raw2);

        // ── Sync correction ───────────────────────────────────────────────────
        // Positive sync → M1 is ahead of M2.
        // Inject into both error terms to push them toward each other.
        float sync = P_Ks * (cnt1 - cnt2);

        // ── Independent PID loops ─────────────────────────────────────────────
        float err1 = (r - cnt1) - sync;   // reduce M1 if it leads
        float err2 = (r - cnt2) + sync;   // boost  M2 if M1 leads

        p_m1_int = constrain(p_m1_int + err1, -200.0f, 200.0f);
        p_m2_int = constrain(p_m2_int + err2, -200.0f, 200.0f);

        float d1 = err1 - p_m1_lastErr;  p_m1_lastErr = err1;
        float d2 = err2 - p_m2_lastErr;  p_m2_lastErr = err2;

        // Feedforward gives a base PWM proportional to profile velocity so the
        // motors track without a large steady-state lag. PID corrects residual error.
        float ff   = P_kFF * p_vel;
        float pwm1 = constrain(ff          + P_kP*err1 + P_kI*p_m1_int + P_kD*d1, 0.0f, 255.0f);
        float pwm2 = constrain(ff + M2_BIAS + P_kP*err2 + P_kI*p_m2_int + P_kD*d2, 0.0f, 255.0f);

        ledcWrite(ENA_CH, (int)pwm1);
        ledcWrite(ENB_CH, (int)pwm2);

        // ── Telemetry (rate-limited to 100Hz — decoupled from 1kHz control loop) ──
        if (millis() - lastTelemetry >= 10) {
            lastTelemetry = millis();
            pkt.m1_count = (int32_t)cnt1;
            pkt.m2_count = (int32_t)cnt2;
            pkt.m1_pwm   = (int16_t)pwm1;
            pkt.m2_pwm   = (int16_t)pwm2;
            strncpy(pkt.label, "PROFILE", sizeof(pkt.label));
            espnowSend();
        }

        // ── Supervisor stop ───────────────────────────────────────────────────
        // Profile exhausted (velocity drained to zero) AND both encoders within
        // the settle window of the target — then cut power simultaneously.
        if (p_ref >= (float)PROFILE_TARGET && p_vel == 0.0f &&
            cnt1 >= (float)(PROFILE_TARGET - PROFILE_SETTLE_WIN) &&
            cnt2 >= (float)(PROFILE_TARGET - PROFILE_SETTLE_WIN)) {
            break;
        }
    }

    stopMotors();
}
#endif

// ═════════════════════════════════════════════════════════════════════════════
//  Setup & Loop
// ═════════════════════════════════════════════════════════════════════════════

void setupEspNow() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

void setup() {
    Serial.begin(115200);

    // Motor pins
    ledcSetup(ENA_CH, LEDC_FREQ, LEDC_RES); ledcAttachPin(ENA, ENA_CH);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENCODER_A1, INPUT_PULLUP);
    setupPCNT(PCNT_UNIT_0, ENCODER_A1);

    ledcSetup(ENB_CH, LEDC_FREQ, LEDC_RES); ledcAttachPin(ENB, ENB_CH);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENCODER_A2, INPUT_PULLUP);
    setupPCNT(PCNT_UNIT_1, ENCODER_A2);

    // Sensor pins
    analogReadResolution(12);
    analogSetPinAttenuation(IR_L_PIN, ADC_11db);
    analogSetPinAttenuation(IR_R_PIN, ADC_11db);
    pinMode(IR_F_PIN, INPUT);

    setupEspNow();

    memset(&pkt, 0, sizeof(pkt));

    Serial.println("Mouse ready.");

#ifdef TEST_FORWARD
    Serial.println("Mode: FORWARD (open-loop verification)");
    Serial.println("Place robot on the floor — it will drive forward for 2.5s.");
    delay(3000);
    runForwardTest();
    while(true);
#endif

#ifdef TEST_STRAIGHT
    Serial.println("Mode: STRAIGHT");
    delay(2000);
    runStraightTest();
    stopMotors();
    Serial.println("Done.");
    while(true);
#endif

#ifdef TEST_TURN
    Serial.println("Mode: TURN");
    delay(2000);
    turn90();
    Serial.println("Done.");
    while(true);
#endif

#ifdef TEST_CHARACTERIZE
    Serial.println("Mode: CHARACTERIZE");
    Serial.println("Place robot on a clear surface or elevated so wheels spin freely.");
    delay(3000);
    runCharacterizeTest();
    stopMotors();
    Serial.println("Done.");
    while(true);
#endif

#ifdef TEST_PROFILE
    Serial.println("Mode: PROFILE");
    delay(2000);
    runProfileTest();
    stopMotors();
    Serial.println("Done.");
    while(true);
#endif
}

void loop() {
#ifdef TEST_SENSORS
    runSensorTest();
    delay(100);
#endif

#ifdef TEST_ESPNOW
    runEspNowTest();
    delay(500);
#endif
}
