#include <Arduino.h>
#include "driver/pcnt.h"
#include <SPIFFS.h>

// ─── Pin definitions ─────────────────────────────────────────────────────────
#define ENA        18
#define IN1        19
#define IN2        21
#define ENCODER_A1 22

#define ENB        25
#define IN3        13
#define IN4        14
#define ENCODER_A2 26

#define LEDC_FREQ  1000
#define LEDC_RES   8

// ─── Logging ─────────────────────────────────────────────────────────────────
#define LOG_MS     10   // 100 Hz

// ─── Impulse parameters ──────────────────────────────────────────────────────
#define IMPULSE_HOLD_MS    1000
#define IMPULSE_STABLE_MS   500
#define IMPULSE_PWM_MIN      20
#define IMPULSE_PWM_MAX     255
#define IMPULSE_PWM_STEP     10
#define IMPULSE_REPS          3

// ─── Steady-state parameters ─────────────────────────────────────────────────
#define SS_HOLD_MS         5000   // total hold time
#define SS_DISCARD_MS      1000   // throw away first second
#define SS_STABLE_MS        500
#define SS_PWM_MIN           20
#define SS_PWM_MAX          255
#define SS_PWM_STEP          20
#define SS_REPS               3

// ─── Brake test parameters ───────────────────────────────────────────────────
// Cruise at these PWMs, then test coast vs active brake
const int BRAKE_PWM_LEVELS[] = {60, 100, 150, 200, 255};
#define BRAKE_NUM_LEVELS    5
#define BRAKE_CRUISE_MS   2000   // spin up to steady state
#define BRAKE_STABLE_MS   1000   // wait for full stop
#define BRAKE_REPS           3

// ─── Reverse parameters ──────────────────────────────────────────────────────
#define REV_HOLD_MS        1000
#define REV_STABLE_MS       500
#define REV_PWM_MIN          20
#define REV_PWM_MAX         255
#define REV_PWM_STEP         10

// ─── Acceleration ramp parameters ────────────────────────────────────────────
const unsigned long RAMP_DURATIONS[] = {20000, 10000, 5000};
#define NUM_RAMPS 3
#define RAMP_STABLE_MS 500

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
    ledcWrite(ENA, 0); ledcWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void brakeMotors(bool m1, bool m2) {
    if (m1) { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); ledcWrite(ENA, 255); }
    if (m2) { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); ledcWrite(ENB, 255); }
}

void coastMotors(bool m1, bool m2) {
    if (m1) { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); ledcWrite(ENA, 0); }
    if (m2) { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); ledcWrite(ENB, 0); }
}

void readEncoders(int32_t &e1, int32_t &e2) {
    int16_t r1, r2;
    pcnt_get_counter_value(PCNT_UNIT_0, &r1);
    pcnt_get_counter_value(PCNT_UNIT_1, &r2);
    e1 = abs(r1);
    e2 = abs(r2);
}

void setMotorFwd(int pwm, bool m1, bool m2) {
    if (m1) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(ENA, pwm); }
    if (m2) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(ENB, pwm); }
}

void setMotorRev(int pwm, bool m1, bool m2) {
    if (m1) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(ENA, pwm); }
    if (m2) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(ENB, pwm); }
}

// Wait until active encoders stable, logging throughout
void waitStopped(File &f, unsigned long t0, bool m1, bool m2,
                 unsigned long stableMs, int extraCol = -1) {
    int32_t prev1 = -1, prev2 = -1;
    unsigned long stableStart = millis();
    unsigned long lastLog = 0;

    while (millis() - stableStart < stableMs) {
        unsigned long now = millis();
        if (now - lastLog >= LOG_MS) {
            lastLog = now;
            int32_t e1, e2;
            readEncoders(e1, e2);
            if (extraCol >= 0)
                f.printf("%lu,%d,%ld,%ld,%d\n", now - t0, 0, e1, e2, extraCol);
            else
                f.printf("%lu,%d,%ld,%ld\n", now - t0, 0, e1, e2);

            bool changed = false;
            if (m1 && e1 != prev1) changed = true;
            if (m2 && e2 != prev2) changed = true;
            prev1 = e1; prev2 = e2;
            if (changed) stableStart = millis();
        }
    }
}

// Build PWM step list with guaranteed 255 at end
int buildPWMList(int *list, int minPwm, int maxPwm, int step) {
    int n = 0;
    for (int p = minPwm; p <= maxPwm; p += step)
        list[n++] = p;
    if (list[n - 1] != maxPwm)
        list[n++] = maxPwm;
    return n;
}

// Dump file over serial then delete
void dumpAndDelete(const char *path, const char *label) {
    Serial.printf("\n>>>START:%s\n", label);
    File f = SPIFFS.open(path, FILE_READ);
    if (!f) { Serial.printf("ERROR: no %s\n", path); Serial.printf(">>>END:%s\n", label); return; }
    while (f.available()) Serial.write(f.read());
    f.close();
    SPIFFS.remove(path);
    Serial.printf(">>>END:%s\n\n", label);
}

// ═════════════════════════════════════════════════════════════════════════════
//  1. IMPULSE TEST — 3 reps per PWM step
// ═════════════════════════════════════════════════════════════════════════════
void runImpulse(const char *path, bool m1, bool m2, const char *label) {
    Serial.printf("Running %s ...\n", label);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.printf("ERROR: cant open %s\n", path); return; }
    f.println("timestamp_ms,pwm,enc_left,enc_right,rep");

    int pwmList[30]; int pwmCount = buildPWMList(pwmList, IMPULSE_PWM_MIN, IMPULSE_PWM_MAX, IMPULSE_PWM_STEP);

    for (int idx = 0; idx < pwmCount; idx++) {
        int pwm = pwmList[idx];
        for (int rep = 1; rep <= IMPULSE_REPS; rep++) {
            pcnt_counter_clear(PCNT_UNIT_0);
            pcnt_counter_clear(PCNT_UNIT_1);
            unsigned long t0 = millis(), lastLog = 0;

            setMotorFwd(pwm, m1, m2);
            while (millis() - t0 < IMPULSE_HOLD_MS) {
                unsigned long now = millis();
                if (now - lastLog >= LOG_MS) {
                    lastLog = now;
                    int32_t e1, e2; readEncoders(e1, e2);
                    f.printf("%lu,%d,%ld,%ld,%d\n", now - t0, pwm, e1, e2, rep);
                }
            }
            stopMotors();
            waitStopped(f, t0, m1, m2, IMPULSE_STABLE_MS, rep);
        }
    }
    f.close();
    Serial.printf("  %s complete\n", label);
    dumpAndDelete(path, label);
}

// ═════════════════════════════════════════════════════════════════════════════
//  2. STEADY-STATE SPEED — 3 reps, hold 5s, log last 4s
// ═════════════════════════════════════════════════════════════════════════════
void runSteadyState(const char *path, bool m1, bool m2, const char *label) {
    Serial.printf("Running %s ...\n", label);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.printf("ERROR: cant open %s\n", path); return; }
    f.println("timestamp_ms,pwm,enc_left,enc_right,rep");

    int pwmList[20]; int pwmCount = buildPWMList(pwmList, SS_PWM_MIN, SS_PWM_MAX, SS_PWM_STEP);

    for (int idx = 0; idx < pwmCount; idx++) {
        int pwm = pwmList[idx];
        for (int rep = 1; rep <= SS_REPS; rep++) {
            pcnt_counter_clear(PCNT_UNIT_0);
            pcnt_counter_clear(PCNT_UNIT_1);
            unsigned long t0 = millis(), lastLog = 0;

            setMotorFwd(pwm, m1, m2);

            // Discard phase — motor spins up but we don't log
            while (millis() - t0 < SS_DISCARD_MS) { delayMicroseconds(100); }

            // Logging phase — steady-state data
            while (millis() - t0 < SS_HOLD_MS) {
                unsigned long now = millis();
                if (now - lastLog >= LOG_MS) {
                    lastLog = now;
                    int32_t e1, e2; readEncoders(e1, e2);
                    f.printf("%lu,%d,%ld,%ld,%d\n", now - t0, pwm, e1, e2, rep);
                }
            }
            stopMotors();
            waitStopped(f, t0, m1, m2, SS_STABLE_MS, rep);
        }
    }
    f.close();
    Serial.printf("  %s complete\n", label);
    dumpAndDelete(path, label);
}

// ═════════════════════════════════════════════════════════════════════════════
//  3. BRAKING TEST — coast vs active brake, 3 reps each
// ═════════════════════════════════════════════════════════════════════════════
void runBrake(const char *path, bool m1, bool m2, const char *label) {
    Serial.printf("Running %s ...\n", label);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.printf("ERROR: cant open %s\n", path); return; }
    f.println("timestamp_ms,cruise_pwm,enc_left,enc_right,brake_type,rep");

    for (int lvl = 0; lvl < BRAKE_NUM_LEVELS; lvl++) {
        int pwm = BRAKE_PWM_LEVELS[lvl];
        for (int brakeType = 0; brakeType <= 1; brakeType++) {
            // 0 = coast, 1 = active brake
            const char *bLabel = brakeType == 0 ? "coast" : "brake";
            for (int rep = 1; rep <= BRAKE_REPS; rep++) {
                pcnt_counter_clear(PCNT_UNIT_0);
                pcnt_counter_clear(PCNT_UNIT_1);

                // Spin up to steady state
                setMotorFwd(pwm, m1, m2);
                delay(BRAKE_CRUISE_MS);

                // Mark t0 at the moment we stop/brake
                pcnt_counter_clear(PCNT_UNIT_0);
                pcnt_counter_clear(PCNT_UNIT_1);
                unsigned long t0 = millis(), lastLog = 0;

                if (brakeType == 0) coastMotors(m1, m2);
                else                brakeMotors(m1, m2);

                // Log deceleration until stopped
                int32_t prev1 = -1, prev2 = -1;
                unsigned long stableStart = millis();

                while (millis() - stableStart < BRAKE_STABLE_MS) {
                    unsigned long now = millis();
                    if (now - lastLog >= LOG_MS) {
                        lastLog = now;
                        int32_t e1, e2; readEncoders(e1, e2);
                        f.printf("%lu,%d,%ld,%ld,%s,%d\n", now - t0, pwm, e1, e2, bLabel, rep);

                        bool changed = false;
                        if (m1 && e1 != prev1) changed = true;
                        if (m2 && e2 != prev2) changed = true;
                        prev1 = e1; prev2 = e2;
                        if (changed) stableStart = millis();
                    }
                }
                stopMotors();
                delay(300);
            }
        }
    }
    f.close();
    Serial.printf("  %s complete\n", label);
    dumpAndDelete(path, label);
}

// ═════════════════════════════════════════════════════════════════════════════
//  4. REVERSE DIRECTION — same as impulse but motor runs backward
// ═════════════════════════════════════════════════════════════════════════════
void runReverse(const char *path, bool m1, bool m2, const char *label) {
    Serial.printf("Running %s ...\n", label);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.printf("ERROR: cant open %s\n", path); return; }
    f.println("timestamp_ms,pwm,enc_left,enc_right");

    int pwmList[30]; int pwmCount = buildPWMList(pwmList, REV_PWM_MIN, REV_PWM_MAX, REV_PWM_STEP);

    for (int idx = 0; idx < pwmCount; idx++) {
        int pwm = pwmList[idx];
        pcnt_counter_clear(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_1);
        unsigned long t0 = millis(), lastLog = 0;

        setMotorRev(pwm, m1, m2);
        while (millis() - t0 < REV_HOLD_MS) {
            unsigned long now = millis();
            if (now - lastLog >= LOG_MS) {
                lastLog = now;
                int32_t e1, e2; readEncoders(e1, e2);
                f.printf("%lu,%d,%ld,%ld\n", now - t0, pwm, e1, e2);
            }
        }
        stopMotors();
        waitStopped(f, t0, m1, m2, REV_STABLE_MS);
    }
    f.close();
    Serial.printf("  %s complete\n", label);
    dumpAndDelete(path, label);
}

// ═════════════════════════════════════════════════════════════════════════════
//  5. ACCELERATION RAMP — 0→255 over 20s, 10s, 5s
// ═════════════════════════════════════════════════════════════════════════════
void runAccel(const char *path, bool m1, bool m2, const char *label) {
    Serial.printf("Running %s ...\n", label);
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.printf("ERROR: cant open %s\n", path); return; }
    f.println("timestamp_ms,pwm,enc_left,enc_right,ramp_sec");

    for (int r = 0; r < NUM_RAMPS; r++) {
        unsigned long rampMs = RAMP_DURATIONS[r];
        int rampSec = (int)(rampMs / 1000);

        pcnt_counter_clear(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_1);
        unsigned long t0 = millis(), lastLog = 0;

        Serial.printf("  ramp %ds ...\n", rampSec);

        while (millis() - t0 < rampMs) {
            unsigned long now = millis();
            unsigned long elapsed = now - t0;
            int pwm = (int)((float)elapsed / (float)rampMs * 255.0f);
            if (pwm > 255) pwm = 255;

            setMotorFwd(pwm, m1, m2);
            if (now - lastLog >= LOG_MS) {
                lastLog = now;
                int32_t e1, e2; readEncoders(e1, e2);
                f.printf("%lu,%d,%ld,%ld,%d\n", elapsed, pwm, e1, e2, rampSec);
            }
        }
        stopMotors();
        waitStopped(f, t0, m1, m2, RAMP_STABLE_MS, rampSec);
    }
    f.close();
    Serial.printf("  %s complete\n", label);
    dumpAndDelete(path, label);
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    if (!SPIFFS.begin(true)) { Serial.println("SPIFFS mount failed"); return; }

    ledcSetup(ENA, LEDC_FREQ, LEDC_RES);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENCODER_A1, INPUT_PULLUP);
    setupPCNT(PCNT_UNIT_0, ENCODER_A1);

    ledcSetup(ENB, LEDC_FREQ, LEDC_RES);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENCODER_A2, INPUT_PULLUP);
    setupPCNT(PCNT_UNIT_1, ENCODER_A2);

    stopMotors();

    Serial.println("=== MOTOR CHARACTERIZATION SUITE ===");
    Serial.println("Elevate robot or place on clear surface.");
    Serial.println("Capture serial output to split into CSVs by >>>START/>>>END markers.");
    Serial.println();
    Serial.println("Tests: impulse(×3 reps) → steady-state(×3) → brake(coast+active,×3) → reverse → ramp");
    Serial.println("Starting in 5 seconds...");
    Serial.println();
    delay(5000);

    // ── 1. Impulse (3 reps per step) ─────────────────────────────────────────
    runImpulse("/tmp.csv", true,  false, "motor_impulse_left.csv");    delay(1000);
    runImpulse("/tmp.csv", false, true,  "motor_impulse_right.csv");   delay(1000);
    runImpulse("/tmp.csv", true,  true,  "motor_impulse_both.csv");    delay(1000);

    // ── 2. Steady-state speed (3 reps per step) ─────────────────────────────
    runSteadyState("/tmp.csv", true,  false, "motor_steady_left.csv");  delay(1000);
    runSteadyState("/tmp.csv", false, true,  "motor_steady_right.csv"); delay(1000);
    runSteadyState("/tmp.csv", true,  true,  "motor_steady_both.csv");  delay(1000);

    // ── 3. Braking: coast vs active brake (3 reps each) ─────────────────────
    runBrake("/tmp.csv", true,  false, "motor_brake_left.csv");  delay(1000);
    runBrake("/tmp.csv", false, true,  "motor_brake_right.csv"); delay(1000);
    runBrake("/tmp.csv", true,  true,  "motor_brake_both.csv");  delay(1000);

    // ── 4. Reverse direction ─────────────────────────────────────────────────
    runReverse("/tmp.csv", true,  false, "motor_reverse_left.csv");  delay(1000);
    runReverse("/tmp.csv", false, true,  "motor_reverse_right.csv"); delay(1000);

    // ── 5. Acceleration ramps ────────────────────────────────────────────────
    runAccel("/tmp.csv", true,  false, "motor_acc_left.csv");  delay(1000);
    runAccel("/tmp.csv", false, true,  "motor_acc_right.csv"); delay(1000);
    runAccel("/tmp.csv", true,  true,  "motor_acc_both.csv");

    stopMotors();
    Serial.println();
    Serial.println("=== ALL TESTS COMPLETE ===");
}

void loop() {}
