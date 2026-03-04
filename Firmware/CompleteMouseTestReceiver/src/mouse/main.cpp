#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/pcnt.h"
#include "../shared/mouse_packet.h"

// ─────────────────────────────────────────────────────────────────────────────
//  TEST MODE — uncomment exactly ONE
// ─────────────────────────────────────────────────────────────────────────────
 #define TEST_STRAIGHT      // PID straight-line drive
// #define TEST_TURN           // encoder-based turn
// #define TEST_SENSORS        // IR sensor readings only
// #define TEST_ESPNOW         // ESP-NOW link check (no motors/sensors)
// ─────────────────────────────────────────────────────────────────────────────

// ─── ESP-NOW ─────────────────────────────────────────────────────────────────
// SETUP STEP: flash receiver first, note its MAC from serial monitor, paste here
uint8_t RECEIVER_MAC[] = {0x38, 0x18, 0x2B, 0x8A, 0x1D, 0x0C}; //  38:18:2B:8A:1D:0C

MousePacket pkt;

void espnowSend() {
    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

// ─── Pin definitions ─────────────────────────────────────────────────────────
#define ENA        18
#define IN1        19
#define IN2        21
#define ENCODER_A1 22

#define ENB        25
#define IN3        13
#define IN4        14
#define ENCODER_A2 26

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
