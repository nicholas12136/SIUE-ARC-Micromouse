#pragma once
#include <stdint.h>

// ─── Shared ESP-NOW packet ────────────────────────────────────────────────────
// Both mouse and receiver include this header so the struct stays in sync.

struct MousePacket {
    // --- Motor counts (raw encoder values) ---
    int32_t m1_count;
    int32_t m2_count;

    // --- Motor PWM outputs ---
    int16_t m1_pwm;
    int16_t m2_pwm;

    // --- Sensor readings ---
    int16_t  adc_left;       // raw ADC
    int16_t  adc_right;
    float    dist_left_cm;   // converted distance
    float    dist_right_cm;
    uint8_t  front_blocked;  // 1 = wall detected, 0 = clear

    // --- State label (which test mode is running) ---
    char label[16];          // e.g. "STRAIGHT", "TURN", "SENSORS"
};
