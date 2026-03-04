#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "../../src/shared/mouse_packet.h"

// ─── Receiver ─────────────────────────────────────────────────────────────────
// Flash this onto the ESP32 connected to your PC via USB.
// Open its serial monitor at 115200 to see all data from the mouse.
//
// SETUP STEP: Before flashing the mouse firmware, open the serial monitor
// on this receiver and note the MAC address it prints. Paste that MAC into
// RECEIVER_MAC in mouse/main.cpp.

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(MousePacket)) return;

    MousePacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    Serial.print("["); Serial.print(pkt.label); Serial.print("] ");

    Serial.print("M1: "); Serial.print(pkt.m1_count);
    Serial.print(" M2: "); Serial.print(pkt.m2_count);
    Serial.print(" | PWM1: "); Serial.print(pkt.m1_pwm);
    Serial.print(" PWM2: "); Serial.print(pkt.m2_pwm);
    Serial.print(" | L: ");
    if (pkt.dist_left_cm < 0) Serial.print("INV");
    else { Serial.print(pkt.dist_left_cm, 1); Serial.print("cm"); }
    Serial.print(" R: ");
    if (pkt.dist_right_cm < 0) Serial.print("INV");
    else { Serial.print(pkt.dist_right_cm, 1); Serial.print("cm"); }
    Serial.print(" | Front: "); Serial.print(pkt.front_blocked ? "WALL" : "CLEAR");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    Serial.print("Receiver MAC: ");
    Serial.println(WiFi.macAddress());  // <-- copy this into mouse/main.cpp

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    Serial.println("Receiver ready, waiting for packets...");
}

void loop() {
    // Nothing to do — ESP-NOW callback handles everything
}
