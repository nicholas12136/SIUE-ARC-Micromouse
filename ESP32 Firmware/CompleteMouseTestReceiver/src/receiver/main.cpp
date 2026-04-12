// ─────────────────────────────────────────────────────────────────────────────
//  Receiver firmware — prints mouse telemetry to USB serial
//
//  Flash this onto the ESP32 connected to your PC.
//  Open its serial monitor at 115200 baud to see live data from the mouse.
//
//  SETUP STEP:
//    1. Flash this receiver first.
//    2. Open the serial monitor — it will print its own MAC address on startup.
//    3. Copy that MAC into RECEIVER_MAC[] in mouse/main.cpp, then flash the mouse.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "../../src/shared/mouse_packet.h"   // shared packet definition with mouse

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW receive callback
//  Called automatically by the ESP-NOW stack whenever a packet arrives.
//  The format printed here mirrors exactly what the mouse sends —
//  no hardcoded template; every field comes directly from the received packet.
// ─────────────────────────────────────────────────────────────────────────────
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    // Ignore packets that are the wrong size (e.g. from another device)
    if (len != sizeof(MousePacket)) return;

    // Copy raw bytes into the packet struct
    MousePacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    // ── Print format matches the mouse's driveToCount telemetry ───────────────
    // Label tells us which mode the mouse is running (e.g. "DRIVE", "PROFILE")
    Serial.print("["); Serial.print(pkt.label); Serial.print("] ");

    // Encoder counts — left (L) and right (R) match mouse naming
    Serial.print("L: ");  Serial.print(pkt.m1_count);
    Serial.print("  R: "); Serial.print(pkt.m2_count);

    // PWM outputs the mouse computed for each motor
    Serial.print(" | PWM_L: "); Serial.print(pkt.m1_pwm);
    Serial.print("  PWM_R: "); Serial.print(pkt.m2_pwm);

    // Profile state — the mouse repurposes the distance fields for this:
    //   dist_left_cm  = reference position (counts)
    //   dist_right_cm = profile velocity   (counts/ms)
    Serial.print(" | Ref: ");  Serial.print(pkt.dist_left_cm,  0);
    Serial.print("  Vel: ");   Serial.print(pkt.dist_right_cm, 2);

    Serial.println();  // end of this packet's line
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // STA mode required for ESP-NOW (no router needed — direct peer-to-peer)
    WiFi.mode(WIFI_STA);

    // Print this device's MAC so you can paste it into mouse/main.cpp
    Serial.print("Receiver MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed — check board and framework version");
        return;
    }

    // Register the callback that fires whenever a packet is received
    esp_now_register_recv_cb(onReceive);

    Serial.println("Receiver ready — waiting for packets from mouse...");
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop — nothing to do here; all work happens in the ESP-NOW callback
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // ESP-NOW callbacks are interrupt-driven — no polling needed
}
