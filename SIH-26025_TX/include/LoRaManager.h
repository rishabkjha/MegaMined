/*
#include <Arduino.h>
#include "IntegratedAlgorithm.h"
#include "LoRa_Packetizer.h"
#include "LoRaManager.h"

// Instantiate modules
IntegratedAlgorithm systemAlgo;
LoRaPacketizer packetizer;
LoRaManager lora(915E6); // 915 MHz (use 868E6 or 433E6 depending on region)

void setup() {
    Serial.begin(115200);

    systemAlgo.begin();
    
    // Single line initialization
    if (!lora.begin()) {
        while (1); // Halt on failure
    }
}

void loop() {
    // 1. Process algorithm metrics
    systemAlgo.update();

    // 2. Feed data to packetizer and send over LoRa (1-liner statement)
    static uint32_t txTimer = 0;
    if (millis() - txTimer >= 1000) { // 1 Hz Tx rate
        txTimer = millis();

        // Feed & Send in 2 clean lines
        packetizer.packData(systemAlgo);
        lora.sendPacket(packetizer);
    }
}
*/

#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "LoRa_Packetizer.h"

class LoRaManager {
private:
    uint8_t _ssPin;
    uint8_t _resetPin;
    uint8_t _dio0Pin;
    long _frequency;

public:
    // Default constructor for ESP32 standard LoRa pinout (915 MHz default)
    LoRaManager(long frequency = 915E6, uint8_t ssPin = 5, uint8_t resetPin = 14, uint8_t dio0Pin = 2)
        : _frequency(frequency), _ssPin(ssPin), _resetPin(resetPin), _dio0Pin(dio0Pin) {}

    // Initialize SPI pins & LoRa hardware
    bool begin(int txPower = 17, int spreadingFactor = 7, long signalBandwidth = 125E3) {
        LoRa.setPins(_ssPin, _resetPin, _dio0Pin);

        if (!LoRa.begin(_frequency)) {
            Serial.println(F("[LoRa Error] Hardware initialization failed! Check wiring."));
            return false;
        }

        // Configure LoRa RF Parameters
        LoRa.setTxPower(txPower);                 // 2 to 20 dBm
        LoRa.setSpreadingFactor(spreadingFactor); // 6 to 12
        LoRa.setSignalBandwidth(signalBandwidth); // 125E3, 250E3, 500E3
        LoRa.setCodingRate4(5);                   // 4/5 coding rate

        Serial.println(F("[LoRa] Module initialized successfully."));
        return true;
    }

    // Direct 1-liner to feed packetizer and transmit over LoRa
    bool sendPacket(const LoRaPacketizer &packetizer) {
        return sendBuffer(packetizer.getBuffer(), packetizer.getPacketSize());
    }

    // Low-level buffer transmitter
    bool sendBuffer(const uint8_t *buffer, size_t size) {
        if (!buffer || size == 0) return false;

        if (LoRa.beginPacket()) {
            LoRa.write(buffer, size);
            bool success = LoRa.endPacket(); // Transmits synchronously
            return success;
        }
        return false;
    }
};

#endif // LORA_MANAGER_H