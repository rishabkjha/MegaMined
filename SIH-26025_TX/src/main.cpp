#include <Arduino.h>
#include "IntegratedAlgorithm.h"
#include "LoRa_Packetizer.h"
#include "LoRaManager.h"
#include "Isolation_Forest_Wrapper.h"

// Hardware Pin Definitions
#define IMU_SDA_PIN     21
#define IMU_SCL_PIN     22
#define VIBE_PIN        25
#define GPS_RX_PIN      16
#define GPS_TX_PIN      17

#define LORA_SS_PIN     5
#define LORA_RST_PIN    27
#define LORA_DIO0_PIN   26
#define LORA_FREQ_HZ    433E6 // Change to 868E6 or 433E6 if required by region

// Module Instantiations
IntegratedAlgorithm algo(IMU_SDA_PIN, IMU_SCL_PIN, VIBE_PIN, GPS_RX_PIN, GPS_TX_PIN);
LoRaPacketizer packetizer;
LoRaManager lora(LORA_FREQ_HZ, LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

// Updated 15-feature CSV header matching FEATURES in train_iso_forest.py
void logCsvHeader() {
    Serial.println(F("displacementX_mm,displacementY_mm,displacementZ_mm,"
                     "accX_mmss2,accY_mmss2,accZ_mmss2,resultantAccel,"
                     "rateRollDegS,ratePitchDegS,rateYawDegS,"
                     "angleRoll,anglePitch,angleYaw,vibrationDetected,isHighPrecision"));
}

// Prints raw SensorPacket field values as a single comma-separated row
void printCsvRow(const SensorPacket& pkt) {
    Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        pkt.displacementX_mm, pkt.displacementY_mm, pkt.displacementZ_mm,
        pkt.accX_mmss2,       pkt.accY_mmss2,       pkt.accZ_mmss2,       pkt.resultantAccel,
        pkt.rateRollDegS,     pkt.ratePitchDegS,    pkt.rateYawDegS,
        pkt.angleRoll,        pkt.anglePitch,       pkt.angleYaw,
        pkt.vibrationDetected,pkt.isHighPrecision
    );
}

// Timer for rate-limiting LoRa transmissions & CSV streaming
uint32_t lastTxTime = 0;
const uint32_t TX_INTERVAL_MS = 1000; // 1 Hz transmission rate

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // USB CDC connection delay

    Serial.println(F("--- Initializing High-Precision System ---"));

    // 1. Initialize IMU, Vibration Sensor, and GPS Telemetry
    algo.begin();

    // 2. Initialize LoRa Module
    if (!lora.begin(/*txPower=*/17, /*spreadingFactor=*/7, /*bandwidth=*/125E3)) {
        Serial.println(F("Critical Error: LoRa hardware setup failed! Halting..."));
        while (1); 
    }

    // Print CSV header so logger can verify columns if started on boot
    logCsvHeader();

    Serial.println(F("Setup complete. Streaming data & waiting for High Precision GPS..."));
}

void loop() {
    // Continuously process sensor metrics (IMU, Vibration, and GPS UART stream)
    algo.update();

    // Rate-limited LoRa Transmission & CSV Data Logging
    if (millis() - lastTxTime >= TX_INTERVAL_MS) {
        lastTxTime = millis();

        // Feed algorithm data into binary packetizer
        packetizer.packData(algo);

        SensorPacket pkt = packetizer.getPacket(); 

        // 1. Output the clean 15-feature CSV line for the serial logger
        printCsvRow(pkt);

        // 2. Run Isolation Forest inference on ESP32
        int verdict = iforestPredict(pkt);  // 1 = Normal, -1 = Anomaly
        float score = iforestScore(pkt);    // >= 0 is normal, < 0 is anomaly

        Serial.printf("[Anomaly Check] Score: %.4f | Verdict: %s\n", 
                      score, (verdict == 1) ? "NORMAL" : "ANOMALY");

        // 3. Transmit packet over LoRa
        bool txStatus = lora.sendPacket(packetizer);

        if (txStatus) {
            Serial.print(F("[LoRa TX] Size: "));
            Serial.print(packetizer.getPacketSize());
            Serial.println(F(" Byte"));
        } else {
            Serial.println(F("[LoRa TX Error]"));
        }
    }

    // Small yield to allow CPU background tasks & prevent tight loop starvation
    delay(1);
}