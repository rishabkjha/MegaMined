#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include "LoRa_Decoder.h"
#include "Supabase_Client.h"

// --- Wi-Fi & Supabase Configuration ---
const char* WIFI_SSID       = "Aviraj's Phone";
const char* WIFI_PASSWORD   = "sarabha3";

// Example format: "https://xyzcompany.supabase.co"
const char* SUPABASE_URL    = "https://umhkwsyytcweuoftyhuc.supabase.co";
const char* SUPABASE_KEY    = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InVtaGt3c3l5dGN3ZXVvZnR5aHVjIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODk4MjAwNzUsImV4cCI6MjEwNTM5NjA3NX0.GJgUW65nteoBPTZ7hYndaauwxaX3Vv4bEzzQ8sqGKPI";

// --- Hardware Pin Configurations ---
#define LORA_SS_PIN     5
#define LORA_RST_PIN    27
#define LORA_DIO0_PIN   26
#define LORA_FREQ_HZ    433E6 

#define RX_GPS_RX_PIN   16
#define RX_GPS_TX_PIN   17
#define RX_GPS_BAUD     9600

// --- Class Instances ---
HardwareSerial gpsSerial(2);
TinyGPSPlus rxGps;
LoRaDecoder decoder;
SupabaseClient supabase(WIFI_SSID, WIFI_PASSWORD, SUPABASE_URL, SUPABASE_KEY);

void processLocalGPS() {
    while (gpsSerial.available() > 0) {
        rxGps.encode(gpsSerial.read());
    }
}

void checkIncomingLoRa() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    uint8_t rxBuffer[sizeof(SensorPacket)];
    
    if (packetSize == decoder.getExpectedSize()) {
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < packetSize) {
            rxBuffer[bytesRead++] = (uint8_t)LoRa.read();
        }

        if (decoder.parseBuffer(rxBuffer, bytesRead)) {
            int rssi = LoRa.packetRssi();
            float snr = LoRa.packetSnr();

            // 1. Output to local serial
            decoder.printTelemetry(rssi, snr);

            // 2. Extract decoded struct
            DecodedTelemetry telemetry = decoder.getTelemetry();

            // 3. Send API Request to Supabase
            double rxLat = rxGps.location.isValid() ? rxGps.location.lat() : 0.0;
            double rxLon = rxGps.location.isValid() ? rxGps.location.lng() : 0.0;

            supabase.sendTelemetryToSupabase(telemetry, rxLat, rxLon);
        } else {
            Serial.println(F("[LoRa RX Error] Packet memory structure mismatch!"));
        }
    } else {
        while (LoRa.available()) LoRa.read();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("--- Initializing LoRa RX + GPS + Supabase Node ---"));

    // 1. Initialize Wi-Fi
    supabase.beginWiFi();

    // 2. Initialize Local GPS
    gpsSerial.begin(RX_GPS_BAUD, SERIAL_8N1, RX_GPS_RX_PIN, RX_GPS_TX_PIN);

    // 3. Initialize LoRa
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQ_HZ)) {
        Serial.println(F("Critical Error: LoRa setup failed!"));
        while (1);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);

    Serial.println(F("[LoRa] Module active. Listening for telemetry..."));
}

void loop() {
    processLocalGPS();
    checkIncomingLoRa();
    delay(1);
}
