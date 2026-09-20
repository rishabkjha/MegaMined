/*
#include "GPS_Telemetry.h"

// Hardware Pin Definitions
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 115200

// Initialize object with custom pins/baud
GPSTelemetry gpsNode(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

void setup() {
    Serial.begin(115200);
    gpsNode.begin();
}

void loop() {
    // Non-blocking update cycle
    if (gpsNode.update()) {
        gpsNode.runAlgorithms();
        gpsNode.printData();
        
        // Example: Direct standard access like lat/lng
        String currentTime = gpsNode.data.getFormattedTime(); 
    }

    // Hardware connection verification
    gpsNode.checkWatchdog();
}
*/

#ifndef GPS_TELEMETRY_H
#define GPS_TELEMETRY_H

#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>

using namespace BLA;

// Telemetry State Container
struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double heading = 0.0;

    float hdop = 99.9f;
    float vdop = 99.9f;
    float pdop = 99.9f;

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

    bool isLocationValid = false;
    bool isAltitudeValid = false;
    bool isHeadingValid = false;
    bool isHdopValid = false;
    bool isVdopValid = false;
    bool isPdopValid = false;
    bool isTimeValid = false;
    bool isHighPrecision = false;

    // Helper method for downstream algorithms
    inline bool hasValidFix() const {
        return isLocationValid && isHdopValid && (hdop <= 3.0f);
    }

    // Helper method to get formatted UTC time string directly
    String getFormattedTime() const {
        if (!isTimeValid) return "INVALID";
        char timeBuf[9]; // HH:MM:SS + null terminator
        snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", hour, minute, second);
        return String(timeBuf);
    }
};

class GPSTelemetry {
private:
    uint8_t _rxPin;
    uint8_t _txPin;
    uint32_t _baudRate;
    
    HardwareSerial _gpsSerial;
    TinyGPSPlus _gps;
    
    // Custom NMEA Parsers for GPGSA fields
    TinyGPSCustom _vdopCustom;
    TinyGPSCustom _pdopCustom;

    bool checkHighPrecisionAccuracy(float hdop, float vdop, float pdop, bool hValid, bool vValid, bool pValid) {
        if (!hValid || !vValid || !pValid) return false;
        return (hdop > 0.0f && hdop <= 1.5f) &&
               (vdop > 0.0f && vdop <= 1.5f) &&
               (pdop > 0.0f && pdop <= 2.0f);
    }

    void updateGPSData() {
        // Location
        data.isLocationValid = _gps.location.isValid();
        data.latitude = data.isLocationValid ? _gps.location.lat() : 0.0;
        data.longitude = data.isLocationValid ? _gps.location.lng() : 0.0;

        // Altitude & Heading
        data.isAltitudeValid = _gps.altitude.isValid();
        data.altitude = data.isAltitudeValid ? _gps.altitude.meters() : 0.0;

        data.isHeadingValid = _gps.course.isValid();
        data.heading = data.isHeadingValid ? _gps.course.deg() : 0.0;

        // DOP Metrics
        data.isHdopValid = _gps.hdop.isValid();
        data.hdop = data.isHdopValid ? _gps.hdop.hdop() : 99.9f;

        data.isVdopValid = (_vdopCustom.isValid() && strlen(_vdopCustom.value()) > 0);
        data.vdop = data.isVdopValid ? atof(_vdopCustom.value()) : 99.9f;

        data.isPdopValid = (_pdopCustom.isValid() && strlen(_pdopCustom.value()) > 0);
        data.pdop = data.isPdopValid ? atof(_pdopCustom.value()) : 99.9f;

        // Evaluate Precision Status
        data.isHighPrecision = checkHighPrecisionAccuracy(
            data.hdop, data.vdop, data.pdop,
            data.isHdopValid, data.isVdopValid, data.isPdopValid
        );

        // UTC Time
        data.isTimeValid = _gps.time.isValid();
        if (data.isTimeValid) {
            data.hour = _gps.time.hour();
            data.minute = _gps.time.minute();
            data.second = _gps.time.second();
        }
    }

public:
    GPSData data;

    // Constructor initializes UART instance and NMEA custom field objects
    GPSTelemetry(uint8_t rxPin = 16, uint8_t txPin = 17, uint32_t baudRate = 115200, uint8_t uartNum = 2)
        : _rxPin(rxPin), _txPin(txPin), _baudRate(baudRate), _gpsSerial(uartNum),
          _vdopCustom(_gps, "GPGSA", 16), _pdopCustom(_gps, "GPGSA", 15) {}

    // Initialize GPS UART hardware
    void begin() {
        _gpsSerial.begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
    }

    bool update() {
        bool updated = false;
        while (_gpsSerial.available() > 0) {
            if (_gps.encode(_gpsSerial.read())) {
                updated = true;
            }
        }

        if (updated) {
            updateGPSData();
        }

        return updated;
    }

    void runAlgorithms() {
        if (!data.hasValidFix()) {
            return;
        }

        // Basic Linear Algebra (BLA) Matrix operations
        BLA::Matrix<2, 1> posVector = {data.latitude, data.longitude};

        if (data.isHighPrecision) {
            // High confidence state updates
        }
    }

    void printData() const {
        Serial.println(F("--- GPS Telemetry ---"));

        if (data.isLocationValid) {
            Serial.print(F("Lat/Lng/Alt    : "));
            Serial.print(data.latitude, 6);
            Serial.print(F(", "));
            Serial.print(data.longitude, 6);
            Serial.print(F(", "));
            Serial.println(data.altitude, 2);
        } else {
            Serial.println(F("Location       : INVALID"));
        }

        Serial.print(F("UTC Time       : "));
        Serial.println(data.getFormattedTime());

        Serial.print(F("DOP (H/V/P)    : "));
        if (data.isHdopValid) Serial.print(data.hdop, 1); else Serial.print(F("N/A"));
        Serial.print(F(" / "));
        if (data.isVdopValid) Serial.print(data.vdop, 1); else Serial.print(F("N/A"));
        Serial.print(F(" / "));
        if (data.isPdopValid) Serial.println(data.pdop, 1); else Serial.println(F("N/A"));

        Serial.print(F("High Precision : "));
        Serial.println(data.isHighPrecision ? F("TRUE (~1-2m)") : F("FALSE"));
        Serial.println();
    }

    void checkWatchdog() {
        if (millis() > 5000 && _gps.charsProcessed() < 10) {
            Serial.println(F("Error: No GPS hardware detected! Check wiring."));
            delay(5000);
        }
    }
};

#endif // GPS_TELEMETRY_H