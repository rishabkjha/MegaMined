#ifndef LORA_DECODER_H
#define LORA_DECODER_H

#include <Arduino.h>

// Strict 1-byte alignment matching the TX structure layout exactly
#pragma pack(push, 1)
struct SensorPacket {
    uint32_t nodeNumber;        // 4 bytes
    int32_t latScaled;          // 4 bytes (1e6)
    int32_t lonScaled;          // 4 bytes (1e6)
    int32_t altScaled;          // 4 bytes (1e6)
    
    uint8_t utcHour;            // 1 byte
    uint8_t utcMinute;          // 1 byte
    uint8_t utcSecond;          // 1 byte

    int16_t displacementX_mm;   // 2 bytes
    int16_t displacementY_mm;   // 2 bytes
    int16_t displacementZ_mm;   // 2 bytes

    int16_t accX_mmss2;         // 2 bytes
    int16_t accY_mmss2;         // 2 bytes
    int16_t accZ_mmss2;         // 2 bytes
    uint16_t resultantAccel;    // 2 bytes

    int16_t rateRollDegS;       // 2 bytes (x10)
    int16_t ratePitchDegS;      // 2 bytes (x10)
    int16_t rateYawDegS;        // 2 bytes (x10)

    int16_t angleRoll;          // 2 bytes (x10)
    int16_t anglePitch;         // 2 bytes (x10)
    int16_t angleYaw;           // 2 bytes (x10)

    uint8_t vibrationDetected : 1;
    uint8_t isHighPrecision   : 1;
    uint8_t isTimeValid       : 1;
    uint8_t reserved          : 5;
    uint8_t packetSequence;     // 1 byte
};
#pragma pack(pop)

// Decoded engineering-unit payload structure
struct DecodedTelemetry {
    uint32_t nodeNumber;
    double latitude;
    double longitude;
    double altitude;
    
    uint8_t utcHour;
    uint8_t utcMinute;
    uint8_t utcSecond;

    float displacementX_m;
    float displacementY_m;
    float displacementZ_m;

    float accX_ms2;
    float accY_ms2;
    float accZ_ms2;
    float resultantAccel_ms2;

    float rateRoll;
    float ratePitch;
    float rateYaw;

    float angleRoll;
    float anglePitch;
    float angleYaw;

    bool vibrationDetected;
    bool isHighPrecision;
    bool isTimeValid;
    uint8_t packetSequence;
};

class LoRaDecoder {
private:
    SensorPacket _rawPacket;

public:
    LoRaDecoder() {
        memset(&_rawPacket, 0, sizeof(SensorPacket));
    }

    constexpr size_t getExpectedSize() const {
        return sizeof(SensorPacket);
    }

    // Unpacks binary stream from LoRa payload buffer
    bool parseBuffer(const uint8_t* buffer, size_t size) {
        if (size != sizeof(SensorPacket) || buffer == nullptr) {
            return false;
        }
        memcpy(&_rawPacket, buffer, sizeof(SensorPacket));
        return true;
    }

    // Converts raw binary packet metrics to standard floating-point variables
    DecodedTelemetry getTelemetry() const {
        DecodedTelemetry data;

        data.nodeNumber = _rawPacket.nodeNumber;
        data.latitude   = _rawPacket.latScaled / 1e6;
        data.longitude  = _rawPacket.lonScaled / 1e6;
        data.altitude   = _rawPacket.altScaled / 1e6;

        data.utcHour   = _rawPacket.utcHour;
        data.utcMinute = _rawPacket.utcMinute;
        data.utcSecond = _rawPacket.utcSecond;

        data.displacementX_m = _rawPacket.displacementX_mm / 1000.0f;
        data.displacementY_m = _rawPacket.displacementY_mm / 1000.0f;
        data.displacementZ_m = _rawPacket.displacementZ_mm / 1000.0f;

        data.accX_ms2           = _rawPacket.accX_mmss2 / 1000.0f;
        data.accY_ms2           = _rawPacket.accY_mmss2 / 1000.0f;
        data.accZ_ms2           = _rawPacket.accZ_mmss2 / 1000.0f;
        data.resultantAccel_ms2 = _rawPacket.resultantAccel / 1000.0f;

        data.rateRoll  = _rawPacket.rateRollDegS / 10.0f;
        data.ratePitch = _rawPacket.ratePitchDegS / 10.0f;
        data.rateYaw   = _rawPacket.rateYawDegS / 10.0f;

        data.angleRoll  = _rawPacket.angleRoll / 10.0f;
        data.anglePitch = _rawPacket.anglePitch / 10.0f;
        data.angleYaw   = _rawPacket.angleYaw / 10.0f;

        data.vibrationDetected = _rawPacket.vibrationDetected;
        data.isHighPrecision   = _rawPacket.isHighPrecision;
        data.isTimeValid       = _rawPacket.isTimeValid;
        data.packetSequence    = _rawPacket.packetSequence;

        return data;
    }

    void printTelemetry(int rssi, float snr) const {
        DecodedTelemetry t = getTelemetry();

        Serial.println(F("\n================= RX TELEMETRY PACKET ================="));
        Serial.print(F("Node ID           : ")); Serial.println(t.nodeNumber);
        Serial.print(F("Sequence No.      : ")); Serial.println(t.packetSequence);
        Serial.print(F("Signal Status     : RSSI = ")); Serial.print(rssi);
        Serial.print(F(" dBm | SNR = ")); Serial.print(snr); Serial.println(F(" dB"));

        Serial.print(F("TX GPS UTC Time   : "));
        if (t.isTimeValid) {
            if (t.utcHour < 10) Serial.print('0'); Serial.print(t.utcHour); Serial.print(':');
            if (t.utcMinute < 10) Serial.print('0'); Serial.print(t.utcMinute); Serial.print(':');
            if (t.utcSecond < 10) Serial.print('0'); Serial.println(t.utcSecond);
        } else {
            Serial.println(F("INVALID"));
        }

        Serial.print(F("TX GPS Coordinates: Lat: ")); Serial.print(t.latitude, 6);
        Serial.print(F(" | Lon: ")); Serial.print(t.longitude, 6);
        Serial.print(F(" | Alt: ")); Serial.print(t.altitude, 2); Serial.println(F(" m"));

        Serial.print(F("Displacement (m)  : X: ")); Serial.print(t.displacementX_m, 3);
        Serial.print(F(" | Y: ")); Serial.print(t.displacementY_m, 3);
        Serial.print(F(" | Z: ")); Serial.println(t.displacementZ_m, 3);

        Serial.print(F("Acceleration(m/s²): X: ")); Serial.print(t.accX_ms2, 2);
        Serial.print(F(" | Y: ")); Serial.print(t.accY_ms2, 2);
        Serial.print(F(" | Z: ")); Serial.print(t.accZ_ms2, 2);
        Serial.print(F(" | Mag: ")); Serial.println(t.resultantAccel_ms2, 2);

        Serial.print(F("Angles (deg)      : Roll: ")); Serial.print(t.angleRoll, 1);
        Serial.print(F(" | Pitch: ")); Serial.print(t.anglePitch, 1);
        Serial.print(F(" | Yaw: ")); Serial.println(t.angleYaw, 1);

        Serial.print(F("System Flags      : HighPrec=")); Serial.print(t.isHighPrecision ? "YES" : "NO");
        Serial.print(F(" | VibeAlert=")); Serial.println(t.vibrationDetected ? "YES" : "NO");
        Serial.println(F("=======================================================\n"));
    }
};

#endif // LORA_DECODER_H