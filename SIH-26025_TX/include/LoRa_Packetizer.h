#ifndef LORA_PACKETIZER_H
#define LORA_PACKETIZER_H

#include <Arduino.h>
#include "IntegratedAlgorithm.h"

// Ensure strict byte alignment without compiler padding
#pragma pack(push, 1)
struct SensorPacket {
    // 0. Node Identification (4 bytes) - Range: 0 to 4,294,967,295
    uint32_t nodeNumber;        // Manual Node ID / Device ID (0 - 1,000,000+)

    // 1. Position Data (12 bytes)
    int32_t latScaled;          // Latitude * 1e6 (6 decimal places accuracy)
    int32_t lonScaled;          // Longitude * 1e6 (6 decimal places accuracy)
    int32_t altScaled;          // Altitude * 1e6 (6 decimal places accuracy)
    
    // 2. UTC Time Data (3 bytes)
    uint8_t utcHour;            // UTC Hour (0-23)
    uint8_t utcMinute;          // UTC Minute (0-59)
    uint8_t utcSecond;          // UTC Second (0-59)

    // 3. Displacements in mm (6 bytes)
    int16_t displacementX_mm;   // Range: -32,768 to +32,767 mm (~32m)
    int16_t displacementY_mm;   // Range: -32,768 to +32,767 mm (~32m)
    int16_t displacementZ_mm;   // Range: -32,768 to +32,767 mm (~32m)

    // 4. Accelerations in mm/s^2 (8 bytes)
    int16_t accX_mmss2;
    int16_t accY_mmss2;
    int16_t accZ_mmss2;
    uint16_t resultantAccel;

    // 5. Rotation Rates in deg/s (6 bytes)
    int16_t rateRollDegS;       // Scaled by 10 (e.g., 12.3 deg/s -> 123)
    int16_t ratePitchDegS;      // Scaled by 10
    int16_t rateYawDegS;        // Scaled by 10

    // 6. Angles in degrees (6 bytes)
    int16_t angleRoll;          // Scaled by 10 (e.g., 45.2 deg -> 452)
    int16_t anglePitch;         // Scaled by 10
    int16_t angleYaw;           // Scaled by 10

    // 7. System Status (2 bytes)
    uint8_t vibrationDetected : 1; // 1 bit flag
    uint8_t isHighPrecision   : 1; // 1 bit flag
    uint8_t isTimeValid       : 1; // 1 bit flag
    uint8_t reserved          : 5; // Padding bits for alignment
    uint8_t packetSequence;        // Rolling counter to track dropped packets
};
#pragma pack(pop)

class LoRaPacketizer {
private:
    SensorPacket _packet;
    uint8_t _sequenceNumber = 0;
    uint32_t _nodeNumber = 1; // Default Node Number

public:
    // Constructor allows setting the node number during object creation
    LoRaPacketizer(uint32_t nodeNum = 1) {
        memset(&_packet, 0, sizeof(SensorPacket));
        _nodeNumber = constrain(nodeNum, 0UL, 1000000UL); // Clamp between 0 and 1,000,000
    }

    // Setter to update node number dynamically
    void setNodeNumber(uint32_t nodeNum) {
        _nodeNumber = constrain(nodeNum, 0UL, 1000000UL);
    }

    uint32_t getNodeNumber() const {
        return _nodeNumber;
    }

    // Populate binary packet directly from IntegratedAlgorithm output
    const SensorPacket& packData(const IntegratedAlgorithm &algo) {
        // Set Node Number
        _packet.nodeNumber = _nodeNumber;

        // GPS Position Data
        _packet.latScaled = (int32_t)(algo.lat * 1e6);
        _packet.lonScaled = (int32_t)(algo.lon * 1e6);
        _packet.altScaled = (int32_t)(algo.alt * 1e6);

        // UTC Time Data
        _packet.utcHour   = algo.utcHour;
        _packet.utcMinute = algo.utcMinute;
        _packet.utcSecond = algo.utcSecond;

        // Displacements (clamped to int16_t range)
        _packet.displacementX_mm = (int16_t)constrain(algo.displacementX_mm, -32768.0f, 32767.0f);
        _packet.displacementY_mm = (int16_t)constrain(algo.displacementY_mm, -32768.0f, 32767.0f);
        _packet.displacementZ_mm = (int16_t)constrain(algo.displacementZ_mm, -32768.0f, 32767.0f);

        // Accelerations
        _packet.accX_mmss2 = (int16_t)constrain(algo.AccX_mmss2, -32768.0f, 32767.0f);
        _packet.accY_mmss2 = (int16_t)constrain(algo.AccY_mmss2, -32768.0f, 32767.0f);
        _packet.accZ_mmss2 = (int16_t)constrain(algo.AccZ_mmss2, -32768.0f, 32767.0f);
        _packet.resultantAccel = (uint16_t)constrain(algo.resultantAccel, 0.0f, 65535.0f);

        // Rotation Rates (Scaled x10 for 0.1 deg/s resolution)
        _packet.rateRollDegS  = (int16_t)constrain(algo.RateRollDegS * 10.0f, -32768.0f, 32767.0f);
        _packet.ratePitchDegS = (int16_t)constrain(algo.RatePitchDegS * 10.0f, -32768.0f, 32767.0f);
        _packet.rateYawDegS   = (int16_t)constrain(algo.RateYawDegS * 10.0f, -32768.0f, 32767.0f);

        // Angles (Scaled x10 for 0.1 deg resolution)
        _packet.angleRoll  = (int16_t)constrain(algo.AngleRoll * 10.0f, -32768.0f, 32767.0f);
        _packet.anglePitch = (int16_t)constrain(algo.AnglePitch * 10.0f, -32768.0f, 32767.0f);
        _packet.angleYaw   = (int16_t)constrain(algo.AngleYaw * 10.0f, -32768.0f, 32767.0f);

        // Status Flags & Sequence Counter
        _packet.vibrationDetected = algo.vibrationDetected ? 1 : 0;
        _packet.isHighPrecision   = algo.gpsHighPrecision;
        _packet.isTimeValid       = algo.isTimeValid ? 1 : 0;
        _packet.packetSequence    = _sequenceNumber++;

        return _packet;
    }

    // Returns pointer to raw bytes for direct buffer feed (e.g. LoRa.write)
    const uint8_t* getBuffer() const {
        return reinterpret_cast<const uint8_t*>(&_packet);
    }

    // Size of the raw packet payload in bytes
    size_t getPacketSize() const {
        return sizeof(SensorPacket);
    }

    // Returns a read-only reference to the current packed struct
    const SensorPacket& getPacket() const {
        return _packet;
    }    

    // Convenient print tool for debugging packed data
    void printPackedPacket() const {
        Serial.println(F("--- Packed LoRa Payload ---"));
        Serial.print(F("Node Number  : ")); Serial.println(_packet.nodeNumber);
        Serial.print(F("Payload Size : ")); Serial.print(getPacketSize()); Serial.println(F(" bytes"));
        Serial.print(F("Sequence No. : ")); Serial.println(_packet.packetSequence);
        Serial.print(F("Packed UTC   : ")); 
        if (_packet.isTimeValid) {
            if (_packet.utcHour < 10) Serial.print(F("0"));
            Serial.print(_packet.utcHour); Serial.print(F(":"));
            if (_packet.utcMinute < 10) Serial.print(F("0"));
            Serial.print(_packet.utcMinute); Serial.print(F(":"));
            if (_packet.utcSecond < 10) Serial.print(F("0"));
            Serial.println(_packet.utcSecond);
        } else {
            Serial.println(F("INVALID"));
        }
        Serial.print(F("Packed Lat/Lon: ")); Serial.print(_packet.latScaled); Serial.print(F(" / ")); Serial.println(_packet.lonScaled);
        Serial.print(F("Packed Disp X/Y/Z: ")); Serial.print(_packet.displacementX_mm); Serial.print(F(", "));
        Serial.print(_packet.displacementY_mm); Serial.print(F(", ")); Serial.println(_packet.displacementZ_mm);
        Serial.print(F("Packed Angles R/P/Y: ")); Serial.print(_packet.angleRoll / 10.0f); Serial.print(F(", "));
        Serial.print(_packet.anglePitch / 10.0f); Serial.print(F(", ")); Serial.println(_packet.angleYaw / 10.0f);
        Serial.print(F("Gyro Rates R/P/Y: ")); Serial.print(_packet.rateRollDegS / 10.0f); Serial.print(F(", "));
        Serial.print(_packet.ratePitchDegS / 10.0f); Serial.print(F(", ")); Serial.println(_packet.rateYawDegS / 10.0f);
        Serial.print(F("GPS High Precision: ")); Serial.println(bool(_packet.isHighPrecision));
        Serial.println(F("---------------------------"));
    }
};

#endif // LORA_PACKETIZER_H