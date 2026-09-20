/*
#include <Arduino.h>
#include "IntegratedAlgorithm.h"

// Define pin configurations
#define IMU_SDA_PIN    21
#define IMU_SCL_PIN    22
#define VIBE_PIN       25
#define GPS_RX_PIN     16
#define GPS_TX_PIN     17

// Instantiate the algorithm processor
IntegratedAlgorithm systemAlgo(
    IMU_SDA_PIN, 
    IMU_SCL_PIN, 
    VIBE_PIN, 
    GPS_RX_PIN, 
    GPS_TX_PIN
);

void setup() {
    // Initialize serial monitor for output
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait for serial console on USB boards

    Serial.println(F("Starting Integrated Algorithm System..."));

    // Initialize all hardware peripherals & sensors
    systemAlgo.begin();

    Serial.println(F("System initialized. Waiting for High Precision GPS fix..."));
}

void loop() {
    // 1. Core update step - continuously processes sensor streams
    systemAlgo.update();

    // 2. Print system log and variable outputs at 5 Hz
    systemAlgo.printData();

    // 3. Example: Direct access to public variables for custom logic
    if (systemAlgo.vibrationDetected) {
        // High vibration warning handling
    }

    // Access displacement and UTC time variables directly anywhere in your code
    float currentX = systemAlgo.displacementX_mm;
    float currentY = systemAlgo.displacementY_mm;
    float currentZ = systemAlgo.displacementZ_mm;

    // Direct access to UTC time
    String timeStr = systemAlgo.utcTime; // Returns "HH:MM:SS" or "INVALID"
    if (systemAlgo.isTimeValid) {
        uint8_t h = systemAlgo.utcHour;
        uint8_t m = systemAlgo.utcMinute;
        uint8_t s = systemAlgo.utcSecond;
    }

    // Maintain stable execution pacing
    delay(2); 
}
*/

#ifndef INTEGRATED_ALGORITHM_H
#define INTEGRATED_ALGORITHM_H

#include <Arduino.h>
#include <math.h>

// Include dependent headers
#include "IMU_Kalman.h"
#include "VibrationSensor.h"
#include "GPS_Telemetry.h"

class IntegratedAlgorithm {
private:
    IMU_Kalman _imu;
    VibrationSensor _vibeSensor;
    GPSTelemetry _gps;

    // Time tracking for IMU integration
    uint32_t _lastTimeUs = 0;

    // Reference/Origin GPS position for local displacement baseline
    double _originLat = 0.0;
    double _originLon = 0.0;
    double _originAlt = 0.0;
    bool _hasOrigin = false;

    // Complementary filter fusion factor for X/Y displacement (IMU heavy, GPS light correction)
    const float _alphaX = 0.98f; // 98% IMU integration, 2% GPS correction
    const float _alphaY = 0.98f;

    // Earth radius in meters for Equirectangular approximation
    const double EARTH_RADIUS_M = 6371000.0;

    // Internal helper: Convert Lat/Lon offset from origin to millimeters
    void convertGpsToLocalMm(double lat, double lon, float &gpsMmX, float &gpsMmY) {
        if (!_hasOrigin) {
            gpsMmX = 0.0f;
            gpsMmY = 0.0f;
            return;
        }

        double latRad = lat * (M_PI / 180.0);
        double originLatRad = _originLat * (M_PI / 180.0);
        double dLat = (lat - _originLat) * (M_PI / 180.0);
        double dLon = (lon - _originLon) * (M_PI / 180.0);

        // North-South displacement (Y-axis) in mm
        gpsMmY = (float)(dLat * EARTH_RADIUS_M * 1000.0);

        // East-West displacement (X-axis) in mm
        gpsMmX = (float)(dLon * EARTH_RADIUS_M * cos((latRad + originLatRad) / 2.0) * 1000.0);
    }

public:
    // ----------------------------------------------------
    // Public Callable Output Variables
    // ----------------------------------------------------
    
    // 1. Displacement in X, Y (mm) - IMU accelerated, GPS corrected
    float displacementX_mm = 0.0f;
    float displacementY_mm = 0.0f;
    float imuVelocityX_mm_s = 0.0f; // Internal velocity state in mm/s
    float imuVelocityY_mm_s = 0.0f;

    // 2. Displacement in Z (mm) - GPS only
    float displacementZ_mm = 0.0f;

    // 3. Angular Rates (deg/s) & Orientations (deg)
    float RateRollDegS = 0.0f;
    float RatePitchDegS = 0.0f;
    float RateYawDegS = 0.0f;
    float AngleRoll = 0.0f;
    float AnglePitch = 0.0f;
    float AngleYaw = 0.0f;

    // 4. Vibration State
    bool vibrationDetected = false;

    // 5. Accelerations (mm/s^2)
    float AccX_mmss2 = 0.0f;
    float AccY_mmss2 = 0.0f;
    float AccZ_mmss2 = 0.0f;
    float resultantAccel = 0.0f;

    // 6. GPS Metrics
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    int gpsHighPrecision = 0;

    // 7. UTC Time Variables
    uint8_t utcHour = 0;
    uint8_t utcMinute = 0;
    uint8_t utcSecond = 0;
    bool isTimeValid = false;
    String utcTime = "INVALID";

    // Constructor passing default/custom pins down to underlying modules
    IntegratedAlgorithm(uint8_t imuSda = 21, uint8_t imuScl = 22,
                        uint8_t vibePin = 25,
                        uint8_t gpsRx = 16, uint8_t gpsTx = 17)
        : _imu(imuSda, imuScl), _vibeSensor(vibePin), _gps(gpsRx, gpsTx) {}

    // Initialize all sensors
    void begin() {
        _imu.begin();
        _vibeSensor.begin();
        _gps.begin();
        _lastTimeUs = micros();
    }

    // Main update step (call repeatedly in loop)
    void update() {
        // Poll sensors
        _vibeSensor.update();
        _gps.update();
        _imu.processData();

        // --- Calculate Delta Time (dt) ---
        uint32_t currentTimeUs = micros();
        float dt = (currentTimeUs - _lastTimeUs) / 1000000.0f; // Convert microseconds to seconds
        _lastTimeUs = currentTimeUs;

        // Prevent large integration spikes on first run or frame drops
        if (dt <= 0.0f || dt > 0.1f) dt = 0.004f;

        // --- 6. GPS Raw Values & UTC Time ---
        lat = _gps.data.latitude;
        lon = _gps.data.longitude;
        alt = _gps.data.altitude;

        utcHour = _gps.data.hour;
        utcMinute = _gps.data.minute;
        utcSecond = _gps.data.second;
        isTimeValid = _gps.data.isTimeValid;
        utcTime = _gps.data.getFormattedTime();
        gpsHighPrecision = _gps.data.isHighPrecision;

        // Set initial baseline origin when high precision fix becomes available
        if (!_hasOrigin) {
            _originLat = lat;
            _originLon = lon;
            _originAlt = alt;
            _hasOrigin = true;
        }

        // --- 4. Vibration State ---
        vibrationDetected = _vibeSensor.isVibrationDetected();

        // --- 3. Angular Rates, Orientations & Resultant ---
        RateRollDegS = _imu.RateRollDegS;
        RatePitchDegS = _imu.RatePitchDegS;
        RateYawDegS = _imu.RateYawDegS;

        AngleRoll = _imu.AngleRoll;
        AnglePitch = _imu.AnglePitch;
        AngleYaw = _imu.AngleYaw;

        // --- 5. Accelerations & Resultant (mm/s^2) ---
        // _imu.ax, ay, az are linear accels in m/s^2; convert to mm/s^2 (* 1000)
        AccX_mmss2 = _imu.ax * 1000.0f;
        AccY_mmss2 = _imu.ay * 1000.0f;
        AccZ_mmss2 = _imu.az * 1000.0f;

        resultantAccel = sqrtf(
            (AccX_mmss2 * AccX_mmss2) +
            (AccY_mmss2 * AccY_mmss2) +
            (AccZ_mmss2 * AccZ_mmss2)
        );

        // --- 2. Z Displacement (GPS only, in mm) ---
        displacementZ_mm = (float)((alt - _originAlt) * 1000.0);

        // --- 1. X, Y Displacement Fusion (IMU major + GPS minor, in mm) ---
        
        // Step 1: Double integration of IMU acceleration for IMU displacement prediction
        imuVelocityX_mm_s += AccX_mmss2 * dt;
        imuVelocityY_mm_s += AccY_mmss2 * dt;

        float imuDisplacementX = displacementX_mm + (imuVelocityX_mm_s * dt);
        float imuDisplacementY = displacementY_mm + (imuVelocityY_mm_s * dt);

        // Step 2: Compute relative GPS displacement from origin in mm
        float gpsDisplacementX = 0.0f;
        float gpsDisplacementY = 0.0f;
        convertGpsToLocalMm(lat, lon, gpsDisplacementX, gpsDisplacementY);

        // Step 3: Sensor Fusion (Complementary Filter)
        displacementX_mm = _alphaX * imuDisplacementX + (1.0f - _alphaX) * gpsDisplacementX;
        displacementY_mm = _alphaY * imuDisplacementY + (1.0f - _alphaY) * gpsDisplacementY;
    }

    // Print routine to display all calculated variables
    void printData() const {
        static uint32_t lastPrintTime = 0;
        if (millis() - lastPrintTime >= 200) { // 5 Hz print rate
            lastPrintTime = millis();

            Serial.println(F("================ INTEGRATED ALGORITHM OUTPUT ================"));
            Serial.print(F("High Precision Active : "));
            Serial.println(_gps.data.isHighPrecision ? F("YES") : F("NO (Algorithm Paused)"));

            Serial.print(F("UTC Time              : ")); Serial.println(utcTime);

            Serial.print(F("GPS Coordinates       : Lat: ")); Serial.print(lat, 6);
            Serial.print(F(" | Lon: ")); Serial.print(lon, 6);
            Serial.print(F(" | Alt: ")); Serial.print(alt, 2); Serial.println(F(" m"));

            Serial.print(F("Displacement (X, Y, Z): X: ")); Serial.print(displacementX_mm, 2);
            Serial.print(F(" mm | Y: ")); Serial.print(displacementY_mm, 2);
            Serial.print(F(" mm | Z: ")); Serial.print(displacementZ_mm, 2); Serial.println(F(" mm"));

            Serial.print(F("Angles (R/P/Y)        : R: ")); Serial.print(AngleRoll, 2);
            Serial.print(F(" | P: ")); Serial.print(AnglePitch, 2);
            Serial.print(F(" | Y: ")); Serial.print(AngleYaw, 2); Serial.println(F(" deg"));

            Serial.print(F("Gyro Rates (R/P/Y)    : R: ")); Serial.print(RateRollDegS, 2);
            Serial.print(F(" | P: ")); Serial.print(RatePitchDegS, 2);
            Serial.print(F(" | Y: ")); Serial.print(RateYawDegS, 2); Serial.println(F(" deg/s"));

            Serial.print(F("Accelerations (X/Y/Z) : X: ")); Serial.print(AccX_mmss2, 2);
            Serial.print(F(" | Y: ")); Serial.print(AccY_mmss2, 2);
            Serial.print(F(" | Z: ")); Serial.print(AccZ_mmss2, 2);
            Serial.print(F(" mm/s^2 | Resultant: ")); Serial.print(resultantAccel, 2); Serial.println(F(" mm/s^2"));

            Serial.print(F("Vibration Status      : "));
            Serial.println(vibrationDetected ? F("VIBRATION DETECTED") : F("NORMAL"));

            Serial.print(F(" | GPS High Precision: ")); Serial.println(_gps.data.isHighPrecision);
            Serial.println(F("============================================================\n"));
        }
    }
};

#endif // INTEGRATED_ALGORITHM_H