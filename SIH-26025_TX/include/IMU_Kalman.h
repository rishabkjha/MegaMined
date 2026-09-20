/*
#include "IMU_Kalman.h"

// Instantiate the IMU object (defaults to SDA 21, SCL 22, I2C Address 0x68)
IMU_Kalman imu(21, 22, 0x68);

void setup() {
    Serial.begin(115200);
    imu.begin(); // Initializes I2C, registers, and calibrates
}

void loop() {
    imu.printData();          // Updates and prints at 10Hz
    imu.maintainLoopRate();   // Enforces strict 250Hz loop timing
}
*/

#ifndef IMU_KALMAN_H
#define IMU_KALMAN_H

#include <Arduino.h>
#include <Wire.h>

class IMU_Kalman {
private:
    uint8_t _sdaPin;
    uint8_t _sclPin;
    uint8_t _i2cAddress;
    
    uint32_t _loopTimer;
    const float _dt = 0.004f; // 250 Hz Loop (4ms)

    // Raw and calibrated rates
    float _rateRoll, _ratePitch, _rateYaw;
    float _rateCalRoll = 0, _rateCalPitch = 0, _rateCalYaw = 0;

    // Calibration biases for Accel
    float _accCalX = 0, _accCalY = 0, _accCalZ = 0;

    // Kalman uncertainties
    float _uncertaintyRoll = 4.0f;
    float _uncertaintyPitch = 4.0f;

    // Internal 1D Kalman Filter update routine
    void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle) {
        state += _dt * gyroRate;
        uncertainty += _dt * _dt * 16.0f; // Process noise
        float gain = uncertainty / (uncertainty + 9.0f); // Measurement noise
        state += gain * (accelAngle - state);
        uncertainty *= (1.0f - gain);
    }

public:
    // Public output variables
    float ax, ay, az;                     // Earth-frame/calibrated accel in m/s^2
    float AccX, AccY, AccZ;               // Body-frame raw accel in g
    float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;// Estimated Euler angles in degrees
    float KalmanAngleRoll = 0, KalmanAnglePitch = 0;
    float RateRollDegS, RatePitchDegS, RateYawDegS; // Calibrated gyro rates in deg/s

    // Constructor
    IMU_Kalman(uint8_t sdaPin = 21, uint8_t sclPin = 22, uint8_t i2cAddress = 0x68) 
        : _sdaPin(sdaPin), _sclPin(sclPin), _i2cAddress(i2cAddress) {}

    // Initialize Wire interface and MPU Registers
    void begin() {
        Wire.begin(_sdaPin, _sclPin);
        Wire.setClock(400000); // 400kHz I2C Fast Mode

        // Power Management 1 -> Wake up
        Wire.beginTransmission(_i2cAddress);
        Wire.write(0x6B);
        Wire.write(0x00);
        Wire.endTransmission();

        // CONFIG -> Set DLPF to ~42Hz bandwidth
        Wire.beginTransmission(_i2cAddress);
        Wire.write(0x1A);
        Wire.write(0x03);
        Wire.endTransmission();

        // Accelerometer Configuration -> +/- 8g range
        Wire.beginTransmission(_i2cAddress);
        Wire.write(0x1C);
        Wire.write(0x10);
        Wire.endTransmission();

        // Gyroscope Configuration -> +/- 500 deg/s range
        Wire.beginTransmission(_i2cAddress);
        Wire.write(0x1B);
        Wire.write(0x08);
        Wire.endTransmission();

        calibrate();
        _loopTimer = micros();
    }

    // Read raw values from the MPU
    void readIMU() {
        Wire.beginTransmission(_i2cAddress);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(_i2cAddress, (uint8_t)14);

        int16_t axLSB = Wire.read() << 8 | Wire.read();
        int16_t ayLSB = Wire.read() << 8 | Wire.read();
        int16_t azLSB = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature bytes
        int16_t gxLSB = Wire.read() << 8 | Wire.read();
        int16_t gyLSB = Wire.read() << 8 | Wire.read();
        int16_t gzLSB = Wire.read() << 8 | Wire.read();

        // Convert LSBs to physical units
        _rateRoll  = (float)gxLSB / 65.5f;
        _ratePitch = (float)gyLSB / 65.5f;
        _rateYaw   = (float)gzLSB / 65.5f;

        AccX = -0.005f + ((float)axLSB / 4096.0f);
        AccY =  0.010f + ((float)ayLSB / 4096.0f);
        AccZ = -0.035f + ((float)azLSB / 4096.0f);
    }

    // Collect initial samples to zero out bias offsets
    void calibrate() {
        _rateCalRoll = 0; _rateCalPitch = 0; _rateCalYaw = 0;
        _accCalX = 0; _accCalY = 0; _accCalZ = 0;

        for (int i = 0; i < 2000; i++) {
            readIMU();
            _rateCalRoll  += _rateRoll;
            _rateCalPitch += _ratePitch;
            _rateCalYaw   += _rateYaw;
            _accCalX      += AccX;
            _accCalY      += AccY;
            _accCalZ      += (AccZ - 1.0f); // Standard gravity reference
            delay(1);
        }

        _rateCalRoll  /= 2000.0f;
        _rateCalPitch /= 2000.0f;
        _rateCalYaw   /= 2000.0f;
        _accCalX      /= 2000.0f;
        _accCalY      /= 2000.0f;
        _accCalZ      /= 2000.0f;
    }

    // Process sensor data & run Kalman estimation step
    void processData() {
        readIMU();

        RateRollDegS  = _rateRoll  - _rateCalRoll;
        RatePitchDegS = _ratePitch - _rateCalPitch;
        RateYawDegS   = _rateYaw   - _rateCalYaw;
        float _RateYawDegS = RateYawDegS;

        // Threshold small noise jitter
        if (abs(RateRollDegS) < 1.0f)  RateRollDegS = 0;
        if (abs(RatePitchDegS) < 1.0f) RatePitchDegS = 0;
        if (abs(RateYawDegS) < 1.0f)   RateYawDegS = 0;

        // Accelerometer Pitch/Roll Angles
        float accAngleRoll  =  atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0f / M_PI);
        float accAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0f / M_PI);

        // Kalman filtering
        update1DKalman(KalmanAngleRoll, _uncertaintyRoll, RateRollDegS, accAngleRoll);
        update1DKalman(KalmanAnglePitch, _uncertaintyPitch, RatePitchDegS, accAnglePitch);

        if(abs(RateYawDegS)<2.0) _RateYawDegS = 0;

        AngleRoll  = KalmanAngleRoll;
        AnglePitch = KalmanAnglePitch;
        AngleYaw += _RateYawDegS*_dt; 

        // Earth-Frame Linear Accelerations (m/s^2)
        ax = (AccX - _accCalX) * 9.81f;
        ay = (AccY - _accCalY) * 9.81f;
        az = (AccZ - _accCalZ) * 9.81f;
    }

    // Print values at 10 Hz rate
    void printData() {
        processData();
        static uint32_t printTimer = 0;
        if (millis() - printTimer >= 100) {
            printTimer = millis();

            char buffer[128];
            snprintf(buffer, sizeof(buffer),
                "Angle R:%6.2f P:%6.2f Y:%6.2f | Rate R:%6.2f P:%6.2f Y:%6.2f | Acc X:%6.2f Y:%6.2f Z:%6.2f",
                AngleRoll, AnglePitch, AngleYaw,
                RateRollDegS, RatePitchDegS, RateYawDegS,
                AccX, AccY, AccZ
            );
            Serial.println(buffer);
        }
    }

    // Call at the end of loop() to maintain exact 250Hz timing
    void maintainLoopRate() {
        while (micros() - _loopTimer < 4000);
        _loopTimer = micros();
    }
};

#endif // IMU_KALMAN_H