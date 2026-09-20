/*
#include "VibrationSensor.h"

VibrationSensor vibSensor(25);

void setup() {
    Serial.begin(115200);
    vibSensor.begin();
}

void loop() {
    vibSensor.update();
    
    // Call every loop cycle; it will only print every 500ms automatically
    vibSensor.printAcquiredData(); 
}
*/

#ifndef VIBRATION_SENSOR_H
#define VIBRATION_SENSOR_H

#include <Arduino.h>

struct SystemState {
    bool vibrationDetected;
};

class VibrationSensor {
private:
    uint8_t _pin;
    unsigned long _debounceDelay;
    unsigned long _lastDebounceTime;
    int _lastPinState;
    SystemState _currentState;

    // Internal print timer tracking variable
    mutable unsigned long _lastPrintTime = 0; 

public:
    VibrationSensor(uint8_t pin = 25, unsigned long debounceDelay = 50)
        : _pin(pin), _debounceDelay(debounceDelay), _lastDebounceTime(0), _lastPinState(LOW) {
        _currentState.vibrationDetected = false;
    }

    void begin() {
        pinMode(_pin, INPUT);
        _lastPinState = digitalRead(_pin);
    }

    void update() {
        int reading = digitalRead(_pin);

        if (reading != _lastPinState) {
            _lastDebounceTime = millis();
        }

        if ((millis() - _lastDebounceTime) > _debounceDelay) {
            _currentState.vibrationDetected = (reading == HIGH);
        }

        _lastPinState = reading;
    }

    SystemState getSystemState() const {
        return _currentState;
    }

    bool isVibrationDetected() const {
        return _currentState.vibrationDetected;
    }

    // Handles rate-limiting internally automatically
    void printAcquiredData(unsigned long intervalMs = 500) const {
        if (millis() - _lastPrintTime >= intervalMs) {
            _lastPrintTime = millis();

            Serial.println(F("--- System Data Log ---"));
            Serial.print(F("Vibration State: "));
            Serial.println(_currentState.vibrationDetected ? F("DETECTED") : F("CLEAR"));
            Serial.println(F("-----------------------"));
        }
    }
};

#endif // VIBRATION_SENSOR_H