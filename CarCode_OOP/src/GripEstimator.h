#pragma once

#include <Arduino.h>

class GripEstimator {
public:
    GripEstimator();

    void update(float rpm, float accelX, float accelY, float dt);

    uint8_t getGrip() const;

private:
    // RPM tracking
    float _prevRPM;
    float _timeSinceRPMChange;
    bool _hasPrev;

    // Body accel averaging between Hall pulses
    float _accelSumX;
    float _accelSumY;
    uint16_t _accelCount;

    // Output
    float _smoothedGrip;

    void resetAccum();

    static constexpr float WHEEL_ACCEL_SCALE = 1703.0f;   // RPM/s per g — derived from wheel radius 0.055m
    static constexpr float DISCREPANCY_MAX = 0.3f;        // g — max discrepancy before grip = 0
    static constexpr float SMOOTHING_ALPHA = 0.3f;        // low-pass filter (0..1, higher = faster response)
    static constexpr float COAST_TIMEOUT  = 0.5f;        // seconds with no RPM change before decaying grip to 100
};
