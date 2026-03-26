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

    static constexpr float WHEEL_ACCEL_SCALE = 600.0f;   // RPM/s per g — tune on real car
    static constexpr float DISCREPANCY_MAX = 2.0f;        // g — max discrepancy before grip = 0
    static constexpr float SMOOTHING_ALPHA = 0.3f;        // low-pass filter (0..1, higher = faster response)
};
