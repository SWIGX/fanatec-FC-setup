#pragma once

#include <Arduino.h>

class GripEstimator {
public:
    GripEstimator();

    void update(float rpm, float accelX, float accelY, float dt);

    uint8_t getGrip() const;

private:
    float _prevRPM;
    bool _hasPrev;
    float _smoothedGrip;

    static constexpr float ACCEL_DISCREPANCY_MAX = 5.0f; // tuning: max discrepancy before grip = 0
    static constexpr float SMOOTHING_ALPHA = 0.1f;       // low-pass filter coefficient (0..1, lower = smoother)
};
