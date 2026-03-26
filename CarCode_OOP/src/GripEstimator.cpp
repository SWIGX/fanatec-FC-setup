#include "GripEstimator.h"
#include <math.h>

GripEstimator::GripEstimator()
    : _prevRPM(0.0f), _hasPrev(false), _smoothedGrip(100.0f) {
}

void GripEstimator::update(float rpm, float accelX, float accelY, float dt) {
    if (!_hasPrev || dt <= 0.0f) {
        _prevRPM = rpm;
        _hasPrev = true;
        return;
    }

    // Wheel angular acceleration (RPM/s)
    float wheelAccel = (rpm - _prevRPM) / dt;
    _prevRPM = rpm;

    // Body acceleration magnitude in horizontal plane (g units from IMU)
    float bodyAccel = sqrtf(accelX * accelX + accelY * accelY);

    // Convert wheel angular accel to comparable scale:
    // RPM/s → rev/s² = wheelAccel / 60
    // We don't need exact m/s² — just a proportional value to compare against bodyAccel.
    // bodyAccel is in g (~9.81 m/s²), so we scale wheelAccel empirically.
    // With a typical RC car wheel, 1g of real acceleration ≈ ~600 RPM/s change.
    // So wheelAccel / 600 gives us a g-equivalent estimate.
    float wheelAccelG = fabsf(wheelAccel) / 600.0f;

    // Discrepancy: how much more the wheel accelerates than the body
    // If wheel accelerates much more than body → slip → low grip
    float discrepancy = 0.0f;
    if (wheelAccelG > bodyAccel) {
        discrepancy = wheelAccelG - bodyAccel;
    }

    // Map discrepancy to raw grip (0..100)
    float rawGrip = 100.0f * (1.0f - discrepancy / ACCEL_DISCREPANCY_MAX);
    if (rawGrip < 0.0f) rawGrip = 0.0f;
    if (rawGrip > 100.0f) rawGrip = 100.0f;

    // Low-pass filter to smooth output
    _smoothedGrip = _smoothedGrip + SMOOTHING_ALPHA * (rawGrip - _smoothedGrip);
}

uint8_t GripEstimator::getGrip() const {
    return static_cast<uint8_t>(_smoothedGrip + 0.5f);
}
