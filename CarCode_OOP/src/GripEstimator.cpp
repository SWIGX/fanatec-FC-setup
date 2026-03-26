#include "GripEstimator.h"
#include <math.h>

GripEstimator::GripEstimator()
    : _prevRPM(0.0f), _timeSinceRPMChange(0.0f), _hasPrev(false),
      _accelSumX(0.0f), _accelSumY(0.0f), _accelCount(0),
      _smoothedGrip(100.0f) {
}

void GripEstimator::resetAccum() {
    _accelSumX = 0.0f;
    _accelSumY = 0.0f;
    _accelCount = 0;
    _timeSinceRPMChange = 0.0f;
}

void GripEstimator::update(float rpm, float accelX, float accelY, float dt) {
    if (dt <= 0.0f) return;

    // Always accumulate body acceleration samples between Hall pulses
    _accelSumX += accelX;
    _accelSumY += accelY;
    _accelCount++;
    _timeSinceRPMChange += dt;

    if (!_hasPrev) {
        _prevRPM = rpm;
        _hasPrev = true;
        resetAccum();
        return;
    }

    // If RPM hasn't changed, decay grip toward 100 (no slip detected while coasting)
    if (rpm == _prevRPM) {
        if (_timeSinceRPMChange > COAST_TIMEOUT) {
            _smoothedGrip += SMOOTHING_ALPHA * (100.0f - _smoothedGrip);
        }
        return;
    }

    if (_timeSinceRPMChange <= 0.0f || _accelCount == 0) {
        _prevRPM = rpm;
        resetAccum();
        return;
    }

    // Average body acceleration over the window between Hall pulses
    float n = static_cast<float>(_accelCount);
    float avgAccelX = _accelSumX / n;
    float avgAccelY = _accelSumY / n;
    float bodyAccel = sqrtf(avgAccelX * avgAccelX + avgAccelY * avgAccelY);

    // Wheel acceleration between Hall pulses
    float wheelAccelRPMperS = (rpm - _prevRPM) / _timeSinceRPMChange;
    float wheelAccelG = wheelAccelRPMperS / WHEEL_ACCEL_SCALE;

    // Discrepancy: wheel accelerating more than the body = slip
    float discrepancy = 0.0f;
    if (wheelAccelG > bodyAccel) {
        discrepancy = wheelAccelG - bodyAccel;
    }

    // Map to grip 0-100
    float rawGrip = 100.0f * (1.0f - discrepancy / DISCREPANCY_MAX);
    if (rawGrip < 0.0f) rawGrip = 0.0f;
    if (rawGrip > 100.0f) rawGrip = 100.0f;

    // Smooth output
    _smoothedGrip += SMOOTHING_ALPHA * (rawGrip - _smoothedGrip);

    _prevRPM = rpm;
    resetAccum();
}

uint8_t GripEstimator::getGrip() const {
    return static_cast<uint8_t>(_smoothedGrip + 0.5f);
}
