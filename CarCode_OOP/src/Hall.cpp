// #include "Hall.h"

// Hall* Hall::instance = nullptr;

// Hall::Hall()
//     : _pin(255),
//       _pulsesPerRevolution(1),
//       _lastPulseUs(0),
//       _intervalUs(0),
//       _newSample(false),
//       _count(0),
//       _rpm(0.0f),
//       _latestIntervalUs(0.0f),
//       _slope(0.0f),
//       _validRPM(false),
//       _lastMeasurementMs(0) {
//     resetData();
// }

// void Hall::begin(uint8_t pin, uint8_t pulsesPerRevolution, bool usePullup) {
//     _pin = pin;
//     _pulsesPerRevolution = (pulsesPerRevolution == 0) ? 1 : pulsesPerRevolution;

//     if (usePullup) {
//         pinMode(_pin, INPUT_PULLUP);
//     } else {
//         pinMode(_pin, INPUT);
//     }

//     resetData();

//     instance = this;
//     attachInterrupt(digitalPinToInterrupt(_pin), Hall::isrRouter, RISING);
// }

// void Hall::isrRouter() {
//     if (instance != nullptr) {
//         instance->handleInterrupt();
//     }
// }

// void Hall::handleInterrupt() {
//     const uint32_t nowUs = micros();

//     if (_lastPulseUs != 0) {
//         const uint32_t dt = nowUs - _lastPulseUs;

//         if (dt < MIN_PULSE_INTERVAL_US) {
//             return;
//         }

//         _intervalUs = dt;
//         _newSample = true;
//     }

//     _lastPulseUs = nowUs;
// }

// void Hall::update() {
//     uint32_t intervalCopy = 0;
//     bool hasNewSample = false;

//     noInterrupts();
//     hasNewSample = _newSample;
//     if (hasNewSample) {
//         intervalCopy = _intervalUs;
//         _newSample = false;
//     }
//     interrupts();

//     const uint32_t nowMs = millis();

//     if (_lastMeasurementMs != 0 && (nowMs - _lastMeasurementMs) >= WATCHDOG_TIMEOUT_MS) {
//         resetData();
//         return;
//     }

//     if (!hasNewSample) {
//         return;
//     }

//     if (intervalCopy == 0) {
//         return;
//     }

//     _latestIntervalUs = static_cast<float>(intervalCopy);
//     _rpm = 60000000.0f / (static_cast<float>(_pulsesPerRevolution) * _latestIntervalUs);
//     _validRPM = true;
//     _lastMeasurementMs = nowMs;

//     pushInterval(_latestIntervalUs);

//     // Regressionslinjen opdateres hver gang der kommer en ny måling
//     _slope = computeSlope();
// }

// void Hall::pushInterval(float intervalUs) {
//     if (_count < FIFO_SIZE) {
//         _intervalFifo[_count] = intervalUs;
//         ++_count;
//         return;
//     }

//     for (uint8_t i = 0; i < FIFO_SIZE - 1; ++i) {
//         _intervalFifo[i] = _intervalFifo[i + 1];
//     }

//     _intervalFifo[FIFO_SIZE - 1] = intervalUs;
// }

// float Hall::computeSlope() const {
//     if (_count < 2) {
//         return 0.0f;
//     }

//     const float n = static_cast<float>(_count);
//     float sumX = 0.0f;
//     float sumY = 0.0f;
//     float sumXY = 0.0f;
//     float sumX2 = 0.0f;

//     for (uint8_t i = 0; i < _count; ++i) {
//         const float x = static_cast<float>(i + 1);   // 1..10
//         const float y = _intervalFifo[i];            // tidsinterval på y-akse

//         sumX += x;
//         sumY += y;
//         sumXY += x * y;
//         sumX2 += x * x;
//     }

//     const float denominator = (n * sumX2) - (sumX * sumX);
//     if (denominator == 0.0f) {
//         return 0.0f;
//     }

//     return ((n * sumXY) - (sumX * sumY)) / denominator;
// }

// void Hall::resetData() {
//     for (uint8_t i = 0; i < FIFO_SIZE; ++i) {
//         _intervalFifo[i] = 0.0f;
//     }

//     _count = 0;
//     _rpm = 0.0f;
//     _latestIntervalUs = 0.0f;
//     _slope = 0.0f;
//     _validRPM = false;
//     _lastMeasurementMs = 0;

//     noInterrupts();
//     _lastPulseUs = 0;
//     _intervalUs = 0;
//     _newSample = false;
//     interrupts();
// }

// float Hall::getRPM() const {
//     return _rpm;
// }

// float Hall::getLatestIntervalUs() const {
//     return _latestIntervalUs;
// }

// float Hall::getIntervalSlope() const {
//     return _slope;
// }

// int16_t Hall::getSlopeInt16() const {
//     float limited = _slope;

//     if (limited > 32767.0f) {
//         limited = 32767.0f;
//     } else if (limited < -32768.0f) {
//         limited = -32768.0f;
//     }

//     return static_cast<int16_t>(limited);
// }

// bool Hall::hasValidRPM() const {
//     return _validRPM;
// }
#include "Hall.h"

Hall* Hall::instance = nullptr;

Hall::Hall()
    : _pin(255),
      _pulsesPerRevolution(1),
      _slopeWindowSize(5),
      _lastPulseUs(0),
      _intervalUs(0),
      _newSample(false),
      _count(0),
      _rpm(0.0f),
      _latestIntervalMs(0.0f),
      _slope(0.0f),
      _validRPM(false),
      _lastMeasurementMs(0) {
    resetData();
}

void Hall::begin(uint8_t pin,
                 uint8_t pulsesPerRevolution,
                 bool usePullup,
                 uint8_t slopeWindowSize) {
    _pin = pin;
    _pulsesPerRevolution = (pulsesPerRevolution == 0) ? 1 : pulsesPerRevolution;

    if (slopeWindowSize < 2) {
        _slopeWindowSize = 2;
    } else if (slopeWindowSize > MAX_FIFO_SIZE) {
        _slopeWindowSize = MAX_FIFO_SIZE;
    } else {
        _slopeWindowSize = slopeWindowSize;
    }

    if (usePullup) {
        pinMode(_pin, INPUT_PULLUP);
    } else {
        pinMode(_pin, INPUT);
    }

    resetData();

    instance = this;

    // Open collector + pullup giver normalt aktiv lav puls.
    // Derfor er FALLING typisk det rigtige valg.
    attachInterrupt(digitalPinToInterrupt(_pin), Hall::isrRouter, FALLING);
}

void Hall::isrRouter() {
    if (instance != nullptr) {
        instance->handleInterrupt();
    }
}

void Hall::handleInterrupt() {
    const uint32_t nowUs = micros();

    if (_lastPulseUs != 0) {
        const uint32_t dt = nowUs - _lastPulseUs;

        if (dt < MIN_PULSE_INTERVAL_US) {
            return;
        }

        _intervalUs = dt;
        _newSample = true;
    }

    _lastPulseUs = nowUs;
}

void Hall::update() {
    uint32_t intervalCopyUs = 0;
    bool hasNewSample = false;

    noInterrupts();
    hasNewSample = _newSample;
    if (hasNewSample) {
        intervalCopyUs = _intervalUs;
        _newSample = false;
    }
    interrupts();

    const uint32_t nowMs = millis();

    if (_lastMeasurementMs != 0 && (nowMs - _lastMeasurementMs) >= WATCHDOG_TIMEOUT_MS) {
        resetData();
        return;
    }

    if (!hasNewSample || intervalCopyUs == 0) {
        return;
    }

    _latestIntervalMs = static_cast<float>(intervalCopyUs) / 1000.0f;
    _rpm = 20000.0f / (static_cast<float>(_pulsesPerRevolution) * static_cast<float>(intervalCopyUs));
    _validRPM = true;
    _lastMeasurementMs = nowMs;

    pushIntervalMs(_latestIntervalMs);

    // Matcher Python logikken, hvor du viser slope_modsat = -slope
    _slope = -computeSlope();
}

void Hall::pushIntervalMs(float intervalMs) {
    if (_count < MAX_FIFO_SIZE) {
        _intervalFifo[_count] = intervalMs;
        ++_count;
        return;
    }

    for (uint8_t i = 0; i < MAX_FIFO_SIZE - 1; ++i) {
        _intervalFifo[i] = _intervalFifo[i + 1];
    }

    _intervalFifo[MAX_FIFO_SIZE - 1] = intervalMs;
}

float Hall::computeSlope() const {
    const uint8_t nSamples = (_count < _slopeWindowSize) ? _count : _slopeWindowSize;

    if (nSamples < 2) {
        return 0.0f;
    }

    const uint8_t startIndex = _count - nSamples;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumXY = 0.0f;
    float sumX2 = 0.0f;

    for (uint8_t i = 0; i < nSamples; ++i) {
        const float x = static_cast<float>(i);
        const float y = _intervalFifo[startIndex + i];

        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    const float n = static_cast<float>(nSamples);
    const float denominator = (n * sumX2) - (sumX * sumX);

    if (denominator == 0.0f) {
        return 0.0f;
    }

    return ((n * sumXY) - (sumX * sumY)) / denominator;
}

void Hall::resetData() {
    for (uint8_t i = 0; i < MAX_FIFO_SIZE; ++i) {
        _intervalFifo[i] = 0.0f;
    }

    _count = 0;
    _rpm = 0.0f;
    _latestIntervalMs = 0.0f;
    _slope = 0.0f;
    _validRPM = false;
    _lastMeasurementMs = 0;

    noInterrupts();
    _lastPulseUs = 0;
    _intervalUs = 0;
    _newSample = false;
    interrupts();
}

float Hall::getRPM() const {
    return _rpm;
}

float Hall::getLatestIntervalMs() const {
    return _latestIntervalMs;
}

float Hall::getIntervalSlope() const {
    return _slope;
}

int16_t Hall::getSlopeInt16() const {
    float limited = _slope;

    if (limited > 32767.0f) {
        limited = 32767.0f;
    } else if (limited < -32768.0f) {
        limited = -32768.0f;
    }

    return static_cast<int16_t>(limited);
}

bool Hall::hasValidRPM() const {
    return _validRPM;
}