#pragma once

#include <Arduino.h>

class Hall {
public:
    Hall();

    void begin(uint8_t pin,
               uint8_t pulsesPerRevolution = 1,
               bool usePullup = true,
               uint8_t slopeWindowSize = 5);

    void update();

    float getRPM() const;
    float getLatestIntervalMs() const;
    float getIntervalSlope() const;
    int16_t getSlopeInt16() const;
    bool hasValidRPM() const;

private:
    static constexpr uint8_t MAX_FIFO_SIZE = 20;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000UL;
    static constexpr uint32_t MIN_PULSE_INTERVAL_US = 300UL;

    static Hall* instance;
    static void isrRouter();
    void handleInterrupt();

    void pushIntervalMs(float intervalMs);
    float computeSlope() const;
    void resetData();

    uint8_t _pin;
    uint8_t _pulsesPerRevolution;
    uint8_t _slopeWindowSize;

    volatile uint32_t _lastPulseUs;
    volatile uint32_t _intervalUs;
    volatile bool _newSample;

    float _intervalFifo[MAX_FIFO_SIZE];
    uint8_t _count;

    float _rpm;
    float _latestIntervalMs;
    float _slope;
    bool _validRPM;

    uint32_t _lastMeasurementMs;
};