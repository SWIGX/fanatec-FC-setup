#pragma once

#include <Arduino.h>

class VehicleControl;

class Failsafe {
public:
    explicit Failsafe(VehicleControl& vc);
    void resetTimer();
    void update(unsigned long currentMillis);

private:
    unsigned long lastOverrideMillis;
    VehicleControl& vehicle;
    const unsigned long timeout = 1000;
};
