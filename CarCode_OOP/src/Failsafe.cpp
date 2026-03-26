#include "Failsafe.h"
#include "VehicleControl.h"

Failsafe::Failsafe(VehicleControl& vc)
    : lastOverrideMillis(0), vehicle(vc) {
}

void Failsafe::resetTimer() {
    lastOverrideMillis = millis();
}

void Failsafe::update(unsigned long currentMillis) {
    if ((currentMillis - lastOverrideMillis) > timeout) {
        vehicle.setNeutralThrottle();
    }
}
