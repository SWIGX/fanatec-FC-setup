#pragma once

#include <Arduino.h>
#include <PWMServo.h>
#include "RCInput.h"

class VehicleControl {
public:
    VehicleControl();
    void begin();
    void processInput(const RCInput& rc);
    void setNeutralThrottle();

private:
    PWMServo steeringServo;
    PWMServo escServo;
    PWMServo camServo;
    PWMServo camServo2;
    PWMServo frontDif;
    PWMServo middleDif;
    PWMServo backDif;

    int gear;
    bool plus_button_pressed;
    bool minus_button_pressed;
    bool is_back_dif_open;
    bool is_middle_dif_open;
    bool is_front_dif_open;
    int steering_trim;
};
