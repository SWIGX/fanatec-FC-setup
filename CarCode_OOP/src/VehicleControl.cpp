#include "VehicleControl.h"

VehicleControl::VehicleControl()
    : gear(0), plus_button_pressed(false), minus_button_pressed(false),
      is_back_dif_open(false), is_middle_dif_open(false), is_front_dif_open(false),
      steering_trim(0) {
}

void VehicleControl::begin() {
    steeringServo.attach(5, 544, 2400);
    escServo.attach(6);

    // attach cam servos
    camServo.attach(9);
    camServo2.attach(10, 1000, 2000);

    // we leave differential servos unattached unless they are used
    // frontDif.attach(...);
    // middleDif.attach(...);
    // backDif.attach(...);

    camServo.write(90);
    camServo2.write(90);
    escServo.write(90);
    steeringServo.write(135 + steering_trim);
}

void VehicleControl::processInput(const RCInput& rc) {
    // steering
    int val = map(rc.ch1, 1000, 2000, 180, 45);
    steeringServo.write(val + steering_trim);

    // gear shifting buttons
    if (rc.ch13 < 1500) {
        if (gear < 1) {
            escServo.write(90);
            gear++;
        }
    }

    if (rc.ch14 < 1500) {
        if (gear > -1) {
            escServo.write(90);
            gear--;
        }
    }

    if (gear == 1) {
        int speed = map(rc.ch3, 1000, 2000, 130, 90);
        escServo.write(speed);
    } else if (gear == -1) {
        int backSpeed = map(rc.ch3, 1000, 2000, 70, 90);
        escServo.write(backSpeed);
    }

    // steering trim buttons
    if (rc.ch9 > 1600 && minus_button_pressed) {
        minus_button_pressed = false;
    }
    if (rc.ch9 < 1600 && !minus_button_pressed) {
        minus_button_pressed = true;
        steering_trim -= 1;
    }

    if (rc.ch12 > 1600 && plus_button_pressed) {
        plus_button_pressed = false;
    }
    if (rc.ch12 < 1600 && !plus_button_pressed) {
        plus_button_pressed = true;
        steering_trim += 1;
    }

    // differentials
    if (rc.ch16 < 1500) {
        if (is_middle_dif_open) {
            // TODO: set PWM value corresponding to closed
            is_middle_dif_open = false;
        } else {
            // TODO: set PWM value corresponding to open
            is_middle_dif_open = true;
        }
    }
    if (rc.ch17 < 1500) {
        if (is_back_dif_open) {
            // TODO: set PWM value corresponding to closed
            is_back_dif_open = false;
        } else {
            // TODO: set PWM value corresponding to open
            is_back_dif_open = true;
        }
    }
    if (rc.ch18 < 1500) {
        if (is_front_dif_open) {
            // TODO: set PWM value corresponding to closed
            is_front_dif_open = false;
        } else {
            // TODO: set PWM value corresponding to open
            is_front_dif_open = true;
        }
    }
}

void VehicleControl::setNeutralThrottle() {
    escServo.write(90);
}
