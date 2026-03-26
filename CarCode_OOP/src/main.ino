#include <Arduino.h>

#include "IMUManager.h"
#include "RCInput.h"
#include "VehicleControl.h"
#include "Failsafe.h"
#include "MAVLinkManager.h"
#include "Hall.h"


IMUManager imu;
RCInput rcInput;
VehicleControl vehicle;
Failsafe failsafe(vehicle);
MAVLinkManager mav(&rcInput, &failsafe);
Hall hall;

void setup() {
    // initialize peripherals and subsystems
    imu.begin();
    vehicle.begin();
    hall.begin(2, 1, true, 5); // example: hall sensor on pin 2
    mav.begin(Serial1); // use Serial1 as MAVLink port
    
}

void loop() {
    imu.update();
    hall.update();

    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();

    // read incoming MAVLink messages and update RC inputs
    mav.process();

    // update vehicle actuators based on latest RC input
    vehicle.processInput(rcInput);

    // check failsafe timeout and possibly zero throttle
    failsafe.update(millis());

    // send periodic MAVLink messages
    mav.update(roll, pitch, rcInput.ch3, yaw, hall.getSlopeInt16());
}
