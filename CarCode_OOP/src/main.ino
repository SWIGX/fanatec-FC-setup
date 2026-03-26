#include <Arduino.h>

#include "IMUManager.h"
#include "RCInput.h"
#include "VehicleControl.h"
#include "Failsafe.h"
#include "MAVLinkManager.h"
#include "Hall.h"
#include "GripEstimator.h"


IMUManager imu;
RCInput rcInput;
VehicleControl vehicle;
Failsafe failsafe(vehicle);
MAVLinkManager mav(&rcInput, &failsafe);
Hall hall;
GripEstimator gripEstimator;

void setup() {
    // initialize peripherals and subsystems
    imu.begin();
    vehicle.begin();
    hall.begin(2, 3, true, 5); // hall sensor on pin 2, 3 magnets per revolution
    mav.begin(Serial1); // use Serial1 as MAVLink port
    
}

static unsigned long prevLoopTime = 0;

void loop() {
    unsigned long now = millis();
    float dt = (prevLoopTime == 0) ? 0.0f : (now - prevLoopTime) / 1000.0f;
    prevLoopTime = now;

    imu.update();
    hall.update();

    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();

    // compute grip estimate from wheel RPM vs body acceleration
    gripEstimator.update(hall.getRPM(), imu.getAccelX(), imu.getAccelY(), dt);

    // read incoming MAVLink messages and update RC inputs
    mav.process();

    // update vehicle actuators based on latest RC input
    vehicle.processInput(rcInput);

    // check failsafe timeout and possibly zero throttle
    failsafe.update(millis());

    // send periodic MAVLink messages
    mav.update(roll, pitch, rcInput.ch3, yaw, hall.getSlopeInt16(), hall.getRPM(), gripEstimator.getGrip());
}
