#pragma once

#include <Arduino.h>
#include <MAVLink.h>

class RCInput;
class Failsafe;

class MAVLinkManager {
public:
    MAVLinkManager(RCInput* rc, Failsafe* fs);
    void begin(HardwareSerial& serialPort);
    void process();
    void update(float roll, float pitch, float throttle, float yaw, int16_t hallSlope);

private:
    HardwareSerial* port;
    RCInput* rc;
    Failsafe* failsafe;

    unsigned long prevHeartbeat;
    unsigned long prevAttitude;
    const unsigned long heartbeatInterval = 1000;
    const unsigned long attitudeInterval = 10;

    // system parameters
    int sysid;
    int compid;
    int type;
    uint8_t system_type;
    uint8_t autopilot_type;
    uint8_t system_mode;
    uint32_t custom_mode;
    uint8_t system_state;


    void sendHeartbeat();
    void sendAttitude(float roll, float pitch, float throttle, float yaw, int16_t hallSlope);
};
