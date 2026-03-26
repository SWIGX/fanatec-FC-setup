#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <MadgwickAHRS.h>

// I2C address of the MPU6050 sensor
static const uint8_t IMU_ADDRESS = 0x68;

class IMUManager {
public:
    IMUManager();
    void begin();
    void update();

    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getAccelX() const;
    float getAccelY() const;

private:
    MPU6050 IMU;
    calData calib;
    AccelData accelData;
    GyroData gyroData;
    Madgwick filter;

    float roll;
    float pitch;
    float yaw;
};
