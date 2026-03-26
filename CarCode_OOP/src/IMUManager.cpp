#include "IMUManager.h"

IMUManager::IMUManager()
    : roll(0), pitch(0), yaw(0) {
    // calib initialized as zero in declaration
}

void IMUManager::begin() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz

    Serial.println("Initializing IMU...");
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
    }

    IMU.setIMUGeometry(1); // 1 rotate 90 under roof mount, diode left
    delay(1000);
    Serial.println("Keep IMU level.");
    delay(1000);
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);

    delay(1000);
    IMU.init(calib, IMU_ADDRESS);
}

void IMUManager::update() {
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ,
                     accelData.accelX, accelData.accelY, accelData.accelZ);

    roll = filter.getRoll() * (PI / 180.0f);
    pitch = filter.getPitch() * (PI / 180.0f);
    yaw = filter.getYaw() * (PI / 180.0f);
}

float IMUManager::getRoll() const {
    return roll;
}

float IMUManager::getPitch() const {
    return pitch;
}

float IMUManager::getYaw() const {
    return yaw;
}

float IMUManager::getAccelX() const {
    return accelData.accelX;
}

float IMUManager::getAccelY() const {
    return accelData.accelY;
}
