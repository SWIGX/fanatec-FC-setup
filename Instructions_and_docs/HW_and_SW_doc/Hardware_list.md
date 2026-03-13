# Hardware Components

This document lists the hardware used in the RC car flight controller setup.

## Main Platform

| Component | Description | Link |
|---|---|---|
| AMEWI AMXRock RCX8B 1:8 Crawler | Base RC vehicle used for the project. Includes motor, ESC, drivetrain and chassis. | https://www.conradelektronik.dk/da/p/amewi-amxrock-rcx8b-scale-crawler-pick-up-1-8-rtr-blau-1-8-brushed-rc-modelbil-elektronik-crawler-rtr-2-4-ghz-2300048.html |
| Teensy 4.1 | Microcontroller used as the flight controller. Handles PWM control and sensor processing. | https://www.pjrc.com/store/teensy41.html |
| MPU6050 | 6-axis accelerometer and gyroscope used for motion sensing. | https://components101.com/sensors/mpu6050-module |

## Compute / Communication

| Component | Description |
|---|---|
| Raspberry Pi 4 (Air Unit) | Runs OpenHD and communicates with the flight controller. |
| Camera module | Used for video transmission through the Air Unit. |

## Control Hardware

| Component | Description |
|---|---|
| Fanatec Steering Wheel | Used as control interface for steering and vehicle control. |
| Fanatec Pedals | Used for throttle, brake and clutch input. |

## Actuators

| Component | Description |
|---|---|
| ESC (Electronic Speed Controller) | Controls motor speed using PWM signal from the flight controller. |
| Steering Servo | Controls steering angle of the RC vehicle. |
| Differential Controller | Controls locking of front, rear and center differential. |

## Power

| Component | Description |
|---|---|
| RC Battery Pack | Powers the RC car electronics. |
| External Power Supply (optional) | Used to power the air unit during debugging. |

## Measurement / Debug Equipment

| Component | Description |
|---|---|
| Analog Discovery 2 | Used for analyzing PWM signals during debugging. |
