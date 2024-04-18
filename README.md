
# Link to the car
https://rc-shop-messmann.de/AMXRock-RCX8P-Scale-Crawler-Pick-Up-18-RTR-orange

# Link to the SercoEsc
https://components101.com/sensors/mpu6050-module

# How to flash a new image to the FlightController.

In this description you find a guide how to get your setup ready to build a new image to it.

## 1. Downloads

First of all we need to download the arduino IDE, this can be downloaded from here:

https://www.arduino.cc/en/software

When you have done the downloading of the IDE, you need. Now you need to download an extra extension for the Arduino IDE, its found in the link below.

https://www.pjrc.com/teensy/td_download.html

The main part of downloading the extension is to insert the link to the json file under `File > Preferences` - The Json link is here `https://www.pjrc.com/teensy/package_teensy_index.json`. When you click **OK** the IDE will automatically download the package.

![](https://www.pjrc.com/teensy/arduino20prefs.png)

## 2. Setup

Now you have the make your IDE ready to flash a image to your Teensy FlightController. In the course we have been using a Teensy 3.2, so you have to select this as board. If you go to the `Board Manager` you can type in **teensy**, and download the Teensy boards. (See screenshot below).

![](https://www.pjrc.com/teensy/arduino20boardsmanager.png)

When you downloaded this package you should be able to select `Teensy 3.2 / 3.1`.

The `SWIGX_RCFC_MPU6050_Teensy32.ino` file have som imports from a few different libaries. Some of the imports we need to manually download ourself with the `Libary Manager` inside the Arduino IDE. The following package you need to download before you will be able to compile the code.

| Package Name | Version |
| ------------ | ------- |
| MAVLink      | 2.0.4   |
| PWMServo     | 2.1.0   |
| MadgwickAHRS | 1.2.0   |
| FastIMU      | 1.2.6   |

After downloading all the packages you should now be click on the `checkmark`(Verify) in left corner. You will receive alot of errors when compiling, but don't worry. If a little windows popus up with

![](https://www.prnt.sc/sRrC2IEChRTJ)

## 3. Info about Teensy 3.2

Here is a small description about the Teensy 3.2, and some important pins we are using to connect to `AIR Unit`(RPI4) and the `Accelerometer`(https://components101.com/sensors/mpu6050-module).

![](https://www.pjrc.com/store/teensy32_card7a_rev3.png)

## Overview of channels from Fanatac wheel

Channel1 - rat
-- 1500 midten
-- til venstre ned -- 1000 min -- fra midt 1.5 rotation til min
-- til højre op -- 2000 max -- fra midt 1.5 rotation til max

Channel16 - button1
-- 1000 - 2000
-- Normally 2000, when pressed 1000

Channel18 - button3
-- 1000 - 2000
-- Normally 2000, when pressed 1000

Channel17 - button4
-- 1000 - 2000
-- Normally 2000, when pressed 1000

Channel15 - pil ned
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel12 -- + knap
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel9 -- minus knap
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel10 - eye
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel11 - ! knap
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel14 - downshifter
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel13 - upshifter
-- 1000 - 2000
-- Normally 2000, when pressed 1000

channel3 - speeder
normal 2000
fuld tryk 1000

channel6 - bremse
normal 2000
fuld tryk 1000

channel2 - kobling
normal 2000
fuld tryk 1000
