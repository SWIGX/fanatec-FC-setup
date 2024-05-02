
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

Below you will see the how the Fanatac steering wheel/buttons is connected to the OpenHD channels which connetcs to the Teensy FlightController channels in the code.

| OpenHD - Channels | Fanatec reference  | PWM Range | Initial value                | Description                  |
| ------- | -----------------  |  -------  | ---------------------------- | ---------------------------- | 
| 1       | Steering wheel     | 1000-2000 | 1500 (Middle of the steering)| If you turn the wheel 1.5 rotation to left or right you will reach max/min.                         |
| 2       | Clutch             | 1000-2000 | 2000                         |                          |
| 3       | Throttle           | 1000-2000 | 2000                         |                          |
| 6       | Brake              | 1000-2000 | 2000                         |                          |
| 9       | Minus-button       | 1000-2000 | 2000                         |                          |
| 10      | Eye-button         | 1000-2000 | 2000                         |                          |
| 11      | !-button           | 1000-2000 | 2000                         |                          |
| 12      | Plus-button        | 1000-2000 | 2000                         |                          |
| 13      | Upshifter          | 1000-2000 | 2000                         |                          |
| 14      | Downshifter        | 1000-2000 | 2000                         |                          |
| 15      | Arrow-down-button  | 1000-2000 | 2000                         |                          |
| 16      | Button-1           | 1000-2000 | 2000                         |                          |
| 17      | Button-4           | 1000-2000 | 2000                         |                          |
| 18      | Button-3           | 1000-2000 | 2000                         |                          |

The above channels is visualized in the code below which you can find in the Github code.

```` C
//FANATEC wheel and pedals
ch1 = ov_chs.chan1_raw; //steering
ch2 = ov_chs.chan2_raw; //clutch
ch3 = ov_chs.chan3_raw; //throttle
ch6 = ov_chs.chan6_raw; //brake
ch9 = ov_chs.chan9_raw; //- button
ch10 = ov_chs.chan10_raw; //eye button
ch11 = ov_chs.chan11_raw; //! button
ch12 = ov_chs.chan12_raw; //+ button
ch13 = ov_chs.chan13_raw; //R flap (Upshifter)
ch14 = ov_chs.chan14_raw; //L flap (Downshifter)
ch15 = ov_chs.chan15_raw; //Arrow down button
ch16 = ov_chs.chan16_raw; //Button 1 - Used for middle differential
ch17 = ov_chs.chan17_raw; //Button 4 - Used for back differential
ch18 = ov_chs.chan18_raw; //Button 3 - Used for front differential

````

So for example the code block below shows that we uses `ov_chs.chan3_raw` directly into the map function, write the correct speed to our `myservoesc`, which is our speed. The range is as above from 1000 to 2000 in PWM, and the max value is `130`, and min is `90` which is idle.

```` C
if (gear == 1)
{
    // NEW SPEEDER 
    int speed = map(ov_chs.chan3_raw, 1000, 2000, 130, 90); 
    myservoesc.write(speed);
}
````








## 4. Troubleshooting
