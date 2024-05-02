//import the libraries
#include <string.h>
#include <Arduino.h>
#include <MAVLink.h>
#include <PWMServo.h>
#include <MadgwickAHRS.h>
#include "FastIMU.h"
#include <Wire.h>

#define PIN_RESET 15
#define PIN_DC    6
#define PIN_CS    10
#define PIN_SCK   13
#define PIN_MOSI2  11

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6050 IMU; 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

/*
 * TEENSY 3.2, Serial
 * 
 */


PWMServo myservo; 
PWMServo myservoesc; 
PWMServo myservocam;
PWMServo myservocam2;

PWMServo myservofrontdif;
PWMServo myservomiddledif;
PWMServo myservobackdif;
bool is_back_dif_open = false;
bool is_middle_dif_open = false;
bool is_front_dif_open = false;

Madgwick filter;

//Available flight modes
static const String STABILIZE = "STABILIZE";

//####################################
//#Initial Configuration of the drone#
//####################################

boolean current_arm = true;
String current_mode = STABILIZE;
float current_roll = 0;
float current_pitch = 0;
float current_throttle = 0; //Min value is 1150 to run motors
float current_yaw = 0;

int ch1 = 0;
int ch2 = 0;
int ch3 = 0;
int ch4 = 0;
int ch5 = 0;
int ch6 = 0;
int ch7 = 0;
int ch8 = 0;

int ch9 = 0;
int ch10 = 0;
int ch11 = 0;
int ch12 = 0;
int ch13 = 0;
int ch14 = 0;
int ch15 = 0;
int ch16 = 0;
int ch17 = 0;
int ch18 = 0;

//HardwareSerial SerialMAV(2); //default pins for 16RX, 17TX
#define SerialMAV Serial1 //Serial //Serial1 is HW serial //115200 ///dev/ttyACM0 on air pi

//#######################
//#Mavlink configuration#
//#######################

int sysid = 1;//255;//GCS                   ///<  1 PX, 255 ground station
int compid = 190;//Mission Planner          ///< The component sending the message
int type = MAV_TYPE_GROUND_ROVER;           ///< This system is an airplane / fixed wing / rc car

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

// Hardware definitions
uint8_t system_mode = MAV_MODE_TEST_ARMED; /// /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
uint32_t custom_mode = MAV_MODE_FLAG_SAFETY_ARMED; ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 10;  // next interval to count
unsigned long previousMillisMAVLink2 = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink2 = 1000;  // next interval to count


int gear = 0;
bool plus_button_pressed = false;
bool minus_button_pressed = false;


const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from APM. 60 = one minute.
int num_hbs_past = num_hbs;

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll = 0;
  float pitch = 0;
  float heading = 0;

int steering_trim = 0;//43;

//Default Arduino function
void setup() {

  Wire.begin();
  Wire.setClock(400000); //400khz clock
  SerialMAV.begin(115200);
  while (!SerialMAV.available()) {
    ;
  }
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
  //  while (true) {
  //    ;
  //  }
  }
  
  IMU.setIMUGeometry(1);//1 rotate 90 under roof mount, diode left //6 roof mount across diode backwards
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
  
  myservo.attach(5,544,2400);
  myservoesc.attach(6);//,1000,2000);

  //Differentials controls
  // myservofrontdif.attach(123);
  // myservomiddledif.attach(123);
  // myservobackdif.attach(123);
 
  myservocam.attach(9);
  myservocam2.attach(10,1000,2000);
     
  //SerialMAV.begin(115200);//, SERIAL_8N1, 16, 17);
  myservocam.write(90);
  myservocam2.write(90);
  myservoesc.write(90);
  //myservo.write(135+steering_trim);


}



void loop() {

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  // Update the Madgwick filter
  filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, accelData.accelX, accelData.accelY, accelData.accelZ);
    
  // set the heading, pitch and roll for transmit
  roll = filter.getRoll()*(PI/180);
  pitch = filter.getPitch()*(PI/180);
  heading = filter.getYaw()*(PI/180);
 
  unsigned long currentMillisMAVLink2 = millis();
  //safety
  if (currentMillisMAVLink2 - previousMillisMAVLink2 >= next_interval_MAVLink2) 
  {
    // Timing variables
    previousMillisMAVLink2 = currentMillisMAVLink2;
    
    myservoesc.write(90);
    // myservo.write(90+steering_trim); //myservo.write(90+43); //original mojave offset

    mav_heartbeat_pack();
  }
 
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {
    // Timing variables
    previousMillisMAVLink = currentMillisMAVLink;
    
    mav_send_attitude(roll, pitch, current_throttle, heading);

  }

  // Check reception buffer
  comm_receive();
}

/* This function gets message from the APM and interprete for Mavlink common messages */
void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  //Checks if drone is connected
  while(SerialMAV.available()) {
    uint8_t c = SerialMAV.read();
        
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //Indicates data flow
     // Serial.println("+");
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            //Serial.println("MAVLINK_MSG_ID_HEARTBEAT");
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
           // Serial.print("State: "); Serial.println(hb.base_mode == 209 ? "Armed" : "Disarmed");
           // Serial.print("Mode: ");
            switch(hb.custom_mode) {
              case 0:
           //     Serial.println("Stabilize");
              break;
              case 2:
           //     Serial.println("AltHold");
              break;
              case 3:
          //      Serial.println("Auto");
              break;
              case 5:
           //     Serial.println("Loiter");
              break;
              case 7:
           //     Serial.println("Circle");
              break;
              default:
            //    Serial.println("Mode not known");
              break;
            }
            //Stablize = 0
            //AltHold = 2
            //Auto = 3
            //Loiter = 5
            //Circle = 7
          }
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            //Serial.println("MAVLINK_MSG_ID_SYS_STATUS");
            //Serial.println("Battery (V): ");
            //Serial.println(sys_status.voltage_battery);
          }
          break;
        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
            //Serial.println("MAVLINK_MSG_ID_PARAM_VALUE");
          }
          break;
        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            //Serial.println("MAVLINK_MSG_ID_RAW_IMU");
          }
          break;
        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            //Serial.println("MAVLINK_MSG_ID_ATTITUDE");
            //Serial.println("ROLL: ");
            //Serial.println(attitude.roll);
          }
          break;
        case MAVLINK_MSG_ID_SET_MODE: // #11
          {
            mavlink_set_mode_t set_mode;
            mavlink_msg_set_mode_decode(&msg, &set_mode);
            /*
            Serial.println("CUSTOM_MODE: ");
            Serial.println(set_mode.custom_mode);
            Serial.println("TARGET_SYSTEM: ");
            Serial.println(set_mode.target_system);
            Serial.println("BASE_MODE: ");
            Serial.println(set_mode.base_mode);
            */
          }
          break;
          //Not overriden channels
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW: // #35
          {
           /* 
           *  RC (Radio controll) channels are the inputs and outputs for controlling all 
           *  actions called from joystick / mission planner. E.g. arm, throttle, pitch.
           */ 
            mavlink_rc_channels_raw_t chs;
            mavlink_msg_rc_channels_raw_decode(&msg, &chs);
/*
            Serial.print("Roll: ");  Serial.print(chs.chan1_raw);
            Serial.println();
            Serial.print("Pitch: ");  Serial.print(chs.chan2_raw + '\n');
            Serial.println();
            Serial.print("Throttle: ");  Serial.print(chs.chan3_raw + '\n');
            Serial.println();
*/
            
          }
          break;
          //Overriden channels for radio values
          case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // #70
          {
            mavlink_rc_channels_override_t ov_chs;
            mavlink_msg_rc_channels_override_decode(&msg, &ov_chs);

            current_roll = ov_chs.chan1_raw;
            current_pitch = ov_chs.chan2_raw;
            current_yaw = ov_chs.chan4_raw;

            current_throttle = ov_chs.chan3_raw;
            
            //Mapping raw channel data to steering intervals
            int val = map(ov_chs.chan1_raw, 1000, 2000, 180, 45); 
            myservo.write(val+steering_trim); 
            
            //Upshifter
            if(ch13 < 1500){
              if(gear < 1){
                gear++;
              }
            }

            //Downshifter
            if(ch14 < 1500){
              if(gear > -1){
                gear--;
              }
            }


            if (gear == 1)
            {
              // NEW SPEEDER 
              int speed = map(ov_chs.chan3_raw, 1000, 2000, 130, 90); 
              myservoesc.write(speed);
            }

            else if (gear == -1)
            {
              // NEW REVERSE SPEED
              int backSpeed = map(ov_chs.chan3_raw, 1000, 2000, 70, 90);
              myservoesc.write(backSpeed);
            }

            //If the minus button has previosly been pressed, but is now released
            if(ch9 > 1600 && minus_button_pressed == true){
              minus_button_pressed = false;
            }
            //If the minus button is being pressed and we are in the "first cycle" of the loop
            if(ch9 < 1600 && minus_button_pressed == false)
            {
              minus_button_pressed = true;
              steering_trim = steering_trim - 1;
              
            }
            //If the plus button has previosly been pressed, but is now released
             if(ch12 > 1600 && plus_button_pressed == true){
              plus_button_pressed = false;
            }
            //If the plus button is being pressed and we are in the "first cycle" of the loop
            if(ch12 < 1600 && plus_button_pressed == false)
            {
              plus_button_pressed = true;
              steering_trim = steering_trim + 1;
              
            }


            if(ch16 < 1500){
              if(is_middle_dif_open){
                //Find out what PWM corresponds to closed
                // myservomiddledif.write()
                is_middle_dif_open = false;
              }
              else{
                //Find out what PWM corresponds to open
                // myservomiddledif.write()
                is_middle_dif_open = true;
              }
            }

            if(ch17 < 1500){
              if(is_back_dif_open){
                //Find out what PWM corresponds to closed
                // myservobackdif.write()
                is_back_dif_open = false;
              }
              else{
                //Find out what PWM corresponds to open
                // myservobackdif.write()
                is_back_dif_open = true;
              }
            }

            if(ch18 < 1500){
              if(is_front_dif_open){
                //Find out what PWM corresponds to closed
                // myservofrontdif.write()
                is_front_dif_open = false;
              }
              else{
                //Find out what PWM corresponds to open
                // myservofrontdif.write()
                is_front_dif_open = true;
              }
            }
            

            //FANATEC wheel and pedals
            ch1 = ov_chs.chan1_raw; //steering
            ch2 = ov_chs.chan2_raw; //clutch
            ch3 = ov_chs.chan3_raw; //throttle

            ch6 = ov_chs.chan6_raw; //brake
 
            ch9 = ov_chs.chan9_raw; //- button
            // ch10 = ov_chs.chan10_raw; //eye button
            // ch11 = ov_chs.chan11_raw; //! button
            ch12 = ov_chs.chan12_raw; //+ button
            ch13 = ov_chs.chan13_raw; //R flap (Upshifter)
            ch14 = ov_chs.chan14_raw; //L flap (Downshifter)
            // ch15 = ov_chs.chan15_raw; //Arrow down button
            ch16 = ov_chs.chan16_raw; //Button 1 - Used for middle differential
            ch17 = ov_chs.chan17_raw; //Button 4 - Used for back differential
            ch18 = ov_chs.chan18_raw; //Button 3 - Used for front differential


            
            unsigned long currentMillisMAVLink2 = millis();
            // Timing variables
            previousMillisMAVLink2 = currentMillisMAVLink2;
          }
          break;
      }
    }
  }
}


void mav_heartbeat_pack() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
//  mavlink_msg_heartbeat_pack(0x01,0xFA, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1,1, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_arm_pack(boolean state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Arm the drone
  //400 stands for MAV_CMD_COMPONENT_ARM_DISARM
  // 1 an 8'th argument is for ARM (0 for DISARM)
  if(state) {
    //ARM
    mavlink_msg_command_long_pack(1, 200, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
  }else {
    //DISARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,0.0,0,0,0,0,0,0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_send_attitude(float roll, float pitch, float throttle, float yaw)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
   
  float rotx = 0;
  float roty = 0;
  float rotz = 0;

  mavlink_msg_attitude_pack(1, 1, &msg, 1, roll, pitch, yaw, rotx, roty, rotz); //check types
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
  //memset(bufTx, 0xFF, sizeof(bufTx));
  mavlink_msg_sys_status_pack(1, 1, &msg, 1, 1, 1, 500,7400,300,50,0,0,0,0,0,0,0,0,0);
 
  /// Copy the message to send buffer
  uint16_t len2 = mavlink_msg_to_send_buffer(buf, &msg);

   //Write Message    
  SerialMAV.write(buf, len2);
}
