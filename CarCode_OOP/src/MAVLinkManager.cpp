#include "MAVLinkManager.h"
#include "RCInput.h"
#include "Failsafe.h"

MAVLinkManager::MAVLinkManager(RCInput* rcInput, Failsafe* fs)
    : port(nullptr), rc(rcInput), failsafe(fs),
      prevHeartbeat(0), prevAttitude(0),
      sysid(1), compid(190), type(MAV_TYPE_GROUND_ROVER),
      system_type(MAV_TYPE_GENERIC), autopilot_type(MAV_AUTOPILOT_GENERIC),
      system_mode(MAV_MODE_TEST_ARMED), custom_mode(MAV_MODE_FLAG_SAFETY_ARMED),
      system_state(MAV_STATE_STANDBY) {
}

void MAVLinkManager::begin(HardwareSerial& serialPort) {
    port = &serialPort;
    port->begin(115200);
    while (!port->available()) {
        ;
    }
}

void MAVLinkManager::process() {
    if (!port) return;

    mavlink_message_t msg;
    mavlink_status_t status;

    while (port->available()) {
        uint8_t c = port->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                } break;
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys;
                    mavlink_msg_sys_status_decode(&msg, &sys);
                } break;
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t p;
                    mavlink_msg_param_value_decode(&msg, &p);
                } break;
                case MAVLINK_MSG_ID_RAW_IMU: {
                    mavlink_raw_imu_t raw;
                    mavlink_msg_raw_imu_decode(&msg, &raw);
                } break;
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                } break;
                case MAVLINK_MSG_ID_SET_MODE: {
                    mavlink_set_mode_t sm;
                    mavlink_msg_set_mode_decode(&msg, &sm);
                } break;
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
                    mavlink_rc_channels_raw_t ch;
                    mavlink_msg_rc_channels_raw_decode(&msg, &ch);
                } break;
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
                    mavlink_rc_channels_override_t ov;
                    mavlink_msg_rc_channels_override_decode(&msg, &ov);
                    if (rc) {
                        rc->updateFromOverride(ov);
                    }
                    if (failsafe) {
                        failsafe->resetTimer();
                    }
                } break;
                default:
                    break;
            }
        }
    }
}

void MAVLinkManager::update(float roll, float pitch, float throttle, float yaw, int16_t hallSlope, float rpm, uint8_t grip) {
    unsigned long now = millis();

    if ((now - prevHeartbeat) >= heartbeatInterval) {
        prevHeartbeat = now;
        sendHeartbeat();
    }

    if ((now - prevAttitude) >= attitudeInterval) {
        prevAttitude = now;
        sendAttitude(roll, pitch, throttle, yaw, hallSlope, rpm, grip);
    }
}

void MAVLinkManager::sendHeartbeat() {
    if (!port) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(1, 1, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port->write(buf, len);
}

void MAVLinkManager::sendAttitude(float roll, float pitch, float throttle, float yaw, int16_t hallSlope, float rpm, uint8_t grip) {
    if (!port) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    float rotx = 0.0f;
    float roty = 0.0f;
    float rotz = 0.0f;

    mavlink_msg_attitude_pack(1, 1, &msg, 1, roll, pitch, yaw, rotx, roty, rotz);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port->write(buf, len);

    mavlink_msg_sys_status_pack(
        1,
        1,
        &msg,
        1,
        1,
        1,
        500,
        static_cast<uint16_t>(hallSlope),  // erstatter 7400
        static_cast<uint16_t>(rpm),
        static_cast<int8_t>(grip),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    );

    uint16_t len2 = mavlink_msg_to_send_buffer(buf, &msg);
    port->write(buf, len2);
}