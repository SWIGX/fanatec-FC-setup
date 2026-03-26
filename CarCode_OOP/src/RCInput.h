#pragma once

#include <MAVLink.h>

class RCInput {
public:
    RCInput();
    void updateFromOverride(const mavlink_rc_channels_override_t& ov);

    int ch1, ch2, ch3, ch4, ch5, ch6;
    int ch7, ch8, ch9, ch10, ch11, ch12;
    int ch13, ch14, ch15, ch16, ch17, ch18;
};
