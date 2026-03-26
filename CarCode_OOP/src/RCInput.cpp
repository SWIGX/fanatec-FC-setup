#include "RCInput.h"

RCInput::RCInput()
    : ch1(0), ch2(0), ch3(0), ch4(0), ch5(0), ch6(0),
      ch7(0), ch8(0), ch9(0), ch10(0), ch11(0), ch12(0),
      ch13(0), ch14(0), ch15(0), ch16(0), ch17(0), ch18(0) {
}

void RCInput::updateFromOverride(const mavlink_rc_channels_override_t& ov) {
    ch1 = ov.chan1_raw;
    ch2 = ov.chan2_raw;
    ch3 = ov.chan3_raw;
    ch4 = ov.chan4_raw;
    ch5 = ov.chan5_raw;
    ch6 = ov.chan6_raw;
    ch7 = ov.chan7_raw;
    ch8 = ov.chan8_raw;
    ch9 = ov.chan9_raw;
    ch10 = ov.chan10_raw;
    ch11 = ov.chan11_raw;
    ch12 = ov.chan12_raw;
    ch13 = ov.chan13_raw;
    ch14 = ov.chan14_raw;
    ch15 = ov.chan15_raw;
    ch16 = ov.chan16_raw;
    ch17 = ov.chan17_raw;
    ch18 = ov.chan18_raw;
}
