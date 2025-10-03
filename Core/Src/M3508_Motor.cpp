#include "M3508_Motor.h"

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    //位运算解码
    ecd_ = rx_data[0] << 8 | rx_data[1];
    ecd_angle_ = linearMapping(ecd_, 0, 8191, 0, 360);
    rotate_speed_ = rx_data[2] << 8 | rx_data[3];
    given_current_ = rx_data[4] << 8 | rx_data[5];
    current_ = linearMapping(given_current_, -16384, 16384, -20, 20);
    temp_ = rx_data[6];
    //角度解算
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180) delta_ecd_angle_ = 360 - delta_ecd_angle_;
    else if (delta_ecd_angle_ < -180) delta_ecd_angle_ = 360 + delta_ecd_angle_;
    delta_angle_ = delta_ecd_angle_ / ratio_;
    last_ecd_angle_ = ecd_angle_;
    angle_ += delta_angle_;
}

float M3508_Motor::linearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
