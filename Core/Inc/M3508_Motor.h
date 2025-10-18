//
// Created by Nebula on 2025/10/3.
//

#ifndef TIMER_M3508_MOTOR_H
#define TIMER_M3508_MOTOR_H
#include "main.h"
#include "pid"

class M3508_Motor
{
private:
    const float ratio_;
    float angle_ = 0.f;
    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float rotate_speed_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;

    // 编码器相关变量
    uint16_t ecd_ = 0;
    uint16_t given_current_ = 0;

public:
    explicit M3508_Motor(const float ratio) : ratio_(ratio)
    {
    };

    void canRxMsgCallback(const uint8_t rx_data[8]);
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);
};

#endif //TIMER_M3508_MOTOR_H
