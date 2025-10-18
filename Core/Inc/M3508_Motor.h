//
// Created by Nebula on 2025/10/3.
//

#ifndef TIMER_M3508_MOTOR_H
#define TIMER_M3508_MOTOR_H
#include "main.h"
#include "pid.h"

class M3508_Motor
{
private:
    PID spid_, ppid_;
    float target_angle_ = 0.0f, fdb_angle_ = 0.0f;
    float target_speed_ = 0.0f, fdb_speed_ = 0.0f, feedforward_speed_ = 0.0f;
    float feedforward_intensity_ = 0.0f, output_intensity_ = 0.0f;

    enum
    {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

    const float ratio_;
    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;

    // 编码器相关变量
    uint16_t ecd_ = 0;
    int16_t speed_raw_ = 0;
    int16_t current_read_ = 0;

    //控制量
    int16_t given_current_ = 0;

public:
    M3508_Motor(const float ratio);
    void canRxMsgCallback(const uint8_t rx_data[8]); //获取输入
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);

    void handle();
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
};

#endif //TIMER_M3508_MOTOR_H
