#include "M3508_Motor.h"
extern uint8_t stop_flag;
extern uint8_t angle_flag;

M3508_Motor::M3508_Motor(const float ratio, float target_angle) : ratio_(ratio),
                                                                  spid_(0.007f, 0.0f, 0.0f, 10.0f, 3.0f),
                                                                  ppid_(160.0f, 0.1f, 0.000000001f, 10.0f, 469 * 6.0f),
                                                                  control_method_(TORQUE), target_angle_(target_angle)
{
};

void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    //位运算解码
    ecd_ = rx_data[0] << 8 | rx_data[1];
    ecd_angle_ = linearMapping(ecd_, 0, 8191, 0, 360);
    speed_raw_ = rx_data[2] << 8 | rx_data[3];
    fdb_speed_ = float(speed_raw_);
    current_read_ = rx_data[4] << 8 | rx_data[5];
    current_ = linearMapping(current_read_, -16384, 16384, -20, 20);
    temp_ = rx_data[6];
    //角度解算
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180) delta_ecd_angle_ = delta_ecd_angle_ - 360;
    else if (delta_ecd_angle_ < -180) delta_ecd_angle_ = 360 + delta_ecd_angle_;
    delta_angle_ = delta_ecd_angle_ / ratio_;
    last_ecd_angle_ = ecd_angle_;
    if (angle_flag == 1)
    {
        fdb_angle_ += delta_angle_;
    }
    else
    {
        fdb_angle_ = 0.0f;
        angle_flag = 1;
    }
    while (fdb_angle_ > 180) fdb_angle_ -= 360;
    while (fdb_angle_ < -180) fdb_angle_ += 360;
}

float M3508_Motor::linearMapping(float in, float in_min, float in_max, float out_min, float out_max)
{
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity)
{
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    spid_.reset();
    ppid_.reset();
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity)
{
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    spid_.reset();
}

void M3508_Motor::SetIntensity(float intensity)
{
    control_method_ = TORQUE;
    feedforward_intensity_ = intensity;
}

void M3508_Motor::handle()
{
    if (stop_flag == 1)
    {
        given_current_ = 0.0f;
        return;
    }
    switch (control_method_)
    {
    case TORQUE:
        output_intensity_ = feedforward_intensity_;
        break;
    case SPEED:
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;
    case POSITION_SPEED:
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;
    }
    if (output_intensity_ > 3) output_intensity_ = 3;
    else if (output_intensity_ < -3) output_intensity_ = -3;
    float given_current_raw_ = output_intensity_ / 0.3;
    given_current_ = static_cast<int16_t>(linearMapping(given_current_raw_, -20, 20, -16384,
                                                        16384));
}

float M3508_Motor::FeedforwardIntensityCalc(float current_angle)
{
    const float C = 0.5 * 0.0552 * 9.81f;
    return (C + F) * sin(current_angle * 3.1415926f / 180.0f);
}
