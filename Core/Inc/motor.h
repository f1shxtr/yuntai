//
// Created by 嘉佳 on 2025/10/3.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "PID.h"
#include "main.h"
class Motor {
private:
    const float ratio_;
    int16_t can_id_ = -1;

    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float rotate_speed_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;

public:
    float angle_ = 0.f;
    PID spid_, ppid_; // make PID instance members so each Motor can have its own PID params
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

    // backward-compatible simple constructor
    explicit Motor(const float ratio):
        ratio_(ratio), can_id_(-1), spid_(), ppid_(),
        target_angle_(0.0f), fdb_angle_(0.0f), target_speed_(0.0f), fdb_speed_(0.0f), feedforward_speed_(0.0f),
        feedforward_intensity_(0.0f), output_intensity_(0.0f), control_method_(TORQUE) {
        spid_.reset();
        ppid_.reset();
    }

    // detailed constructor with per-PID parameters and CAN id
    Motor(const float ratio, int16_t can_id,
          float p_kp, float p_ki, float p_kd,
          float s_kp, float s_ki, float s_kd,
          float p_imax, float s_imax,
          float p_out_max, float s_out_max,
          float p_d_filter_k, float s_d_filter_k);

    void can_rx_msg_callback(const uint8_t rx_data[8]);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    void handle();
    float FeedforwardIntensityCalc(float current_angle);
};
#endif