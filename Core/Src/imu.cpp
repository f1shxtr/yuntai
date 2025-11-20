// Core/Src/imu.cpp
#include "imu.h"
#include "bmi088.h"
#include <cmath>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float wrapAngleDeg(float a)
{
    // 将角度规约到 [-180, 180)
    // 使用 fmodf 更高效：注意负数处理
    a = fmodf(a, 360.0f);
    if (a >= 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

// === 构造与 init 保持不变 ===
// 构造函数实现，签名需与声明完全一致
IMU::IMU(const float& dt, const float& kg, const float& g_thres,
         const float R_imu[3][3], const float gyro_bias[3])
    : mahony_(dt, kg, g_thres)
{
    // 保存积分周期，避免角度积分无限增长
    div_t = dt;

    // 拷贝安装矩阵和陀螺零偏
    std::memcpy(R_imu_, R_imu, sizeof(float) * 9);
    std::memcpy(gyro_bias_, gyro_bias, sizeof(float) * 3);

    // 初始化四元数为单位四元数
    q_[0] = 1.0f; q_[1] = 0.0f; q_[2] = 0.0f; q_[3] = 0.0f;

    // 清零传感器相关缓存
    for (int i = 0; i < 3; ++i) {
        gyro_sensor_[i] = 0.0f;
        gyro_sensor_dps_[i] = 0.0f;
        accel_sensor_[i] = 0.0f;
        raw_data_.accel[i] = 0.0f;
        raw_data_.gyro[i]  = 0.0f;
    }

    // 其它缓冲与状态初始化
    raw_data_.temp[0] = 0.0f;
    for (int i = 0; i < 6; ++i) {
        rx_acc_data[i] = 0;
        rx_gyro_data[i] = 0;
    }

    acc_roll_acc_ = 0.0f;
    acc_pitch_acc_= 0.0f;
    roll = pitch = yaw = 0.0f;
}
// ========================================================= //
//                   读取 BMI088 传感器                      //
// ========================================================= //
void IMU::readSensor()
{
    uint8_t raw_range;
    uint8_t gyro_range;

    // ----------- ACC ------------
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    raw_range &= 0x03;

    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    int16_t ax_raw = rx_acc_data[0] * 256 + rx_acc_data[1];
    int16_t ay_raw = rx_acc_data[2] * 256 + rx_acc_data[3];
    int16_t az_raw = rx_acc_data[4] * 256 + rx_acc_data[5];

    float acc_scale = 1000.0f * (1 << (raw_range + 1)) * 1.5f / 32768.0f;

    raw_data_.accel[0] = ax_raw * acc_scale;
    raw_data_.accel[1] = ay_raw * acc_scale;
    raw_data_.accel[2] = az_raw * acc_scale;

    // ---------- 加速度计角度 ----------
    acc_roll_acc_  = atan2f(raw_data_.accel[1], raw_data_.accel[2]) * 180.0f / M_PI;
    acc_pitch_acc_ = atan2f(-raw_data_.accel[0],
                     sqrtf(raw_data_.accel[1]*raw_data_.accel[1] + raw_data_.accel[2]*raw_data_.accel[2]))
                     * 180.0f / M_PI;

    // 读取加速度计温度寄存器（示例寄存器 0x22，两字节）
    // Core/Src/imu.cpp (替换原有的 if(...) 块)
{
    uint8_t temp_buf[2] = {0};
    // 直接调用读取函数（不比较返回值）
    bmi088_accel_read_reg(0x22, temp_buf, 2);
    int16_t temp_raw_acc = (int16_t)((temp_buf[0] << 8) | temp_buf[1]);
    // 常用近似转换：T = 23 + raw/512
    raw_data_.temp[0] = 23.0f + (float)temp_raw_acc / 512.0f;
}


    // ----------- GYRO ------------
    bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
    gyro_range &= 0x07;

    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

    int16_t gx_raw = rx_gyro_data[0] * 256 + rx_gyro_data[1];
    int16_t gy_raw = rx_gyro_data[2] * 256 + rx_gyro_data[3];
    int16_t gz_raw = rx_gyro_data[4] * 256 + rx_gyro_data[5];

    float full_scale_dps = 2000.0f / (1 << gyro_range);
    float gyro_scale = full_scale_dps / 32768.0f;

    gyro_sensor_dps_[0] = gx_raw * gyro_scale - gyro_bias_[0];
    gyro_sensor_dps_[1] = gy_raw * gyro_scale - gyro_bias_[1];
    gyro_sensor_dps_[2] = gz_raw * gyro_scale - gyro_bias_[2];

    // ---- 积分得到角度 ----
    gyro_sensor_[0] += gyro_sensor_dps_[0] * div_t;
    gyro_sensor_[1] += gyro_sensor_dps_[1] * div_t;
    gyro_sensor_[2] += gyro_sensor_dps_[2] * div_t;

    // 防止积分值无限增长：把积分角规约到 [-180,180)
    gyro_sensor_[0] = wrapAngleDeg(gyro_sensor_[0]);
    gyro_sensor_[1] = wrapAngleDeg(gyro_sensor_[1]);
    gyro_sensor_[2] = wrapAngleDeg(gyro_sensor_[2]);
}

// ========================================================= //
//                简单互补滤波融合 roll/pitch                 //
// ========================================================= //
void IMU::filter(float k)
{
    // k = 加速度计比例因子（0~1） 例如 0.02~0.2
    roll  = gyro_sensor_[0] + k * (acc_roll_acc_  - gyro_sensor_[0]);
    pitch = gyro_sensor_[1] + k * (acc_pitch_acc_ - gyro_sensor_[1]);
    // yaw 无法从加速度计得到，只能纯陀螺仪积分
    yaw = gyro_sensor_[2];

    // 规约输出角度到 [-180,180)
    roll  = wrapAngleDeg(roll);
    pitch = wrapAngleDeg(pitch);
    yaw   = wrapAngleDeg(yaw);
}

// update 保持不变
void IMU::update()
{
    filter(0.2f);   // 互补滤波比例
}
