#include "imu.h"
#include "bmi088.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// ===================== IMU 构造函数 ===================== //
IMU::IMU(const float& dt, const float& kg, const float& g_thres,
         const float R_imu[3][3], const float gyro_bias[3])
    : mahony_(dt, kg, g_thres)
{
    // 拷贝安装矩阵
    std::memcpy(R_imu_, R_imu, sizeof(float) * 9);

    // 拷贝零飘
    std::memcpy(gyro_bias_, gyro_bias, sizeof(float) * 3);
}

// ===================== 初始化 ===================== //
void IMU::init(EulerAngle_t euler_deg_init)
{
    // 初始化 quaternion，用欧拉角转四元数（简易版）
    float cy = cosf(euler_deg_init.yaw   * 0.5f * M_PI/180.f);
    float sy = sinf(euler_deg_init.yaw   * 0.5f * M_PI/180.f);
    float cp = cosf(euler_deg_init.pitch * 0.5f * M_PI/180.f);
    float sp = sinf(euler_deg_init.pitch * 0.5f * M_PI/180.f);
    float cr = cosf(euler_deg_init.roll  * 0.5f * M_PI/180.f);
    float sr = sinf(euler_deg_init.roll  * 0.5f * M_PI/180.f);

    q_[0] = cr * cp * cy + sr * sp * sy;
    q_[1] = sr * cp * cy - cr * sp * sy;
    q_[2] = cr * sp * cy + sr * cp * sy;
    q_[3] = cr * cp * sy - sr * sp * cy;

    // BMI088 初始化
    bmi088_init();
}

// ========================================================= //
//               IMU::readSensor() 读取 BMI088               //
// ========================================================= //
void IMU::readSensor()
{
    // ---------------- ACC ---------------- //
    uint8_t raw_range;
    uint8_t rx_acc_data[6];

    bmi088_accel_read_reg(0x41, &raw_range, 1);
    raw_range &= 0x03;

    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    int16_t Ax_int = (rx_acc_data[0] << 8) | rx_acc_data[1];
    int16_t Ay_int = (rx_acc_data[2] << 8) | rx_acc_data[3];
    int16_t Az_int = (rx_acc_data[4] << 8) | rx_acc_data[5];

    float range_g = (1 << (raw_range + 1)) * 1.5f; // 3,6,12,24g

    float Ax_mps2 = (float)Ax_int / 32768.0f * range_g * 9.80665f;
    float Ay_mps2 = (float)Ay_int / 32768.0f * range_g * 9.80665f;
    float Az_mps2 = (float)Az_int / 32768.0f * range_g * 9.80665f;

    accel_sensor_[0] = Ax_mps2;
    accel_sensor_[1] = Ay_mps2;
    accel_sensor_[2] = Az_mps2;

    // ---------------- GYRO ---------------- //
    uint8_t gyro_range;
    uint8_t rx_gyro_data[6];

    bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
    gyro_range &= 0x07;

    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

    int16_t Gx_int = (rx_gyro_data[0] << 8) | rx_gyro_data[1];
    int16_t Gy_int = (rx_gyro_data[2] << 8) | rx_gyro_data[3];
    int16_t Gz_int = (rx_gyro_data[4] << 8) | rx_gyro_data[5];

    float full_scale_dps = 2000.0f / (1 << gyro_range);
    float dps_per_lsb = full_scale_dps / 32768.0f;

    // 去零飘后的角速度(°/s)
    gyro_sensor_dps_[0] = Gx_int * dps_per_lsb - gyro_bias_[0];
    gyro_sensor_dps_[1] = Gy_int * dps_per_lsb - gyro_bias_[1];
    gyro_sensor_dps_[2] = Gz_int * dps_per_lsb - gyro_bias_[2];

    // 转 rad/s
    gyro_sensor_[0] = gyro_sensor_dps_[0] * M_PI / 180.f;
    gyro_sensor_[1] = gyro_sensor_dps_[1] * M_PI / 180.f;
    gyro_sensor_[2] = gyro_sensor_dps_[2] * M_PI / 180.f;
}

// ========================================================= //
//                  update() = Mahony融合                    //
// ========================================================= //
void IMU::update()
{
    // 将 R_imu 旋转矩阵用于坐标变换
    for(int i=0; i<3; i++)
    {
        accel_world_[i] =
            R_imu_[i][0] * accel_sensor_[0] +
            R_imu_[i][1] * accel_sensor_[1] +
            R_imu_[i][2] * accel_sensor_[2];

        gyro_world_[i] =
            R_imu_[i][0] * gyro_sensor_[0] +
            R_imu_[i][1] * gyro_sensor_[1] +
            R_imu_[i][2] * gyro_sensor_[2];
    }

    // 调用你原样的 Mahony::update(q, gyro, accel)
    mahony_.update(q_, gyro_world_, accel_world_);

    // 由四元数得到欧拉角
    float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];

    float sinr = 2.f * (qw*qx + qy*qz);
    float cosr = 1.f - 2.f * (qx*qx + qy*qy);
    euler_rad_.roll = atan2f(sinr, cosr);

    float sinp = 2.f * (qw*qy - qz*qx);
    if (fabs(sinp) >= 1)
        euler_rad_.pitch = copysignf(M_PI/2, sinp);
    else
        euler_rad_.pitch = asinf(sinp);

    float siny = 2.f * (qw*qz + qx*qy);
    float cosy = 1.f - 2.f * (qy*qy + qz*qz);
    euler_rad_.yaw = atan2f(siny, cosy);

    // 转成角度
    euler_deg_.yaw   = euler_rad_.yaw   * 180.f/M_PI;
    euler_deg_.pitch = euler_rad_.pitch * 180.f/M_PI;
    euler_deg_.roll  = euler_rad_.roll  * 180.f/M_PI;
}