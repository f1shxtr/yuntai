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

    uint8_t raw_range;
    uint8_t gyro_range;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    raw_range = raw_range & 0x03;
    // code here

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    int16_t Accel_X_int16 = rx_acc_data[0] * 256 + rx_acc_data[1];
    int16_t Accel_Y_int16 = rx_acc_data[2] * 256 + rx_acc_data[3];
    int16_t Accel_Z_int16 = rx_acc_data[4] * 256 + rx_acc_data[5];

    raw_data_.accel[0] = (float)Accel_X_int16;
    raw_data_.accel[1] = (float)Accel_Y_int16;
    raw_data_.accel[2] = (float)Accel_Z_int16;
    accel_sensor_[0] = (float)Accel_X_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;
    accel_sensor_[1] = (float)Accel_Y_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;
    accel_sensor_[2] = (float)Accel_Z_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;

    gyro_sensor_[0] = accel_sensor_[0];
    gyro_sensor_[1] = accel_sensor_[1];
    gyro_sensor_[2] = accel_sensor_[2];
}

// ========================================================= //
//
//               IMU::readSensor() 读取 BMI088               //
// ========================================================= //
void IMU::readSensor()
{
    uint8_t raw_range;
    uint8_t gyro_range;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    raw_range = raw_range & 0x03;
    // code here

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    int16_t Accel_X_int16 = rx_acc_data[0] * 256 + rx_acc_data[1];
    int16_t Accel_Y_int16 = rx_acc_data[2] * 256 + rx_acc_data[3];
    int16_t Accel_Z_int16 = rx_acc_data[4] * 256 + rx_acc_data[5];

    raw_data_.accel[0] = (float)Accel_X_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;
    raw_data_.accel[1] = (float)Accel_Y_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;
    raw_data_.accel[2] = (float)Accel_Z_int16 / 32768.0f * 1000.0f * (1 << (raw_range + 1)) * 1.5f;
    accel_sensor_[0] = std::atan(raw_data_.accel[1]/raw_data_.accel[2])* (180 / M_PI);
    accel_sensor_[1] = std::atan(-raw_data_.accel[0]/std::sqrt(raw_data_.accel[1]*raw_data_.accel[1]+raw_data_.accel[2]*raw_data_.accel[2]))* (180 / M_PI);

    // code here


    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
    gyro_range = gyro_range & 0x07;
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    bmi088_gyro_read_reg(0x02, rx_gyro_data,6);
    // 3. 用量程系数将原始数据转换为常用单位
    int16_t Rate_X = rx_gyro_data[0] * 256 + rx_gyro_data[1];
    int16_t Rate_Y = rx_gyro_data[2] * 256 + rx_gyro_data[3];
    int16_t Rate_Z = rx_gyro_data[4] * 256 + rx_gyro_data[5];

    raw_data_.gyro[0] = (float)Rate_X;
    raw_data_.gyro[1] = (float)Rate_Y;
    raw_data_.gyro[2] = (float)Rate_Z;
    float full_scale_dps = 2000.0f / (1 << gyro_range);
    float gyro_scale = full_scale_dps / 32768.0f;

    // 换算 °/s
    gyro_sensor_dps_[0] = raw_data_.gyro[0] * gyro_scale;
    gyro_sensor_dps_[1] = raw_data_.gyro[1] * gyro_scale;
    gyro_sensor_dps_[2] = raw_data_.gyro[2] * gyro_scale;
    gyro_sensor_[0] += gyro_sensor_dps_[0] * div_t - gyro_bias_[0];
    gyro_sensor_[1] += gyro_sensor_dps_[1] * div_t - gyro_bias_[1];
    gyro_sensor_[2] += gyro_sensor_dps_[2] * div_t - gyro_bias_[2];
}

// ========================================================= //
//                  update() = Mahony融合                    //
// ========================================================= //
void IMU::filter(float k)
{
    roll=gyro_sensor_[0]+(accel_sensor_[0]-gyro_sensor_[0])*k;
    pitch=gyro_sensor_[1]+(accel_sensor_[1]-gyro_sensor_[1])*k;
    yaw=gyro_sensor_[2]+(accel_sensor_[2]-gyro_sensor_[2])*k;
}
void IMU::update()
{
    filter(0.2f);
}
// 将 R_imu 旋转矩阵用于坐标变换
/*    for(int i=0; i<3; i++)
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
    euler_deg_.roll  = euler_rad_.roll  * 180.f/M_PI; */
