// IMU.cpp
#include "IMU.h"
#include "bmi088.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 构造：以成员初始化调用 Mahony 构造（假设有该构造）
IMU::IMU(const float& dt, const float& kg, const float& g_thres,
         const float R_imu[3][3], const float gyro_bias[3])
    : mahony_(dt, kg, g_thres) // 如果 Mahony 构造器不同，请改这里
{
  // 复制安装方向矩阵
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R_imu_[i][j] = R_imu[i][j];

  // 复制陀螺零漂补偿
  gyro_bias_[0] = gyro_bias[0];
  gyro_bias_[1] = gyro_bias[1];
  gyro_bias_[2] = gyro_bias[2];

  // 初始化成员
  memset(&raw_data_, 0, sizeof(raw_data_));
  for (int i = 0; i < 3; ++i) {
    gyro_sensor_[i] = gyro_world_[i] = 0.0f;
    gyro_sensor_dps_[i] = gyro_world_dps_[i] = 0.0f;
    accel_sensor_[i] = accel_world_[i] = 0.0f;
  }
  euler_deg_ = EulerAngle_t(0.0f, 0.0f, 0.0f);
  euler_rad_ = EulerAngle_t(0.0f, 0.0f, 0.0f);
  q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
}

// init: 初始化 imu 硬件，并用初始欧拉角设置四元数
void IMU::init(EulerAngle_t euler_deg_init) {
  // 初始化底层 BMI088
  bmi088_init();

  // 将初始欧拉角转四元数（ZYX 顺序）
  float yaw   = euler_deg_init.yaw   * (M_PI / 180.0f);
  float pitch = euler_deg_init.pitch * (M_PI / 180.0f);
  float roll  = euler_deg_init.roll  * (M_PI / 180.0f);

  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  q_[0] = cr * cp * cy + sr * sp * sy;
  q_[1] = sr * cp * cy - cr * sp * sy;
  q_[2] = cr * sp * cy + sr * cp * sy;
  q_[3] = cr * cp * sy - sr * sp * cy;

  // 如果 Mahony 需要内部初始化当前四元数，可尝试：
  // mahony_.setQuaternion(q_); // 如果存在该接口
}

// readSensor: 从 BMI088 读取原始寄存器并换算到物理量（单位： accel -> m/s^2, gyro -> deg/s 存 raw）
void IMU::readSensor() {
  uint8_t buf[6];

  // ----- Accel -----
  // 读取量程寄存器（0x41）以判断 range（0..3 -> 3/6/12/24g）
  uint8_t raw_range = 0;
  bmi088_accel_read_reg(0x41, &raw_range, 1);
  raw_range &= 0x03;

  // 读取加速度六字节
  bmi088_accel_read_reg(0x12, buf, 6);
  int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

  // 量程系数： raw_range=0 -> 3g, 1->6g, 2->12g, 3->24g
  float scale_g = (1 << (raw_range + 1)) * 1.5f; // 3,6,12,24
  const float G = 9.80665f;
  float accel_x_mps2 = (float)ax_raw / 32768.0f * scale_g * G;
  float accel_y_mps2 = (float)ay_raw / 32768.0f * scale_g * G;
  float accel_z_mps2 = (float)az_raw / 32768.0f * scale_g * G;

  raw_data_.accel[0] = accel_x_mps2;
  raw_data_.accel[1] = accel_y_mps2;
  raw_data_.accel[2] = accel_z_mps2;

  // ----- Gyro -----
  uint8_t gyro_range = 0;
  bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
  gyro_range &= 0x07;

  bmi088_gyro_read_reg(0x02, buf, 6);
  int16_t gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

  // full scale in dps: 2000/(1<<range) (range 0->2000,1->1000,...)
  float full_scale_dps = 2000.0f / (1 << gyro_range);
  float gyro_scale_dps = full_scale_dps / 32768.0f;

  float gx_dps = gx_raw * gyro_scale_dps;
  float gy_dps = gy_raw * gyro_scale_dps;
  float gz_dps = gz_raw * gyro_scale_dps;

  // store raw_data_ in the units of deg/s for compatibility
  raw_data_.gyro[0] = gx_dps;
  raw_data_.gyro[1] = gy_dps;
  raw_data_.gyro[2] = gz_dps;

  // temp: if available, not used here
  raw_data_.temp[0] = 0.0f;
}

// update: 将 raw_data_ 换算到 sensor/world，调用 Mahony 更新 q_ 并计算欧拉角
void IMU::update(void) {
  // 1) 从 raw_data_ -> sensor 数组（accel in m/s^2, gyro in deg/s）
  for (int i = 0; i < 3; ++i) {
    accel_sensor_[i] = raw_data_.accel[i];                // m/s^2
    gyro_world_dps_[i] = raw_data_.gyro[i];               // deg/s (will convert)
    // convert deg/s to rad/s and remove bias (bias in rad/s? your gyro_bias_ likely in deg/s or rad/s)
    // We assume gyro_bias_ is in deg/s as user provided 0.0s; if it is rad/s adjust accordingly.
    float gyro_deg_per_s = raw_data_.gyro[i] - gyro_bias_[i]; // deg/s
    gyro_sensor_[i] = gyro_deg_per_s * (M_PI / 180.0f);      // rad/s (sensor frame)
    gyro_sensor_dps_[i] = gyro_deg_per_s;
  }

  // 2) 传感器坐标到机体/world 坐标（R_imu_ * v_sensor）
  for (int i = 0; i < 3; ++i) {
    accel_world_[i] = 0.0f;
    gyro_world_[i]  = 0.0f;
    for (int j = 0; j < 3; ++j) {
      accel_world_[i] += R_imu_[i][j] * accel_sensor_[j];
      gyro_world_[i]  += R_imu_[i][j] * gyro_sensor_[j];
    }
  }

  // 3) 调用 Mahony 更新四元数
  // Mahony::update(q, ws, as) 这里 q_ 会被更新并写回 q
  mahony_.update(q_, gyro_world_, accel_world_);

  // 4) 四元数 -> 欧拉角（弧度）
  float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];

  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  euler_rad_.roll = atan2f(sinr_cosp, cosr_cosp);

  // pitch (y-axis)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabsf(sinp) >= 1.0f)
    euler_rad_.pitch = copysignf(M_PI / 2.0f, sinp);
  else
    euler_rad_.pitch = asinf(sinp);

  // yaw (z-axis)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  euler_rad_.yaw = atan2f(siny_cosp, cosy_cosp);

  // 5) 转为角度并保存
  euler_deg_.roll  = euler_rad_.roll  * (180.0f / M_PI);
  euler_deg_.pitch = euler_rad_.pitch * (180.0f / M_PI);
  euler_deg_.yaw   = euler_rad_.yaw   * (180.0f / M_PI);
}