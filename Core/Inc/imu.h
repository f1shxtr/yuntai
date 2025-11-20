#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include "mahony.h"

// 欧拉角
typedef struct EulerAngle {
  float yaw;
  float pitch;
  float roll;
  explicit EulerAngle(float y = 0, float p = 0, float r = 0)
    : yaw(y), pitch(p), roll(r) {}
} EulerAngle_t;

// 传感器原始数据
typedef struct imuRawData {
  float gyro[3];
  float accel[3];
  float temp[1];
} ImuRawData_t;

class IMU {
public:
  // 设置imu解算算法参数(dt为计算间隔，kg为加速度计融合系数，g_thres为融合开启阈值，用于初始化Mahony)，
  // 传感器安装方向(R_imu[3][3]，默认为I(3)），输入预校准传感器零飘
  IMU(const float& dt, const float& kg, const float& g_thres,
      const float R_imu[3][3], const float gyro_bias[3]);
  // IMU传感器初始化
  void init(EulerAngle_t euler_deg_init);
  // 读取bmi088数据：加速度，角速度，温度
  void readSensor();
  // 利用线性互补滤波算法，上一时刻姿态角，加速度与角速度更新当前姿态角
  void update(void);
  void filter(float k);
public:
  // IMU 原始数据
  ImuRawData_t raw_data_;
  uint8_t rx_acc_data[6];
  uint8_t rx_gyro_data[6];
  // IMU 换算得到的加速度与角速度
  // angular velocity(rad/s) 角速度
  float gyro_sensor_[3], gyro_world_[3];
  // angular velocity(dps) 角速度
  float gyro_sensor_dps_[3], gyro_world_dps_[3];
  // acceleration(m/s^2) 加速度
  float accel_sensor_[3], accel_world_[3];
  float roll,pitch,yaw;
  // 姿态解算得到的姿态角
  // 姿态角描述：euler angle 欧拉角
  EulerAngle_t euler_deg_, euler_rad_;
  // 姿态角描述：quaternion 四元数
  float q_[4] = {1, 0, 0, 0};

private:
  // 传感器变换矩阵(对应imu安装方向)
  float R_imu_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  // 陀螺仪零飘补偿项
  float gyro_bias_[3];
  float div_t = 0.001f; // 陀螺仪数据转换时间间隔(s)
  // Mahony解算
  Mahony mahony_;
};

#endif // IMU_H
