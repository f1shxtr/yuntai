#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "bmi088.h"

void bmi088_write_byte(uint8_t tx_data) {
        HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length) {
        HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

void bmi088_write_reg(uint8_t reg, uint8_t data) {
        bmi088_write_byte(reg & 0x7F);
        bmi088_write_byte(data);
}

void BMI088_ACCEL_NS_L() {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); // 拉低
}

void BMI088_ACCEL_NS_H() {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);   // 拉高
}


void BMI088_GYRO_NS_L() {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);   // 拉低
}

void BMI088_GYRO_NS_H() {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);     // 拉高
}

// 参考: acc写入，相当于加上片选的 bmi088_write_reg 函数
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
        BMI088_GYRO_NS_H();
        BMI088_ACCEL_NS_L();

        bmi088_write_byte(reg & 0x7F);
        bmi088_write_byte(data);

        BMI088_ACCEL_NS_H();
}

// 尝试完成 ↓
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
        BMI088_GYRO_NS_H();
        BMI088_ACCEL_NS_L();

        bmi088_write_byte(reg | 0x80);

        uint8_t dummy = 0x00;
        bmi088_read_byte(&dummy, 1);
        bmi088_read_byte(rx_data, length);

        BMI088_ACCEL_NS_H();       // 取消片选
}// 加速度计读取，注意需要忽略第一位数据dummy byte
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_L();

        bmi088_write_byte(reg | 0x80) ;
        bmi088_read_byte(rx_data, length);

        BMI088_GYRO_NS_H();
}// 陀螺仪读取
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_L();

        bmi088_write_byte(reg & 0x7F);
        bmi088_write_byte(tx_data);

        BMI088_GYRO_NS_H();
}// gyro写入

void bmi088_init() {
        // Soft Reset ACCEL
        BMI088_ACCEL_NS_L();
        bmi088_write_reg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
        HAL_Delay(1);
        BMI088_ACCEL_NS_H();

        // Soft Reset GYRO
        BMI088_GYRO_NS_L();
        bmi088_write_reg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
        HAL_Delay(30);
        BMI088_GYRO_NS_H();

        // Switch ACCEL to Normal Mode
        BMI088_ACCEL_NS_L();
        HAL_Delay(1);
        bmi088_write_reg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
        HAL_Delay(1);
        BMI088_ACCEL_NS_H();
}

