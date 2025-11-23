// user_tasks.cpp
#include "../Inc/user_tasks.h"
#include "IMU.h"
#include "motor.h"
#include "can.h"
#include "FreeRTOS.h"

// ========= 全局（共享给 callback.cpp） =========
volatile uint32_t can_rx_id = 0;
volatile uint8_t can_rx_ready = 0; // ISR sets this when new message available
volatile uint8_t can_rx_buffer[8] = {0};
volatile uint8_t can_tx_ready = 0; // set by RX task to request TX
volatile uint32_t can_rx_count = 0; // incremented by ISR when a message is received

uint8_t tx_data[8] = {0};
extern CAN_TxHeaderTypeDef tx_header;
extern CAN_HandleTypeDef hcan1;
uint32_t* pTxMailbox;

// ========= IMU 实例 =========
static const float R_imu[3][3] = {
    {1,0,0},
    {0,1,0},
    {0,0,1}
};
static const float gyro_bias[3] = {0,0,0};

IMU imu(0.001f, 0.2f, 0.1f, R_imu, gyro_bias);

// ========= Motor 实例 =========
Motor motor_pitch(1.0f, 0x208, 0,0,0, 0,0,0, 4000,4000, 16384,16384, 0.1f,0.0f);
Motor motor_yaw  (1.0f, 0x205, 0,0,0,210,0,0,4000,4000,16384,16384,0.1f,0.0f);


// =====================================================================================
// CAN RX Task —— 等待 CAN ISR 事件 → 调 motor → 触发 TX
// =====================================================================================
[[noreturn]] void can_rx_task(void*)
{
    while (true)
    {
        // polling: wait until ISR sets can_rx_ready
        while (!can_rx_ready) {
            osDelay(1);
        }

        // copy data locally then clear flag
        uint8_t data[8];
        for (int i = 0; i < 8; ++i)
            data[i] = can_rx_buffer[i];
        uint32_t id = can_rx_id;
        can_rx_ready = 0; // clear for next message

        if (id == 0x208) {
            motor_pitch.can_rx_msg_callback(data);
            motor_pitch.handle();
        }
        else if (id == 0x205) {
            motor_yaw.can_rx_msg_callback(data);
            motor_yaw.handle();
        }

        // signal tx task by setting can_tx_ready
        can_tx_ready = 1;
    }
}


// =====================================================================================
// CAN TX Task —— 等待发送事件 → 发送 CMD
// =====================================================================================
[[noreturn]] void can_tx_task(void*)
{
    while (true)
    {
        // polling: wait for can_tx_ready
        while (!can_tx_ready) {
            osDelay(1);
        }

        uint8_t local[8];
        for (int i = 0; i < 8; ++i)
            local[i] = tx_data[i];

        can_tx_ready = 0;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, local, pTxMailbox);
    }
}


// =====================================================================================
// IMU Task —— 固定 1ms 更新
// =====================================================================================
[[noreturn]] void imu_task(void*)
{
    TickType_t tick = osKernelGetTickCount();

    while (true)
    {
        imu.readSensor();
        imu.update();
        tick += 1;
        osDelayUntil(tick);
    }
}


// =====================================================================================
// 任务初始化
// =====================================================================================
void user_tasks_init()
{
     // RX 任务
     osThreadAttr_t rx_attr = {
         .name = "can_rx",
         .stack_size = 512,
         .priority = osPriorityNormal
     };
     osThreadNew(can_rx_task, NULL, &rx_attr);

    // TX 任务
    osThreadAttr_t tx_attr = {
        .name = "can_tx",
        .stack_size = 512,
        .priority = osPriorityLow
    };
    osThreadNew(can_tx_task, NULL, &tx_attr);

    // IMU 任务
    osThreadAttr_t imu_attr = {
        .name = "imu",
        .stack_size = 1024,
        .priority = osPriorityNormal
    };
    osThreadNew(imu_task, NULL, &imu_attr);
}


// =====================================================================================
// Motor Stop
// =====================================================================================
void motor_stop()
{
    motor_pitch.SetIntensity(0.0f);
    motor_yaw.SetIntensity(0.0f);

    for (int i = 0; i < 8; ++i)
        tx_data[i] = 0;
}