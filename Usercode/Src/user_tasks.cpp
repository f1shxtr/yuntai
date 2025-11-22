#include "../Inc/user_tasks.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "IMU.h"
#include "motor.h"
#include "can.h"

// extern HAL objects used for CAN send
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint32_t* pTxMailbox;
extern float target_angle; // ensure referenced symbol is declared

// 外部数组先定义好
static const float R_imu[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static const float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// 全局 IMU 对象
IMU imu(0.001f, 0.2f, 0.1f, R_imu, gyro_bias);

// CAN/Task synchronization primitives (simple flags+buffer)
volatile uint8_t can_rx_ready = 0;
volatile uint8_t can_rx_buffer[8] = {0};
volatile uint8_t can_tx_ready = 0;
volatile uint32_t can_rx_id = 0;

// create two motor instances as requested
Motor motor_pitch(1.0f, 0x208, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4000.0f, 4000.0f, 16384.0f, 16384.0f, 0.1f, 0.0f);
Motor motor_yaw(1.0f, 0x205, 0.0f, 0.0f, 0.0f, 210.0f, 0.0f, 0.0f, 4000.0f, 4000.0f, 16384.0f, 16384.0f, 0.1f, 0.0f);


// Task attributes
osThreadId_t can_rx_task_handle;
constexpr osThreadAttr_t can_rx_task_attributes = {
    .name = "can_rx_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

osThreadId_t can_tx_task_handle;
constexpr osThreadAttr_t can_tx_task_attributes = {
    .name = "can_tx_task",
    .stack_size = 128 * 4,
    .priority = osPriorityLow,
};

osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attributes = {
    .name = "imu_task",
    .stack_size = 512 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void can_rx_task(void* ) {

    while (true) {
        if (can_rx_ready) {
            // copy buffer locally to avoid ISR race
            uint8_t buf[8];
            for (int i = 0; i < 8; ++i) buf[i] = can_rx_buffer[i];
            uint32_t id = can_rx_id;
            can_rx_ready = 0;

            // dispatch to proper motor based on ID
            if (id == 0x208) {
                motor_pitch.can_rx_msg_callback(buf);
                motor_pitch.handle();
            } else if (id == 0x205) {
                motor_yaw.can_rx_msg_callback(buf);
                motor_yaw.handle();
            } else {
                // unknown id: ignore or log
            }

            // mark tx ready to be sent by tx task
            if (can_tx_ready == 0) {
                can_tx_ready = 1;
            }
        }

    }
}

[[noreturn]] void can_tx_task(void* ){

    while (true) {
            // send tx_data via HAL CAN (use existing tx_header and pTxMailbox from Core)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, pTxMailbox);
        }

}


[[noreturn]] void imu_task(void* ) {
    TickType_t tick = osKernelGetTickCount();
    while (true) {
        imu.readSensor();
        imu.update();
        tick += 1;
        osDelayUntil(tick);
    }
}

// 创建任务: ensure osThreadNew uses function pointers
void user_tasks_init() {
    can_rx_task_handle = osThreadNew(can_rx_task, NULL, &can_rx_task_attributes);
    can_tx_task_handle = osThreadNew(can_tx_task, NULL, &can_tx_task_attributes);
    imu_task_handle = osThreadNew(imu_task, NULL, &imu_task_attributes);
}

void motor_stop(void) {
    // stop both motors
    motor_pitch.SetIntensity(0.0f);
    motor_yaw.SetIntensity(0.0f);
    // ensure tx_data cleared for immediate send
    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
}

