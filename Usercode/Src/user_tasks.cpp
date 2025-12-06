// user_tasks.cpp
#include "../Inc/user_tasks.h"
#include "IMU.h"
#include "motor.h"
#include "can.h"
#include "FreeRTOS.h"
#include "rc.h"
// ========= 全局（共享给 callback.cpp） =========
volatile uint32_t can_rx_id = 0;
volatile uint8_t can_rx_ready = 0; // ISR sets this when new message available
volatile uint8_t can_rx_buffer[8] = {0};
volatile uint8_t can_tx_ready = 0; // set by RX task to request TX
 // incremented by ISR when a message is received

uint8_t tx_data[8] = {0};
uint8_t rx_data[8] = {0};
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
RC_Class rcc;
// ========= Motor 实例 =========
Motor motor_pitch(1.0f, 0x208, 0,0,0, 0,0,0, 4000,4000, 16384,16384, 0.1f,0.0f);
Motor motor_yaw  (1.0f, 0x205, 0,0,0,210,0,0,4000,4000,16384,16384,0.1f,0.0f);
float motor_pitch_targetspeed = 0.0f;
float motor_yaw_targetspeed = 0.0f;
float motor_pitch_targetangle = 0.0f;
float motor_yaw_targetangle = 0.0f;

// =====================================================================================
// CAN RX Task —— 等待 CAN ISR 事件 → 调 motor → 触发 TX
// =====================================================================================
[[noreturn]] void can_rx_task(void*)
{
    while (true)
    {
        // polling: wait until ISR sets can_rx_ready

        // copy data locally then clear flag
        uint8_t data[8];
        for (int i = 0; i < 8; ++i)
            data[i] = can_rx_buffer[i];
        uint32_t id = can_rx_id;// clear for next message

        rcc.is_connected = (HAL_GetTick() - rcc.last_receive_tick <= 1000) ? 1 : 0;
        // data_ready 可用于触发调试观察
        if(rcc.data_ready)
        {
            rcc.handle();
            rcc.data_ready = 0;
        }

        motor_yaw_targetangle = rcc.rc.ch0 * 180.0f;

        if (id == 0x208) {
            motor_pitch.can_rx_msg_callback(data);
            motor_pitch.SetSpeed(motor_pitch_targetspeed, 0.0f);
            motor_pitch.handle();
        }
        else if (id == 0x205) {
            motor_yaw.can_rx_msg_callback(data);
            motor_yaw.SetSpeed(motor_yaw_targetspeed, 0.0f);
            motor_yaw.handle();
        }
        // signal tx task by setting can_tx_ready
    //    can_tx_ready = 1;
    }
}


extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;

//Motor motor_pitch(1,0x208,8,0.01,600,49,0,0,4000,4000,16384,16384,0.06,0.0);
//Motor motor_yaw(1,0x205,20,0.001,600,210,0,0,4000,4000,16384,16384,0.07,0.0);

constexpr auto flag_1 = 1u << 0;
constexpr auto flag_2 = 1u << 1;
constexpr auto flag_3 = 1u << 2;
constexpr auto flag_4 = 1u << 3;
constexpr auto flag_5 = 1u << 4;

osSemaphoreAttr_t CAN_semaphore_attributes = {.name = "CAN_semaphore"};
osSemaphoreId_t CAN_semaphore_handle;
osSemaphoreAttr_t Rcc_semaphore_attributes = {.name = "Rcc_semaphore"};
osSemaphoreId_t Rcc_semaphore_handle;
osEventFlagsAttr_t event_attributes = {.name = "event"};
osEventFlagsId_t event_handle;

osThreadId_t motor_feedback_decoding_task_handle;
constexpr osThreadAttr_t motor_feedback_decoding_task_attributes = {
    .name = "motor_feedback_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityHigh1),
};

osThreadId_t PID_control_task_handle;
constexpr osThreadAttr_t PID_control_task_attributes = {
    .name = "PID_control_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityLow),
};

osThreadId_t CAN_emmit_task_handle;
constexpr osThreadAttr_t CAN_emmit_task_attributes = {
    .name = "CAN_emmit_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityLow),
};

osThreadId_t IMU_decoding_task_handle;
constexpr osThreadAttr_t IMU_decoding_task_attributes = {
    .name = "IMU_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityNormal),
};

osThreadId_t Rcc_decoding_task_handle;
constexpr osThreadAttr_t Rcc_decoding_task_attributes = {
    .name = "Rcc_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityHigh),
};

osThreadId_t Stop_motor_task_handle;
constexpr osThreadAttr_t Stop_motor_task_attributes = {
    .name = "Stop_motor_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityHigh),
};

[[noreturn]] void motor_feedback_decoding_task(void*) {
    while (1)
    {
        osSemaphoreAcquire(CAN_semaphore_handle, osWaitForever);
        if (rx_header.StdId == 0x208)
        {
            motor_pitch.can_rx_msg_callback(rx_data);
        }
        else if (rx_header.StdId == 0x205)
        {
            motor_yaw.can_rx_msg_callback(rx_data);
        }
        osEventFlagsSet(event_handle, flag_4);
    }
}

[[noreturn]] void PID_control_task(void*) {
    while (1)
    {
        osEventFlagsWait(event_handle, flag_4,osFlagsWaitAny, osWaitForever);//后续改为flag_4|flag_3
        //float feedforward_intensity =motor_pitch.FeedforwardIntensityCalc();
        //motor_pitch.SetPosition(target_angle_pitch,0.0f,feedforward_intensity);
        motor_pitch.SetSpeed(motor_pitch_targetspeed,0);
        motor_pitch.handle();

        //motor_yaw.SetPosition(target_angle_yaw,0.0f,0.0f);
        motor_yaw.SetSpeed(motor_yaw_targetspeed,0);
        motor_yaw.handle();
        osEventFlagsClear(event_handle, flag_3);
        osEventFlagsSet(event_handle, flag_5);
        osEventFlagsClear(event_handle, flag_4);
    }
}

[[noreturn]] void CAN_emmit_task(void*) {
    while (1)
    {
        osEventFlagsWait(event_handle, flag_5, osFlagsWaitAll, osWaitForever); //后续加入flag_5|flag_2
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, pTxMailbox);
        osEventFlagsClear(event_handle, flag_5);
    }
}

[[noreturn]] void IMU_decoding_task(void*) {
    while (1)
    {
        imu.readSensor();
        imu.update();
        osDelay(1);
    }
}

[[noreturn]] void Rcc_decoding_task(void*) {
    while (1)
    {
        osSemaphoreAcquire(Rcc_semaphore_handle, osWaitForever);
        rcc.handle();
        if (rcc.rc.s1 == 1)
        {
            osEventFlagsSet(event_handle, flag_2);
        }
        else if (rcc.rc.s1 == 2)
        {
            osEventFlagsSet(event_handle, flag_1);
            osEventFlagsClear(event_handle, flag_2);
        }
        motor_pitch_targetangle = rcc.rc.ch1 * 25 + 30;
        motor_yaw_targetangle = rcc.rc.ch0 * 90;
        osEventFlagsSet(event_handle, flag_3);
    }
}//暂不用

/*[[noreturn]] void Stop_motor_task(void*) {
    while (1)
    {
        osEventFlagsWait(event_handle, flag_1, osFlagsWaitAll, osWaitForever);
        uint8_t stop_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, stop_data, &can_tx_mail_box_);
        osEventFlagsClear(event_handle, flag_1);
    }
}*/

extern "C" void user_tasks_init() {
    CAN_semaphore_handle = osSemaphoreNew(1, 1, &CAN_semaphore_attributes);
    Rcc_semaphore_handle = osSemaphoreNew(1, 1, &Rcc_semaphore_attributes);
    event_handle = osEventFlagsNew(&event_attributes);
    motor_feedback_decoding_task_handle = osThreadNew(motor_feedback_decoding_task, NULL, &motor_feedback_decoding_task_attributes);
    PID_control_task_handle = osThreadNew(PID_control_task, NULL, &PID_control_task_attributes);
    CAN_emmit_task_handle = osThreadNew(CAN_emmit_task, NULL, &CAN_emmit_task_attributes);
    IMU_decoding_task_handle = osThreadNew(IMU_decoding_task, NULL, &IMU_decoding_task_attributes);
    Rcc_decoding_task_handle = osThreadNew(Rcc_decoding_task, NULL, &Rcc_decoding_task_attributes);
    //Stop_motor_task_handle = osThreadNew(Stop_motor_task, NULL, &Stop_motor_task_attributes);
}

// 保留一个简单的电机停止函数（header 中声明）
extern "C" void motor_stop(void) {
    motor_pitch.SetIntensity(0.0f);
    motor_yaw.SetIntensity(0.0f);
    for (int i = 0; i < 8; ++i) tx_data[i] = 0;
}