// File: `Usercode/Inc/user_tasks.h`
#ifndef IMU_USER_TASKS_H
#define IMU_USER_TASKS_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

    // CAN 消息结构（C 兼容）
    typedef struct {
        uint32_t id;
        uint8_t data[8];
        uint8_t dlc;
    } CanMessage;

    // 在 ISR/其他 C 代码中也能引用的外部对象
    extern QueueHandle_t can_rx_queue;
    extern QueueHandle_t can_tx_queue;
    extern uint32_t* pTxMailbox;

    // 对外初始化和停止接口（保持 C 链接，便于 C 代码调用）
    void user_tasks_init(void);
    void motor_stop(void);

    // 其它 OS 原语（如需要）保留
    extern osEventFlagsId_t event_handle;

#ifdef __cplusplus
}
#endif

#endif // IMU_USER_TASKS_H
