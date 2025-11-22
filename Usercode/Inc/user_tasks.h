//
// Created by Fish on 2025/11/1.
//

#ifndef IMU_USER_TASKS_H
#define IMU_USER_TASKS_H

#include <stdint.h>
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C"{
#endif

    void user_tasks_init(void);
    void motor_stop(void);

    // OS primitives used by user tasks (event flags and semaphores)
    extern osSemaphoreId_t CAN_semaphore_handle;
    extern osSemaphoreId_t Rcc_semaphore_handle;
    extern osEventFlagsId_t event_handle;

#ifdef  __cplusplus
}
#endif
#endif // IMU_USER_TASKS_H
