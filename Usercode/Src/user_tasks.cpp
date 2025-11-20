#include "../Inc/user_tasks.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "IMU.h"

// 外部数组先定义好
static const float R_imu[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static const float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// 全局 IMU 对象
IMU imu(0.001f, 0.2f, 0.1f, R_imu, gyro_bias);

/*[[noreturn]] void control_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        // 这里写你的控制逻辑
        osDelayUntil(tick + 1);
    }
} */

[[noreturn]] void imu_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        imu.readSensor();
        imu.update();
        osDelayUntil(tick + 1);
    }
}

// 任务属性
osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attributes = {
    .name = "imu_task",
    .stack_size = 512 * 4,
    .priority = osPriorityNormal,
};

/* osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attributes = {
    .name = "control_task",
    .stack_size = 128 * 4,
    .priority = osPriorityAboveNormal,
}; */

// 创建任务
void user_tasks_init() {
    //control_task_handle = osThreadNew(control_task, nullptr, &control_task_attributes);
    imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attributes);
}