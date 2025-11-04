//
// Created by Fish on 2025/11/1.
//

#include "../Inc/user_tasks.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

uint32_t  count = 0;

[[noreturn]] void test_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        ++count;
        osDelayUntil(tick + 1);
    }
}

osThreadId_t test_task_handle;
constexpr osThreadAttr_t test_task_attributes = {
    .name = "test_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};
void user_tasks_init() {
    test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes);
}
