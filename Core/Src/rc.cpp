//
// Created by Fish on 2025/10/25.
//

#include "rc.h"
#include "../../Usercode/Inc/user_tasks.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart3;
RC_Class RC;

RC_Class::RC_Class() : last_receive_tick(0), data_ready(0)
{
    rc.ch0 = 0;
    rc.ch2 = 0;
    rc.ch3 = 0;
    rc.ch1 = 0;
    rc.s1 = MID;
    rc.s2 = MID;
}

// ----------------- 初始化 DMA 接收 -----------------
void RC_Class::init()
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18);

}

// ----------------- 主循环处理 -----------------
void RC_Class::handle()
{

    // 更新连接状态
    is_connected = (HAL_GetTick() - last_receive_tick <= RC_TIMEOUT_MS) ? 1 : 0;

    if(data_ready)
    {
        parse();
        data_ready = 0;
    }
}

void RC_Class::parse() {
    if(pData == NULL)
    {
        return;
    }

    int16_t ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    int16_t ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))
   & 0x07FF;
    int16_t ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
    ((int16_t)pData[4] << 10)) & 0x07FF;
    int16_t ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &
   0x07FF;
    rc.ch0 = ((float)ch0 - 1024.0f) / 660.0f;
    rc.ch1 = ((float)ch1 - 1024.0f) / 660.0f;
    rc.ch2 = ((float)ch2 - 1024.0f) / 660.0f;
    rc.ch3 = ((float)ch3 - 1024.0f) / 660.0f;
    uint16_t s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    uint16_t s2 = ((pData[5] >> 4) & 0x0003);
    switch(s1)
    {
        case 1: rc.s1 = UP; break;
        case 2: rc.s1 = DOWN; break;
        default: rc.s1 = MID; break;
    }

    switch(s2)
    {
        case 1: rc.s2 = UP; break;
        case 2: rc.s2 = DOWN; motor_stop();
        default: rc.s2 = MID; break;
    }
}

