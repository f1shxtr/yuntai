// callback.cpp
#include "can.h"
#include "cmsis_os2.h"
#include "rc.h"
#include "usart.h"
#include <cstring>
#include "../../Usercode/Inc/user_tasks.h"

extern CAN_RxHeaderTypeDef rx_header;
extern uint8_t can_rx_buffer[8];
// 来自 user_tasks.cpp 的共享变量 (polling mode)
extern volatile uint8_t can_tx_ready;
extern volatile uint32_t can_rx_id;
extern volatile uint8_t can_rx_ready; // signal set by ISR for polling mode
extern volatile uint32_t can_rx_count; // debug counter, incremented when ISR reads a message
extern RC_Class rcc;
void init() {
    rcc.init();
}
extern UART_HandleTypeDef huart3;
extern "C" {
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        // Read message into local buffer quickly using provided handle
        HAL_StatusTypeDef st = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_buffer);
        if (st == HAL_OK) {
            // set id and polling-ready flag (task will copy/clear later)
            can_rx_id = rx_header.StdId;
            can_rx_ready = 1;
            // increment debug counter
        }
    }
}
} // extern "C"
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance != USART3) return;

    // 拷贝数据到 last_buf
    memcpy(rcc.pData, rcc.rx_buf, Size);
    rcc.data_ready = 1;

    // 更新接收时间戳
    rcc.last_receive_tick = HAL_GetTick();

    // 继续接收下一帧
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rcc.rx_buf, 18);
}