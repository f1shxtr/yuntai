// callback.cpp
#include "can.h"
#include "cmsis_os2.h"
#include "../../Usercode/Inc/user_tasks.h"

extern CAN_RxHeaderTypeDef rx_header;
extern uint8_t can_rx_buffer[8];
// 来自 user_tasks.cpp 的共享变量 (polling mode)
extern volatile uint8_t can_tx_ready;
extern volatile uint32_t can_rx_id;
extern volatile uint8_t can_rx_ready; // signal set by ISR for polling mode
extern volatile uint32_t can_rx_count; // debug counter, incremented when ISR reads a message

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
            ++can_rx_count;
        } else {
            // optional: could set an error flag or count
        }
    }
}
} // extern "C"
