#include "can.h"
#include "gpio.h"
#include "motor.h"
#include "main.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
extern uint8_t stop_flag;
extern float target_angle;
extern float target_speed;
uint32_t* pTxMailbox;

// Ensure external TIM handle is declared (used in HAL_TIM_PeriodElapsedCallback)

// forward-declare shared flags/buffer defined in user tasks
extern volatile uint8_t can_rx_ready;
extern uint8_t can_rx_buffer[8];
extern volatile uint8_t can_tx_ready;
extern volatile uint32_t can_rx_id;

// motor_stop is defined in user_tasks


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        // Read message into local buffer quickly
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        // Copy to shared buffer and set flag for task to process
        for (int i = 0; i < 8; ++i) {
            can_rx_buffer[i] = rx_data[i];
        }
        can_rx_id = rx_header.StdId;
        can_rx_ready = 1;
    }
}

