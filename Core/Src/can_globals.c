/* CAN global symbols shared between C modules */
#include "can.h"
#include <stdint.h>

CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header;

uint8_t rx_data[8] = {0};
uint8_t tx_data[8] = {0};

// mailbox index used by HAL_CAN_AddTxMessage (store as uint32_t)
uint32_t can_tx_mail_box_ = 0;

