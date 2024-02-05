#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32f4xx_hal.h"
#include "can.h"

void CAN_Comm_Init(void);
void CANx_Transmit(CAN_HandleTypeDef *hcan, uint32_t tx_id, uint8_t* tx_data, uint8_t len);

#endif
