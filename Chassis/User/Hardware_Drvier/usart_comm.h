#ifndef __USART_COMM_H
#define __USART_COMM_H

#include "stm32f4xx_hal.h"
#include "DT7control_driver.h"
#include "usart.h"

#include "string.h"

//串口定义
#define	DBUS_HUART	huart1

void USART_Comm_Init(void);
void USART_User_IRQHandler(UART_HandleTypeDef *huart);

#endif
