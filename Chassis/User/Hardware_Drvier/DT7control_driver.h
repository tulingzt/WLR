#ifndef __DT7CONTROL_DRIVER_H
#define __DT7CONTROL_DRIVER_H

#include "stm32f4xx_hal.h"

#define DT7_DATA_LEN 18

#define RC_UP 1
#define RC_MI 3
#define RC_DN 2

typedef __packed struct
{
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	uint8_t sw1;
	uint8_t sw2;
	__packed struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;
	} mouse;
	__packed union
	{
		uint16_t key_code;
		__packed struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;//16个键位
		} bit;
	} kb;
} rc_t;

extern rc_t rc;

uint8_t DT7control_Receive(rc_t *rc, uint8_t *buff);

#endif
