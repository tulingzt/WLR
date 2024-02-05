#ifndef __TIMU_DRIVER_H
#define __TIMU_DRIVER_H

#include "stm32f4xx_hal.h"
#include "can.h"

#define IMU_PALSTANCE_ID	0X013
#define IMU_ANGLE_ID		0x014
#define IMU_ELSE_ID			0x015

typedef struct
{
	//反馈数据
	float pitch, yaw, roll;	//rad
	float wy, wz;			//rad/s
	float az;				//m/(s^2)
} Timu_t;

extern Timu_t imu;

void Timu_Receive(Timu_t* imu, uint32_t id, uint8_t* rx_data);

#endif
