#ifndef __HTMOTOR_DRIVER_H
#define __HTMOTOR_DRIVER_H

#include "stm32f4xx_hal.h"
#include "can.h"

#define HT_MOTOR_ID			0x00

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f	// N-m
#define T_MAX 18.0f

typedef struct
{
	//电机参数
	uint32_t id;
	CAN_HandleTypeDef* hcan;
	uint32_t send_cnt, receive_cnt;
	float err_percent;
	//安装角度补偿
	float zero_point;
	//控制数据
	float p, v, kp, kd, t;
	//反馈数据
	float position, velocity, torque;	//rad rad/s N*m
} HTmotor_t;

extern HTmotor_t joint_motor[4];

void HTmotor_Init(HTmotor_t* motor, CAN_HandleTypeDef* hcan, uint32_t id, float zero_point);
void HTmotor_SendControlPara(HTmotor_t* motor);
void HTmotor_ControlCmd(HTmotor_t* motor, uint8_t cmd);
void HTmotor_Receive(HTmotor_t* motor, uint8_t* rx_data);

#endif
