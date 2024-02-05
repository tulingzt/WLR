#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"

#define JOINT_MOTOR_RESET_TORQUE 1.5f
#define JOINT_MOTOR_RESET_ERROR 0.005f

typedef struct
{
	uint8_t stop_cnt;
	uint8_t reset_flag;
	float last_position;
} motor_reset_t;

typedef enum
{
	CHASSIS_MODE_PROTECT,
	CHASSIS_MODE_FOLLOW,
	CHASSIS_MODE_ROTATE,
	CHASSIS_MODE_FIGHT,
	CHASSIS_MODE_UNFLLOW,
	CHASSIS_MODE_PRONE
} chassis_mode_e;

typedef __packed union
{
	uint8_t buff[5];
	__packed struct
	{
		__packed struct
		{
			uint8_t robot_mode	:3;
			uint8_t control_mode:1;
			uint8_t jump_mode	:1;
			uint8_t height_mode	:1;
			uint8_t shift_mode	:1;
			uint8_t vacancy		:1;
		} mode_msg;
		int16_t spd_vx;
		int16_t spd_vy;
	} data;
} chassis_input_t;

typedef enum
{
	CHASSIS_FORCE_CONTROL,
	CHASSIS_POSITION_CONTROL
} chassis_control_mode_e;

typedef struct
{
	chassis_mode_e mode, last_mode;
	chassis_control_mode_e control_mode;
	uint8_t joint_motor_reset;
} chassis_t;

extern chassis_t chassis;
extern chassis_input_t chassis_input;

void Chassis_Task(void const *argu);

#endif
