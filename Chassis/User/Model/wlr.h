#ifndef __WHEEL_LEG_ROBOT_H
#define __WHEEL_LEG_ROBOT_H

#include "stdint.h"

typedef struct
{
	float X_ref[6 * 1];
	float X_fdb[6 * 1];
	float X_diff[6 * 1];
	float U_ref[2 * 1];
	float K[2 * 6];
	//x2为速度 用来积分计算x1位移
	//x4为腿摆角角速度 用来差分计算dot_x4腿摆角加速度
	float last_x2, last_x4;
	float dot_x4;
} lqr_t;

//全身运动控制
typedef struct
{
	//目标数据
	float v_set, wz_set, high_set, roll_set, q0_set;
	float yaw_set;
	//反馈数据
	float roll_fdb, pit_fdb, wy_fdb, wz_fdb, az_fdb;
	//机械误差补偿
	float q0_offs, roll_offs, wz_offs;
	//限幅控制
	float max_speed_error, max_stop_angle, max_wz_error;
	//控制标志
	uint8_t jump_flag, jump_cnt, high_mode, shift_mode, stop_mode, control_mode;
	//单侧控制参数
	struct
	{
		//接收数据
		float q1, q4;
		float w1, w4;
		float t1, t4;
		float wy;
		//发送数据
		float T1, T4, Tw;
		float P1, P4;
		//中间数据
		uint8_t fly_flag;
		float q0_kal, w0_kal, Fn_kal;
		float Fn_fdb;
		float T0, Fy;
	} side[2];
} wlr_t;

extern wlr_t wlr;
extern lqr_t lqr[2];

void WLR_Init(void);
void WLR_Protest(void);
void WLR_Control(void);
void K_Array_Update(float K[2][6], float high_set);

#endif
