#include "chassis_task.h"
#include "wlr.h"
#include "leg_vmc.h"
#include "DT7control_driver.h"
#include "HTmotor_driver.h"
#include "DJImotor_driver.h"
#include "Timu_driver.h"
#include "math_calcu.h"
#include "pid.h"
#include "cmsis_os.h"
#include "math.h"

motor_reset_t motor_reset[4];
chassis_t chassis;
pid_t pid_yaw;

//电机卡限位进行复位校准
static void Joint_Motor_Reset(void)
{
	uint32_t thread_wake_time = osKernelSysTick();
	while(rc.sw1 == RC_MI || rc.sw1 == 0)
	{
		osDelayUntil(&thread_wake_time, 2);
	}
	driver_motor[0].t = 0;
	driver_motor[1].t = 0;
	//标志位清零
	chassis.joint_motor_reset = 0;
	for(int i = 0; i < 4; i++)
		motor_reset[i].reset_flag = 0;

	while(!chassis.joint_motor_reset)
	{
		if(rc.sw1 == RC_MI || rc.sw1 == 0)//未解锁
		{
			for(int i = 0; i < 4; i++)
			{
				joint_motor[i].t = 0;
				HTmotor_SendControlPara(&joint_motor[i]);
			}
			DJImotor_SendControlTorque();
		}
		else//开始复位
		{
			if(motor_reset[0].reset_flag == 1 && motor_reset[1].reset_flag == 1 && \
				motor_reset[2].reset_flag == 1 && motor_reset[3].reset_flag == 1)
				chassis.joint_motor_reset = 1;
			else
				chassis.joint_motor_reset = 0;
			for(int i = 0; i < 4; i++)
			{
				if(motor_reset[i].reset_flag == 0)
				{
					if(i == 0 || i == 2)
						joint_motor[i].t = -JOINT_MOTOR_RESET_TORQUE;
					else
						joint_motor[i].t = JOINT_MOTOR_RESET_TORQUE;
					if(fabs(motor_reset[i].last_position - joint_motor[i].position) < JOINT_MOTOR_RESET_ERROR)
						motor_reset[i].stop_cnt++;
					else
						motor_reset[i].stop_cnt = 0;
					if(motor_reset[i].stop_cnt >= 100)
					{
						joint_motor[i].t = 0;
						motor_reset[i].reset_flag = 1;
						HTmotor_ControlCmd(&joint_motor[i], CMD_ZERO_POSITION);
					}
					else
					{
						HTmotor_SendControlPara(&joint_motor[i]);
						motor_reset[i].last_position = joint_motor[i].position;
					}
				}
				else
				{
					joint_motor[i].t = 0;
					HTmotor_SendControlPara(&joint_motor[i]);
				}
			}
			DJImotor_SendControlTorque();
		}
		osDelayUntil(&thread_wake_time, 2);
	}
}

//底盘参数初始化，电机复位
static void Chassis_Init(void)
{
	joint_motor[0].zero_point = 0.64;
	joint_motor[1].zero_point = 0.78;
	joint_motor[2].zero_point = 0.77;
	joint_motor[3].zero_point = 0.65;
//	PID_Init(&pid_yaw, 0, 0, 0, 0, 1);
	WLR_Init();
	Joint_Motor_Reset();
}

//底盘数据输入更新
static void Chassis_Data_Input(void)
{
	taskENTER_CRITICAL();
	//模式输入
	if(rc.sw1 == RC_UP)			//力控
		wlr.control_mode = 1;
	else if(rc.sw1 == RC_DN)	//位控
		wlr.control_mode = 2;
	else						//保护模式
		wlr.control_mode = 0;

	if(rc.sw2 == RC_UP)			//加速
		wlr.shift_mode = 1;
	else
		wlr.shift_mode = 0;
	
	if(rc.sw2 == RC_DN)			//刹车
		wlr.stop_mode = 1;
	else
		wlr.stop_mode = 0;

	if(rc.sw2 == RC_UP)			//加速
		wlr.high_mode = 1;//大高度
	else
		wlr.high_mode = 0;
//	if()
//		wlr.high_mode = 0;		//小高度
//	else
//		wlr.high_mode = 1;		//大高度
	
	if(rc.ch5 == 660)			//跳跃
		wlr.jump_flag = 1;
	//控制数据输入
	wlr.wz_set = -(float)rc.ch1/660*4;
	wlr.v_set = (float)rc.ch2/660*4;
	//陀螺仪数据输入
//	if(wlr.control_mode == 0)
//	{
//		wlr.yaw_set = imu.yaw;
//		wlr.wz_set = 0;
//	}
//	else
//	{
//		wlr.yaw_set -= (float)rc.ch1/660/100;
//		wlr.wz_set = PID_Calc(&pid_yaw, wlr.yaw_set, imu.yaw);
//	}
	wlr.roll_fdb	=  -imu.roll;
	wlr.pit_fdb		=  imu.pitch;
	wlr.wy_fdb		=  imu.wy;
	wlr.wz_fdb		=  imu.wz;
	wlr.az_fdb		=  imu.az;
	//电机数据输入
	wlr.side[0].q4 =  joint_motor[0].position - joint_motor[0].zero_point;
	wlr.side[0].q1 =  joint_motor[1].position + joint_motor[1].zero_point + PI;
	wlr.side[0].w4 =  joint_motor[0].velocity;
	wlr.side[0].w1 =  joint_motor[1].velocity;
	wlr.side[0].t4 =  joint_motor[0].torque;
	wlr.side[0].t1 =  joint_motor[1].torque;
	wlr.side[0].wy =  -driver_motor[0].velocity;//可能需要加滤波

	wlr.side[1].q1 = -joint_motor[2].position + joint_motor[2].zero_point + PI;
	wlr.side[1].q4 = -joint_motor[3].position - joint_motor[3].zero_point;
	wlr.side[1].w1 = -joint_motor[2].velocity;
	wlr.side[1].w4 = -joint_motor[3].velocity;
	wlr.side[1].t1 = -joint_motor[2].torque;
	wlr.side[1].t4 = -joint_motor[3].torque;
	wlr.side[1].wy =  driver_motor[1].velocity;//可能需要加滤波
	taskEXIT_CRITICAL();
}

//底盘数据输出，控制电机
static void Chassis_Data_Output(void)
{
	if(wlr.control_mode == 1)		//力控
	{
		for(int i = 0; i < 4; i++)
		{
			joint_motor[i].kp = 0;
			joint_motor[i].kd = 0.03;
		}
		joint_motor[0].t =  wlr.side[0].T4;
		joint_motor[1].t =  wlr.side[0].T1;
		joint_motor[2].t = -wlr.side[1].T1;
		joint_motor[3].t = -wlr.side[1].T4;
		driver_motor[0].t = -wlr.side[0].Tw;
		driver_motor[1].t = wlr.side[1].Tw;
		
	}
	else if(wlr.control_mode == 2)	//位控
	{
		for(int i = 0; i < 4; i++)
		{
			joint_motor[i].kp = 10;
			joint_motor[i].kd = 2;
			joint_motor[i].t = 0;
		}
		joint_motor[0].p =  wlr.side[0].P4 + joint_motor[0].zero_point;
		joint_motor[1].p =  wlr.side[0].P1 - joint_motor[1].zero_point - PI;
		joint_motor[2].p = -wlr.side[1].P1 + joint_motor[2].zero_point + PI;
		joint_motor[3].p = -wlr.side[1].P4 - joint_motor[3].zero_point;
		driver_motor[0].t = -wlr.side[0].Tw;
		driver_motor[1].t =  wlr.side[1].Tw;
	}
	else							//保护模式
	{
		WLR_Protest();
		for(int i = 0; i < 4; i++)
		{
			joint_motor[i].kp = 0;
			joint_motor[i].kd = 0;
			joint_motor[i].t = 0;
		}
		driver_motor[0].t = 0;
		driver_motor[1].t = 0;
	}
	//输出
	for(int i = 0; i < 4; i++)
	{
		HTmotor_SendControlPara(&joint_motor[i]);
	}
	DJImotor_SendControlTorque();
}

void Chassis_Task(void const *argu)
{
	Chassis_Init();
	uint32_t thread_wake_time = osKernelSysTick();
	for(;;)
	{
		if(rc.sw1 == RC_MI && rc.sw2 == RC_DN)
			Chassis_Init();
		Chassis_Data_Input();
		WLR_Control();
		Chassis_Data_Output();
		osDelayUntil(&thread_wake_time, 2);
	}
}
