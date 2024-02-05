#include "DJImotor_driver.h"
#include "can_comm.h"
#include "string.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef PI
#define PI 3.14159265358979323846f
#endif

DJImotor_t driver_motor[2];

//大疆电机发送数据转化单位
static void DJImotor_ConvertData(DJImotor_t* motor)
{
	motor->send_cnt++;
	switch(motor->motor_type)
	{
		case DJI_2006_MOTOR:
			motor->tx_current = (int16_t)(motor->t / DJI_2006_REDUCTION_RATIO * DJI_2006_ORIGINAL_REDUCTION_RATIO \
								/ DJI_2006_MOTOR_TORQUE_CONSTANT \
								/ DJI_2006_MOTOR_CURRENT_RANGE * DJI_2006_MOTOR_DATA_RANGE);
			LIMIT_MIN_MAX(motor->tx_current, -DJI_2006_MOTOR_DATA_RANGE, DJI_2006_MOTOR_DATA_RANGE);
			break;
		case DJI_3508_MOTOR:
			motor->tx_current = (int16_t)(motor->t / DJI_3508_REDUCTION_RATIO * DJI_3508_ORIGINAL_REDUCTION_RATIO \
								/ DJI_3508_MOTOR_TORQUE_CONSTANT \
								/ DJI_3508_MOTOR_CURRENT_RANGE * DJI_3508_MOTOR_DATA_RANGE);
			LIMIT_MIN_MAX(motor->tx_current, -DJI_3508_MOTOR_DATA_RANGE, DJI_3508_MOTOR_DATA_RANGE);
			break;
		case DJI_6020_MOTOR:
			motor->tx_current = (int16_t)(motor->t \
								/ DJI_6020_MOTOR_TORQUE_CONSTANT \
								/ DJI_6020_MOTOR_CURRENT_RANGE * DJI_6020_MOTOR_DATA_RANGE);
			LIMIT_MIN_MAX(motor->tx_current, -DJI_6020_MOTOR_DATA_RANGE, DJI_6020_MOTOR_DATA_RANGE);
			break;
		default:break;
	}
}

//大疆电机初始化，输入电机类型，方便后续数据标准化
void DJImotor_Init(DJImotor_t* motor, uint8_t motor_type)
{
	motor->motor_type = motor_type;
}

//接收大疆电机数据，并化为国际单位
void DJImotor_Receive(DJImotor_t* motor, uint8_t* rx_data)
{
	motor->receive_cnt++;
	motor->err_percent = 1.0f * (motor->receive_cnt - motor->send_cnt) / motor->send_cnt;
	motor->last_ecd = motor->ecd;
	motor->ecd			= (uint16_t)(rx_data[0] << 8 | rx_data[1]);
	motor->speed_rpm	= (int16_t)(rx_data[2] << 8 | rx_data[3]);
	motor->rx_current		= (int16_t)(rx_data[4] << 8 | rx_data[5]);
	motor->temperature	= rx_data[6];
	if(motor->receive_cnt < 50)
	{
		motor->offset_ecd = motor->ecd;
		motor->round_cnt = 0;
	}
	if (motor->ecd - motor->last_ecd >= 4096)
		motor->round_cnt--;
	else if (motor->ecd - motor->last_ecd <= -4096)
		motor->round_cnt++;
	motor->total_ecd = motor->round_cnt * 8192 + motor->ecd - motor->offset_ecd;

	if(motor->motor_type == DJI_2006_MOTOR)
	{
		motor->position = 2.0f * PI * motor->total_ecd / 8192;
		motor->velocity = 2.0f * PI * motor->speed_rpm / 60;
		motor->torque = (float)motor->rx_current / DJI_2006_MOTOR_DATA_RANGE \
						* DJI_2006_MOTOR_CURRENT_RANGE * DJI_2006_MOTOR_TORQUE_CONSTANT \
						/ DJI_2006_ORIGINAL_REDUCTION_RATIO * DJI_2006_REDUCTION_RATIO;
	}
	else if(motor->motor_type == DJI_3508_MOTOR)
	{
		motor->position = 2.0f * PI * motor->total_ecd / 8192 / DJI_3508_REDUCTION_RATIO;
		motor->velocity = 2.0f * PI * motor->speed_rpm / 60 / DJI_3508_REDUCTION_RATIO;
		motor->torque = (float)motor->rx_current / DJI_3508_MOTOR_DATA_RANGE \
						* DJI_3508_MOTOR_CURRENT_RANGE * DJI_3508_MOTOR_TORQUE_CONSTANT \
						/ DJI_3508_ORIGINAL_REDUCTION_RATIO * DJI_3508_REDUCTION_RATIO;
	}
	else if(motor->motor_type == DJI_6020_MOTOR)
	{
		motor->position = 2.0f * PI * motor->total_ecd / 8192;
		motor->velocity = 2.0f * PI * motor->speed_rpm / 60;
		motor->torque = (float)motor->rx_current / DJI_6020_MOTOR_DATA_RANGE \
						* DJI_6020_MOTOR_CURRENT_RANGE * DJI_6020_MOTOR_TORQUE_CONSTANT;
	}
}
	
//将大疆电机控制力矩转化单位并发送
void DJImotor_SendControlTorque(void)
{
	static uint8_t tx_data[8];
	DJImotor_ConvertData(&driver_motor[0]);
	DJImotor_ConvertData(&driver_motor[1]);

//	memcpy(&tx_data[0], &driver_motor[0].tx_current, 2);
//	memcpy(&tx_data[2], &driver_motor[1].tx_current, 2);
////	memcpy(&tx_data[4], &driver_motor[0].tx_current, 2);
////	memcpy(&tx_data[6], &driver_motor[0].tx_current, 2);

	tx_data[0] = driver_motor[0].tx_current >> 8;
	tx_data[1] = driver_motor[0].tx_current;
	tx_data[2] = driver_motor[1].tx_current >> 8;
	tx_data[3] = driver_motor[1].tx_current;
	tx_data[4] = 0 >> 8;
	tx_data[5] = 0;
	tx_data[6] = 0 >> 8;
	tx_data[7] = 0;
	CANx_Transmit(&hcan2, 0x200, tx_data, 8);
}
