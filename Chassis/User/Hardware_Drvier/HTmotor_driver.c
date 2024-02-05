#include "HTmotor_driver.h"
#include "can_comm.h"
#include "can.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

HTmotor_t joint_motor[4];

//根据协议，对float参数进行转换，用于数据发送
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

//根据协议，对uint参数进行转换，用于数据接收
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//海泰电机初始化
//输入电机所在can总线以及电机ID，方便后续自动发送
void HTmotor_Init(HTmotor_t* motor, CAN_HandleTypeDef* hcan, uint32_t id, float zero_point)
{
	motor->id = id;
	motor->hcan = hcan;
	HTmotor_ControlCmd(motor, CMD_MOTOR_MODE);
	HAL_Delay(1);
	HTmotor_SendControlPara(motor);
	HAL_Delay(1);
}

//发送海泰电机控制参数
void HTmotor_SendControlPara(HTmotor_t* motor)
{
	uint16_t p, v, kp, kd, t;
	uint8_t buf[8];

	//限制输入的参数在定义的范围内
	LIMIT_MIN_MAX(motor->p,  P_MIN,  P_MAX);
	LIMIT_MIN_MAX(motor->v,  V_MIN,  V_MAX);
	LIMIT_MIN_MAX(motor->kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(motor->kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(motor->t,  T_MIN,  T_MAX);

	//根据协议，对float参数进行转换
	p = float_to_uint(motor->p,      P_MIN,  P_MAX,  16);
	v = float_to_uint(motor->v,      V_MIN,  V_MAX,  12);
	kp = float_to_uint(motor->kp,    KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor->kd,    KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor->t,      T_MIN,  T_MAX,  12);

	//根据传输协议，把数据转换为CAN命令数据字段
	buf[0] = p >> 8;
	buf[1] = p & 0xFF;
	buf[2] = v >> 4;
	buf[3] = ((v & 0xF) << 4) | (kp >> 8);
	buf[4] = kp & 0xFF;
	buf[5] = kd >> 4;
	buf[6] = ((kd & 0xF) << 4) | (t >> 8);
	buf[7] = t & 0xff;

	//通过CAN接口把buf中的内容发送出去
	motor->send_cnt++;
	CANx_Transmit(motor->hcan, motor->id, buf, sizeof(buf));
}

//发送海泰电机控制命令
void HTmotor_ControlCmd(HTmotor_t* motor, uint8_t cmd)
{
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
	switch(cmd)
	{
		case CMD_MOTOR_MODE:
			buf[7] = 0xFC;
			break;
		case CMD_RESET_MODE:
			buf[7] = 0xFD;
		break;
		case CMD_ZERO_POSITION:
			buf[7] = 0xFE;
		break;
		default:
		return; /* 直接退出函数 */
	}
	CANx_Transmit(motor->hcan, motor->id, buf, sizeof(buf));
}

//接收海泰电机数据
void HTmotor_Receive(HTmotor_t* motor, uint8_t* rx_data)
{
	uint16_t tmp_value;
	motor->receive_cnt++;
	motor->err_percent = 1.0f * (motor->receive_cnt - motor->send_cnt) / motor->send_cnt;

	// 根据协议，对uint参数进行转换
	tmp_value = (rx_data[1] << 8) |  rx_data[2];
	motor->position = uint_to_float(tmp_value, P_MIN, P_MAX, 16);
	tmp_value = (rx_data[3] << 4) | (rx_data[4] >> 4);
	motor->velocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
	tmp_value = ((0x0f & rx_data[4]) << 8) | rx_data[5];
	motor->torque = uint_to_float(tmp_value, T_MIN, T_MAX, 12);
}
