#include "can_comm.h"
#include "HTmotor_driver.h"
#include "DJImotor_driver.h"
#include "Timu_driver.h"

CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef Rx1Message;
CAN_RxHeaderTypeDef Rx2Message;
uint8_t CAN1_Rx_data[8];
uint8_t CAN2_Rx_data[8];

static void CANx_Filter_Init(CAN_HandleTypeDef *hcan, uint32_t *ID, uint8_t num);

//can总线初始化以及所有can设备初始化
void CAN_Comm_Init(void)
{
	//设置canID
	uint32_t can_ID1[4] = {HT_MOTOR_ID, IMU_PALSTANCE_ID, IMU_ANGLE_ID, IMU_ELSE_ID};
	uint32_t can_ID2[2] = {0x201, 0x202};
	CANx_Filter_Init(&hcan1, can_ID1, sizeof(can_ID1)/sizeof(can_ID1[0]));
	CANx_Filter_Init(&hcan2, can_ID2, sizeof(can_ID2)/sizeof(can_ID2[0]));

	//打开can并使能邮箱
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);

	//海泰电机初始化
	HTmotor_Init(&joint_motor[0], &hcan1, 0x01, 0.0f);
	HTmotor_Init(&joint_motor[1], &hcan1, 0x02, 0.0f);
	HTmotor_Init(&joint_motor[2], &hcan1, 0x03, 0.0f);
	HTmotor_Init(&joint_motor[3], &hcan1, 0x04, 0.0f);

	//DJImotor Init
	DJImotor_Init(&driver_motor[0], DJI_3508_MOTOR);
	DJImotor_Init(&driver_motor[1], DJI_3508_MOTOR);
}

//邮箱0接收回调函数，接收can1数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, CAN1_Rx_data);
		switch(Rx1Message.StdId)
		{
			case HT_MOTOR_ID:
			{
				switch (CAN1_Rx_data[0])
				{
					case 0x01:HTmotor_Receive(&joint_motor[0], CAN1_Rx_data);break;
					case 0x02:HTmotor_Receive(&joint_motor[1], CAN1_Rx_data);break;
					case 0x03:HTmotor_Receive(&joint_motor[2], CAN1_Rx_data);break;
					case 0x04:HTmotor_Receive(&joint_motor[3], CAN1_Rx_data);break;
					default:break;
				}
				break;
			}
			case IMU_PALSTANCE_ID:
			case IMU_ANGLE_ID:
			case IMU_ELSE_ID:
				Timu_Receive(&imu, Rx1Message.StdId, CAN1_Rx_data);break;
			default:break;
		}
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	} 
}

//邮箱1接收回调函数，接收can2数据
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Rx2Message, CAN2_Rx_data);
		switch(Rx2Message.StdId)
		{
			case 0x201:
				DJImotor_Receive(&driver_motor[0], CAN2_Rx_data);break;
			case 0x202:
				DJImotor_Receive(&driver_motor[1], CAN2_Rx_data);break;
			default:
				break;
		}
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	}
}

//can发送数据统一接口，提供给其它文件调用
void CANx_Transmit(CAN_HandleTypeDef *hcan, uint32_t tx_id, uint8_t* tx_data, uint8_t len)
{
	uint8_t FreeTxNum = 0;
	static uint32_t TxMailbox;

	TxMessage.StdId = tx_id;
	TxMessage.IDE	= CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = len;

	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	while(FreeTxNum == 0) 
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);

	HAL_CAN_AddTxMessage(hcan, &TxMessage, tx_data, &TxMailbox);
}

//can过滤器初始化
//输入ID数组指针，自动计算数组长度，列表模式使用对每个ID进行过滤，一个can有14个过滤器
//理论一个can最高可以配置14*4=56个ID
//can1收发用邮箱0，can2收发用邮箱1
static void CANx_Filter_Init(CAN_HandleTypeDef *hcan, uint32_t *ID, uint8_t num)
{
	uint8_t i = 0, filter_num = 0;
	CAN_FilterTypeDef can_filter;

	uint8_t ID_num = num;

	can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter.SlaveStartFilterBank = 14;
	can_filter.FilterActivation = ENABLE;
	
	if(hcan == &hcan1)
	{
		can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	}
	else if(hcan == &hcan2)
	{
		filter_num = 14;
		can_filter.FilterFIFOAssignment = CAN_FilterFIFO1;
	}
	
	while(i < ID_num)
	{
		can_filter.FilterBank = filter_num;
		can_filter.FilterIdHigh = ID[i++] << 5;
		if(i < ID_num)
			can_filter.FilterIdLow = ID[i++] << 5;
		else
		{
			HAL_CAN_ConfigFilter(hcan, &can_filter);
			while(HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK);
			break;
		}
		if(i < ID_num)
			can_filter.FilterMaskIdHigh = ID[i++] << 5;
		else
		{
			HAL_CAN_ConfigFilter(hcan, &can_filter);
			while(HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK);
			break;
		}
		if(i < ID_num)
		{
			filter_num++;
			can_filter.FilterMaskIdLow = ID[i++] << 5;
			HAL_CAN_ConfigFilter(hcan, &can_filter);
			while(HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK);
		}
		else
		{
			HAL_CAN_ConfigFilter(hcan, &can_filter);
			while(HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK);
			break;
		}
	}
}
