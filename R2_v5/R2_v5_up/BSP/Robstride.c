#include "Robstride.h"
#include <string.h>

RobStride_Motor RobStride_01;
float RS_kp, RS_kd, RS_des, RS_tor;
/*
 * 说明：
 * 1. 本文件使用 STM32 FDCAN 外设发送 RobStride 的 Classic CAN 帧。
 * 2. MIT协议仍然是8字节Classic CAN，不是CAN FD数据帧。
 * 3. 如RobStride不接在hfdcan3，请在初始化后调用 RobStride_Motor_Set_FDCAN(&hfdcanX)。
 */
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

typedef struct
{
	uint32_t StdId;
	uint32_t ExtId;
	uint32_t IDE;
	uint32_t RTR;
	uint32_t DLC;
} RobStride_CAN_TxHeaderTypeDef;

#define CAN_ID_STD FDCAN_STANDARD_ID
#define CAN_ID_EXT FDCAN_EXTENDED_ID
#define CAN_RTR_DATA FDCAN_DATA_FRAME

static FDCAN_HandleTypeDef *RobStride_FDCAN_Handle = &hfdcan3;
#define hcan (*RobStride_FDCAN_Handle)

static uint32_t RobStride_FDCAN_DLC(uint32_t dlc)
{
	switch (dlc)
	{
	case 0:
		return FDCAN_DLC_BYTES_0;
	case 1:
		return FDCAN_DLC_BYTES_1;
	case 2:
		return FDCAN_DLC_BYTES_2;
	case 3:
		return FDCAN_DLC_BYTES_3;
	case 4:
		return FDCAN_DLC_BYTES_4;
	case 5:
		return FDCAN_DLC_BYTES_5;
	case 6:
		return FDCAN_DLC_BYTES_6;
	case 7:
		return FDCAN_DLC_BYTES_7;
	case 8:
	default:
		return FDCAN_DLC_BYTES_8;
	}
}

static HAL_StatusTypeDef RobStride_FDCAN_AddTxMessage(FDCAN_HandleTypeDef *hfdcan, RobStride_CAN_TxHeaderTypeDef *can_header, uint8_t *data, uint32_t *mailbox)
{
	FDCAN_TxHeaderTypeDef tx_header = {0};

	if (mailbox != 0)
	{
		*mailbox = 0;
	}

	tx_header.Identifier = (can_header->IDE == CAN_ID_EXT) ? can_header->ExtId : can_header->StdId;
	tx_header.IdType = can_header->IDE;
	tx_header.TxFrameType = can_header->RTR;
	tx_header.DataLength = RobStride_FDCAN_DLC(can_header->DLC);
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
	{
		return HAL_BUSY;
	}

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}

void RobStride_Motor_Set_FDCAN(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan != 0)
	{
		RobStride_FDCAN_Handle = hfdcan;
	}
}
const uint16_t Index_List[15] = {
	0X7005, 0X7006, 0X700A, 0X700B, 0X7010,
	0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
	0x7019, 0x701A, 0x701B, 0x701C, 0x701D};
uint32_t Mailbox; // 定义邮箱变量

/*******************************************************************************
 * @功能     		: RobStride电机实例化的构造函数
 * @参数         : CAN ID
 * @返回值 			: void
 * @概述  				: 初始化电机ID。
 *******************************************************************************/
void RobStride_Motor_Init(RobStride_Motor *motor, uint8_t CAN_Id, uint8_t MIT_mode)
{
	memset(motor, 0, sizeof(*motor));
	data_read_write_init(&motor->drw, Index_List);
	motor->CAN_ID = CAN_Id;
	motor->Master_CAN_ID = 0xFD;
	motor->Motor_Set_All.set_motor_mode = move_control_mode;
	motor->MIT_Mode = MIT_mode;
	motor->MIT_Type = operationControl;
}

void RobStride_Motor_InitWithOffset(RobStride_Motor *motor, float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id, uint8_t MIT_mode)
{
	RobStride_Motor_Init(motor, CAN_Id, MIT_mode);
	motor->Motor_Offset_MotoFunc = Offset_MotoFunc;
}
/*******************************************************************************
 * @功能     		: uint16_t型转float型浮点数
 * @参数1        : 需要转换的值
 * @参数2        : x的最小值
 * @参数3        : x的最大值
 * @参数4        : 需要转换的进制数
 * @返回值 			: 十进制的float型浮点数
 * @概述  				: None
 *******************************************************************************/
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	uint32_t span = (1 << bits) - 1;
	x &= span;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}
/*******************************************************************************
 * @功能     		: float浮点数转int型
 * @参数1        : 需要转换的值
 * @参数2        : x的最小值
 * @参数3        : x的最大值
 * @参数4        : 需要转换的进制数
 * @返回值 			: 十进制的int型整数
 * @概述  				: None
 *******************************************************************************/
int float_to_uint_RS(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/*******************************************************************************
 * @功能     		: uint8_t数组转float浮点数
 * @参数        	: 需要转换的数组
 * @返回值 			: 十进制的float型浮点数
 * @概述  				: None
 *******************************************************************************/
float Byte_to_float(uint8_t *bytedata)
{
	float data_float = 0.0f;
	memcpy(&data_float, &bytedata[4], sizeof(float));
	return data_float;
}
/*******************************************************************************
 * @功能     	: MIT错位码 转换至 私有模式错位码
 * @参数        	: MIT错位码
 * @返回值 		: uint8_t
 * @概述  		: None
 *******************************************************************************/
uint8_t mapFaults(uint16_t fault16)
{
	uint8_t fault8 = 0;

	if (fault16 & (1 << 14))
		fault8 |= (1 << 4); // 过载故障
	if (fault16 & (1 << 7))
		fault8 |= (1 << 5); // 未标定
	if (fault16 & (1 << 3))
		fault8 |= (1 << 3); // 磁编码故障
	if (fault16 & (1 << 2))
		fault8 |= (1 << 0); // 欠压故障
	if (fault16 & (1 << 1))
		fault8 |= (1 << 1); // 驱动故障
	if (fault16 & (1 << 0))
		fault8 |= (1 << 2); // 过温

	return fault8;
}
// int count_num = 0 ;
/*******************************************************************************
 * @功能     	: 接收处理函数		（通信类型2 17应答帧 0应答帧）
 * @参数1        : 接收到的数据
 * @参数2        : 接收到的CANID
 * @返回值 		: None
 * @概述  		: drw只有通过通信17发送以后才有值
 *******************************************************************************/
void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId)
{
	if (motor->MIT_Mode)
	{
		if ((ID_ExtId & 0xFF) == 0XFD)
		{
			if (DataFrame[3] == 0x00 && DataFrame[4] == 0x00 && DataFrame[5] == 0x00 && DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
			{
				uint16_t fault16 = 0;
				memcpy(&fault16, &DataFrame[1], 2);
				motor->error_code = mapFaults(fault16);
			}
			else
			{
				motor->Pos_Info.Angle = uint16_to_float((DataFrame[1] << 8) | (DataFrame[2]), P_MIN, P_MAX, 16);
				motor->Pos_Info.Speed = uint16_to_float((DataFrame[3] << 4) | (DataFrame[4] >> 4), V_MIN, V_MAX, 12);
				motor->Pos_Info.Torque = uint16_to_float((DataFrame[4] << 8) | (DataFrame[5]), T_MIN, T_MAX, 12);
				motor->Pos_Info.Temp = ((DataFrame[6] << 8) | DataFrame[7]) * 0.1;
			}
		}
		else
		{
			memcpy(&motor->Unique_ID, DataFrame, 8);
		}
	}
	else
	{
		if ((uint8_t)((ID_ExtId & 0xFF00) >> 8) == motor->CAN_ID)
		{
			if ((int)((ID_ExtId & 0x3F000000) >> 24) == 2)
			{
				motor->Pos_Info.Angle = uint16_to_float(DataFrame[0] << 8 | DataFrame[1], P_MIN, P_MAX, 16);
				motor->Pos_Info.Speed = uint16_to_float(DataFrame[2] << 8 | DataFrame[3], V_MIN, V_MAX, 16);
				motor->Pos_Info.Torque = uint16_to_float(DataFrame[4] << 8 | DataFrame[5], T_MIN, T_MAX, 16);
				motor->Pos_Info.Temp = (DataFrame[6] << 8 | DataFrame[7]) * 0.1;
				motor->error_code = (uint8_t)((ID_ExtId & 0x3F0000) >> 16);
				motor->Pos_Info.pattern = (uint8_t)((ID_ExtId & 0xC00000) >> 22);
			}
			else if ((int)((ID_ExtId & 0x3F000000) >> 24) == 17)
			{
				for (int index_num = 0; index_num <= 13; index_num++)
				{
					if ((DataFrame[1] << 8 | DataFrame[0]) == Index_List[index_num])
						switch (index_num)
						{
						case 0:
							motor->drw.run_mode.data = (uint8_t)DataFrame[4];
							break;
						case 1:
							motor->drw.iq_ref.data = Byte_to_float(DataFrame);
							break;
						case 2:
							motor->drw.spd_ref.data = Byte_to_float(DataFrame);
							break;
						case 3:
							motor->drw.imit_torque.data = Byte_to_float(DataFrame);
							break;
						case 4:
							motor->drw.cur_kp.data = Byte_to_float(DataFrame);
							break;
						case 5:
							motor->drw.cur_ki.data = Byte_to_float(DataFrame);
							break;
						case 6:
							motor->drw.cur_filt_gain.data = Byte_to_float(DataFrame);
							break;
						case 7:
							motor->drw.loc_ref.data = Byte_to_float(DataFrame);
							break;
						case 8:
							motor->drw.limit_spd.data = Byte_to_float(DataFrame);
							break;
						case 9:
							motor->drw.limit_cur.data = Byte_to_float(DataFrame);
							break;
						case 10:
							motor->drw.mechPos.data = Byte_to_float(DataFrame);
							break;
						case 11:
							motor->drw.iqf.data = Byte_to_float(DataFrame);
							break;
						case 12:
							motor->drw.mechVel.data = Byte_to_float(DataFrame);
							break;
						case 13:
							motor->drw.VBUS.data = Byte_to_float(DataFrame);
							break;
						}
				}
			}
			else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
			{
				motor->CAN_ID = (uint8_t)((ID_ExtId & 0xFF00) >> 8);
				memcpy(&motor->Unique_ID, DataFrame, 8);
			}
		}
	}
}
/*******************************************************************************
 * @功能     		: RobStride电机获取设备ID和MCU（通信类型0）
 * @参数         : None
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void RobStride_Get_CAN_ID(RobStride_Motor *motor)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_Get_ID << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
 * @功能     		: RobStride电机运控模式  （通信类型1）
 * @参数1        : 力矩（-4Nm~4Nm）
 * @参数2        : 目标角度(-4π~4π)
 * @参数3        : 目标角速度(-30rad/s~30rad/s)
 * @参数4        : Kp(0.0~500.0)
 * @参数5        : Kp(0.0~5.0)
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	motor->Motor_Set_All.set_Torque = Torque;
	motor->Motor_Set_All.set_angle = Angle;
	motor->Motor_Set_All.set_speed = Speed;
	motor->Motor_Set_All.set_Kp = Kp;
	motor->Motor_Set_All.set_Kd = Kd;
	if (motor->drw.run_mode.data != 0)
	{
		Set_RobStride_Motor_parameter(motor, 0X7005, move_control_mode, Set_mode); // 设置电机模式
		Get_RobStride_Motor_parameter(motor, 0x7005);
		Enable_Motor(motor);
		motor->Motor_Set_All.set_motor_mode = move_control_mode;
	}
	if (motor->Pos_Info.pattern != 2)
	{
		Enable_Motor(motor);
	}
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotionControl << 24 | float_to_uint_RS(motor->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8 | motor->CAN_ID;
	txdata[0] = float_to_uint_RS(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16) >> 8;
	txdata[1] = float_to_uint_RS(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
	txdata[2] = float_to_uint_RS(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16) >> 8;
	txdata[3] = float_to_uint_RS(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
	txdata[4] = float_to_uint_RS(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16) >> 8;
	txdata[5] = float_to_uint_RS(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16);
	txdata[6] = float_to_uint_RS(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16) >> 8;
	txdata[7] = float_to_uint_RS(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16);
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
// MIT模式使能
void RobStride_Motor_MIT_Enable(RobStride_Motor *motor)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;
	txMsg.IDE = CAN_ID_STD;
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.DLC = 8;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFC;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

// MIT模式失能
void RobStride_Motor_MIT_Disable(RobStride_Motor *motor)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;
	txMsg.IDE = CAN_ID_STD;
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.DLC = 8;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFD;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

// MIT模式清除或检查错误
void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;			   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;					   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;				   // 设置远程传输请求
	txMsg.DLC = 8;							   // 设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFB;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}

// MIT设置电机运行模式
void RobStride_Motor_MIT_SetMotorType(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;			   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;					   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;				   // 设置远程传输请求
	txMsg.DLC = 8;							   // 设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFC;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}

// MIT设置电机ID
void RobStride_Motor_MIT_SetMotorId(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;			   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;					   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;				   // 设置远程传输请求
	txMsg.DLC = 8;							   // 设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0x01;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}

// MIT控制模式
void RobStride_Motor_MIT_Control(RobStride_Motor *motor, float Angle, float Speed, float Kp, float Kd, float Torque)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;			   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;					   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;				   // 设置远程传输请求
	txMsg.DLC = 8;							   // 设置数据长度
	txdata[0] = float_to_uint_RS(Angle, P_MIN, P_MAX, 16) >> 8;
	txdata[1] = float_to_uint_RS(Angle, P_MIN, P_MAX, 16);
	txdata[2] = float_to_uint_RS(Speed, V_MIN, V_MAX, 12) >> 4;
	txdata[3] = float_to_uint_RS(Speed, V_MIN, V_MAX, 12) << 4 | float_to_uint_RS(Kp, KP_MIN, KP_MAX, 12) >> 8;
	txdata[4] = float_to_uint_RS(Kp, KP_MIN, KP_MAX, 12);
	txdata[5] = float_to_uint_RS(Kd, KD_MIN, KD_MAX, 12) >> 4;
	txdata[6] = float_to_uint_RS(Kd, KD_MIN, KD_MAX, 12) << 4 | float_to_uint_RS(Torque, T_MIN, T_MAX, 12) >> 8;
	txdata[7] = float_to_uint_RS(Torque, T_MIN, T_MAX, 12);
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}

// MIT位置模式
void RobStride_Motor_MIT_PositionControl(RobStride_Motor *motor, float position_rad, float speed_rad_per_s)
{
	uint8_t txdata[8] = {0};									   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0};					   // 发送邮箱
	txMsg.StdId = (1 << 8) | motor->CAN_ID;						   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;										   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;									   // 设置远程传输请求
	txMsg.DLC = 8;												   // 设置数据长度
	memcpy(&txdata[0], &position_rad, 4);						   // 将位置数据复制到发送数据数组中
	memcpy(&txdata[4], &speed_rad_per_s, 4);					   // 将速度数据复制到发送数据数组中
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}
// MIT模式速度控制实现
void RobStride_Motor_MIT_SpeedControl(RobStride_Motor *motor, float speed_rad_per_s, float current_limit)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = (2 << 8) | motor->CAN_ID;
	txMsg.IDE = CAN_ID_STD;	  // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; // 设置远程传输请求
	txMsg.DLC = 8;			  // 设置数据长度
	memcpy(&txdata[0], &speed_rad_per_s, 4);
	memcpy(&txdata[4], &current_limit, 4);
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

// MIT零点设置模式
void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor *motor)
{
	uint8_t txdata[8] = {0};				   // 定义发送数据数组
	RobStride_CAN_TxHeaderTypeDef txMsg = {0}; // 发送邮箱
	txMsg.StdId = motor->CAN_ID;			   // 设置标准ID
	txMsg.IDE = CAN_ID_STD;					   // 设置标识符类型
	txMsg.RTR = CAN_RTR_DATA;				   // 设置远程传输请求
	txMsg.DLC = 8;							   // 设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFE;
	RobStride_FDCAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
 * @功能     		: RobStride电机位置模式(PP插补位置模式控制)
 * @参数1        : 目标角速度(-30rad/s~30rad/s)
 * @参数2        : 目标角度(-4π~4π)
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Speed, float Angle)
{
	motor->Motor_Set_All.set_speed = Speed;
	motor->Motor_Set_All.set_angle = Angle;
	//		motor->Motor_Set_All.set_limit_speed = vel_max;
	//		motor->Motor_Set_All.set_acceleration = acc_set;
	//		if (motor->drw.run_mode.data != 1 && motor->Pos_Info.pattern == 2)
	if (motor->drw.run_mode.data != 1)
	{
		Set_RobStride_Motor_parameter(motor, 0X7005, Pos_control_mode, Set_mode); // 设置电机模式
		Get_RobStride_Motor_parameter(motor, 0x7005);
		motor->Motor_Set_All.set_motor_mode = Pos_control_mode;
		Enable_Motor(motor);
		Set_RobStride_Motor_parameter(motor, 0X7024, motor->Motor_Set_All.set_limit_speed, Set_parameter);
		Set_RobStride_Motor_parameter(motor, 0X7025, motor->Motor_Set_All.set_acceleration, Set_parameter);
	}
	HAL_Delay(1);
	Set_RobStride_Motor_parameter(motor, 0X7016, motor->Motor_Set_All.set_angle, Set_parameter);
}
/*******************************************************************************
 * @功能     		: RobStride电机位置模式(CSP位置模式控制)
 * @参数1        : 目标角度(-4π~4π)
 * @参数2        : 目标角速度(0rad/s~44rad/s)
 * @返回值 				: void
 * @概述  				: None
 *******************************************************************************/
void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd)
{
	if (motor->MIT_Mode)
	{
		RobStride_Motor_MIT_PositionControl(motor, Angle, limit_spd);
	}
	else
	{
		motor->Motor_Set_All.set_angle = Angle;
		motor->Motor_Set_All.set_limit_speed = limit_spd;
		if (motor->drw.run_mode.data != 1)
		{
			Set_RobStride_Motor_parameter(motor, 0X7005, CSP_control_mode, Set_mode);
			Get_RobStride_Motor_parameter(motor, 0x7005);
			Enable_Motor(motor);
			Set_RobStride_Motor_parameter(motor, 0X7017, motor->Motor_Set_All.set_limit_speed, Set_parameter);
		}
		HAL_Delay(1);
		Set_RobStride_Motor_parameter(motor, 0X7016, motor->Motor_Set_All.set_angle, Set_parameter);
	}
}

/*******************************************************************************
 * @功能     		: RobStride电机速度模式
 * @参数1        : 目标角速度(-30rad/s~30rad/s)
 * @参数2        : 目标电流限制(0~23A)
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur)
{
	motor->Motor_Set_All.set_speed = Speed;
	motor->Motor_Set_All.set_limit_cur = limit_cur;
	if (motor->drw.run_mode.data != 2)
	{
		Set_RobStride_Motor_parameter(motor, 0X7005, Speed_control_mode, Set_mode); // 设置电机模式
		Get_RobStride_Motor_parameter(motor, 0x7005);
		Enable_Motor(motor);
		motor->Motor_Set_All.set_motor_mode = Speed_control_mode;
		Set_RobStride_Motor_parameter(motor, 0X7018, motor->Motor_Set_All.set_limit_cur, Set_parameter);
		Set_RobStride_Motor_parameter(motor, 0X7022, 10, Set_parameter);
		//		Set_RobStride_Motor_parameter(motor, 0X7022, motor->Motor_Set_All.set_acceleration, Set_parameter);
	}
	Set_RobStride_Motor_parameter(motor, 0X700A, motor->Motor_Set_All.set_speed, Set_parameter);
}
/*******************************************************************************
 * @功能     		: RobStride电机电流模式
 * @参数         : 目标电流(-23~23A)
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStride_Motor_current_control(RobStride_Motor *motor, float current)
{
	motor->Motor_Set_All.set_current = current;
	motor->output = motor->Motor_Set_All.set_current;
	if (motor->Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStride_Motor_parameter(motor, 0X7005, Elect_control_mode, Set_mode); // 设置电机模式
		Get_RobStride_Motor_parameter(motor, 0x7005);
		motor->Motor_Set_All.set_motor_mode = Elect_control_mode;
		Enable_Motor(motor);
	}
	Set_RobStride_Motor_parameter(motor, 0X7006, motor->Motor_Set_All.set_current, Set_parameter);
}
/*******************************************************************************
 * @功能     	: RobStride电机零点模式(电机回到机械零点)
 * @参数         : None
 * @返回值 		: void
 * @概述  		: None
 *******************************************************************************/
void RobStride_Motor_Set_Zero_control(RobStride_Motor *motor)
{
	Set_RobStride_Motor_parameter(motor, 0X7005, Set_Zero_mode, Set_mode); // 设置电机模式
}
/*******************************************************************************
 * @功能     		: RobStride电机使能 （通信类型3）
 * @参数         : None
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void Enable_Motor(RobStride_Motor *motor)
{
	if (motor->MIT_Mode)
	{
		RobStride_Motor_MIT_Enable(motor);
	}
	else
	{
		uint8_t txdata[8] = {0};					   // 发送数据
		RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱

		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		TxMessage.ExtId = Communication_Type_MotorEnable << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
		RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
	}
}
/*******************************************************************************
 * @功能     		: RobStride电机失能 （通信类型4）
 * @参数         : 是否清除错误位（0不清除 1清除）
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void Disenable_Motor(RobStride_Motor *motor, uint8_t clear_error)
{
	if (motor->MIT_Mode)
	{
		RobStride_Motor_MIT_Disable(motor);
	}
	else
	{
		uint8_t txdata[8] = {0};					   // 发送数据
		RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱

		txdata[0] = clear_error;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		TxMessage.ExtId = Communication_Type_MotorStop << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
		RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
		Set_RobStride_Motor_parameter(motor, 0X7005, move_control_mode, Set_mode);
	}
}
/*******************************************************************************
 * @功能     		: RobStride电机写入参数 （通信类型18）
 * @参数1        : 参数地址
 * @参数2        : 参数数值
 * @参数3        : 选择是传入控制模式 还是其他参数 （Set_mode设置控制模式 Set_parameter设置参数）
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = Index;
	txdata[1] = Index >> 8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;
	if (Value_mode == 'p')
	{
		memcpy(&txdata[4], &Value, 4);
	}
	else if (Value_mode == 'j')
	{
		motor->Motor_Set_All.set_motor_mode = (int)Value;
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;
		txdata[6] = 0x00;
		txdata[7] = 0x00;
	}
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
 * @功能     		: RobStride电机单个参数读取 （通信类型17）
 * @参数         : 参数地址
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	txdata[0] = Index;
	txdata[1] = Index >> 8;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_GetSingleParameter << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
 * @功能     		: RobStride电机设置CAN_ID （通信类型7）
 * @参数         : 修改后（预设）CANID
 * @返回值 			: void
 * @概述  				: None
 *******************************************************************************/
void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID)
{
	Disenable_Motor(motor, 0);
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_Can_ID << 24 | Set_CAN_ID << 16 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
 * @功能     		: RobStride电机设置机械零点 （通信类型6）
 * @参数         : None
 * @返回值 			: void
 * @概述  				: 会把当前电机位置设为机械零位， 会先失能电机, 再使能电机
 *******************************************************************************/
void Set_ZeroPos(RobStride_Motor *motor)
{
	Disenable_Motor(motor, 0);					   // 失能电机
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetPosZero << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = 1;
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
	Enable_Motor(motor);
}

/*******************************************************************************
 * @功能     		: RobStride电机数据保存 （通信类型22）
 * @参数      		: None
 * @返回值 				: void
 * @概述  				: 会把当前单参数写入表中的数据写为默认值，重新上电后参数保持为此指令运行时的参数
 *******************************************************************************/
void RobStride_Motor_MotorDataSave(RobStride_Motor *motor)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorDataSave << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = 0x07;
	txdata[7] = 0x08;
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机波特率修改 （通信类型23）
* @参数      		: 波特率模式:	 01（1M）
									02（500K）
									03（250K）
									04（125K）
* @返回值 				: void
* @概述  				: 将电机波特率修改为对应的值，例如参数为01，波特率修改为1M
*******************************************************************************/
void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_BaudRateChange << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;												   // 这一字节无所谓，填什么都可以，这里写的是0x08
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机主动上报设置 （通信类型24）
* @参数      		: 上报模式：	00（关闭）
														01（开启）
* @返回值 				: void
* @概述  				: 开启/关闭 电机主动上报，默认上报周期为10ms
*******************************************************************************/
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_ProactiveEscalationSet << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;												   // 这一字节无所谓，填什么都可以，这里写的是0x08
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机协议修改 （通信类型25）
* @参数      		: 协议类型：		00（私有协议）
										01（Canopen）
										02（MIT协议）
* @返回值 				: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor_MIT_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.StdId = motor->CAN_ID;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFD;												   // 这一字节无所谓，填什么都可以，这里写的是0x08
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
 * @功能     	: FDCAN接收帧转交给RobStride解析
 * @参数1       : 电机对象
 * @参数2       : FDCAN接收头
 * @参数3       : FDCAN接收数据，至少8字节
 * @返回值 		: 1表示已经转交解析，0表示不是该电机的帧或参数非法
 * @概述  		: 在 HAL_FDCAN_RxFifo0Callback() 中调用即可。
 *******************************************************************************/
uint8_t RobStride_Motor_RxFDCAN(RobStride_Motor *motor, const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
	uint32_t id = 0;

	if ((motor == 0) || (rx_header == 0) || (rx_data == 0))
	{
		return 0;
	}

	id = rx_header->Identifier;

	if (motor->MIT_Mode)
	{
		/*
		 * MIT模式通常为标准帧：
		 * 发送控制：StdId = motor->CAN_ID
		 * 反馈帧：StdId低8位通常为Master_CAN_ID，即0xFD，rx_data[0]为电机ID
		 */
		if (rx_header->IdType != FDCAN_STANDARD_ID)
		{
			return 0;
		}

		if (((id & 0xFFU) == (motor->Master_CAN_ID & 0xFFU)) && (rx_data[0] == motor->CAN_ID))
		{
			RobStride_Motor_Analysis(motor, rx_data, id);
			return 1;
		}

		/*
		 * 部分指令反馈可能直接使用电机ID或其他标准ID；
		 * 这里保留一个兼容入口，但不抢其他电机的反馈。
		 */
		if ((id & 0xFFU) == motor->CAN_ID)
		{
			RobStride_Motor_Analysis(motor, rx_data, id);
			return 1;
		}

		return 0;
	}

	/*
	 * 私有协议主要使用扩展帧。
	 * RobStride_Motor_Analysis() 内部还会再次检查ID中的电机号。
	 */
	if (rx_header->IdType == FDCAN_EXTENDED_ID)
	{
		RobStride_Motor_Analysis(motor, rx_data, id);
		return 1;
	}

	return 0;
}

/*******************************************************************************
 * @功能     		: RobStride电机数据的参数地址初始化
 * @参数         : 数据的参数地址数组
 * @返回值 			: void
 * @概述  				: 会在创建电机类时自动调用
 *******************************************************************************/
void data_read_write_init(data_read_write *drw, const uint16_t *index_list)
{
	if (index_list == 0)
	{
		index_list = Index_List;
	}

	drw->run_mode.index = index_list[0];
	drw->iq_ref.index = index_list[1];
	drw->spd_ref.index = index_list[2];
	drw->imit_torque.index = index_list[3];
	drw->cur_kp.index = index_list[4];
	drw->cur_ki.index = index_list[5];
	drw->cur_filt_gain.index = index_list[6];
	drw->loc_ref.index = index_list[7];
	drw->limit_spd.index = index_list[8];
	drw->limit_cur.index = index_list[9];
	drw->mechPos.index = index_list[10];
	drw->iqf.index = index_list[11];
	drw->mechVel.index = index_list[12];
	drw->VBUS.index = index_list[13];
	drw->rotation.index = index_list[14];
}

void RobStride_Motor_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};					   // 发送数据
	RobStride_CAN_TxHeaderTypeDef TxMessage = {0}; // 发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorModeSet << 24 | motor->Master_CAN_ID << 8 | motor->CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;												   // 这个一位随便填，此处填0x08
	RobStride_FDCAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

void RobStride_Motor_Set_MIT_Mode(RobStride_Motor *motor, uint8_t MIT_mode)
{
	motor->MIT_Mode = MIT_mode;
}

void RobStride_Motor_Set_MIT_Type(RobStride_Motor *motor, MIT_TYPE MIT_type)
{
	motor->MIT_Type = MIT_type;
}

uint8_t RobStride_Motor_Get_MIT_Mode(const RobStride_Motor *motor)
{
	return motor->MIT_Mode;
}

MIT_TYPE RobStride_Motor_Get_MIT_Type(const RobStride_Motor *motor)
{
	return motor->MIT_Type;
}
