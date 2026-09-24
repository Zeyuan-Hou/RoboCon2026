#include "bsp_can.h"
void can1_start(void)
{
    // 滤波器初始化
    CAN1_FILTER_CONFIG(&hcan1);

    // 启动CAN1
    HAL_CAN_Start(&hcan1);

    // 使能中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// @brief 配置过滤器
// @param CAN1 or CAN2
// @retval None
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void can2_start(void)
{
    // 滤波器初始化
    CAN2_FILTER_CONFIG(&hcan2);

    // 启动CAN1
    HAL_CAN_Start(&hcan2);

    // 使能中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_FILTER_CONFIG(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 14;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 28;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/**
 * @brief 发送标准ID的数据帧
 * @param hcan CAN的句柄
 * @param ID 数据帧ID
 * @param pData 数组指针
 * @param Len 字节数0~8，请选择0x08
 */
void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
    static CAN_TxHeaderTypeDef Tx_Header;

    Tx_Header.StdId = ID;
 //   Tx_Header.ExtId = CAN_ID_STD;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = Len;

    /* 找到空的发送邮箱，把数据发送出去 */
    if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX2);
        }
    }
}

void CAN_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t TxCurrent[8];
    TxCurrent[0] = (current1 >> 8);
    TxCurrent[1] = current1;
    TxCurrent[2] = (current2 >> 8);
    TxCurrent[3] = current2;
    TxCurrent[4] = (current3 >> 8);
    TxCurrent[5] = current3;
    TxCurrent[6] = (current4 >> 8);
    TxCurrent[7] = current4;
    CANx_SendStdData(hcan, id, TxCurrent, 8);
}


float GetEncoderNumber(ST_MOTOR *motor, uint8_t msg[8])
{
    motor->EncoderNum = (msg[0] << 8) | (msg[1]);
    return motor->EncoderNum;
}

float GetSpeed_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8])
{
    float speed_temp;
		memcpy(&speed_temp,&msg[4],4);
    return speed_temp;
}
float GetAngle_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8])
{
		float angle_temp;
		memcpy(&angle_temp,msg,4);
    return angle_temp;
	
}
float GetSpeed_DJI ( CAN_RxHeaderTypeDef* pCanRxMsg, uint8_t msg[8]) 
{  
    int32_t speed_temp;  
    int32_t base_value = 0xFFFF; // 可能是一个基础值或者偏移量  
    if (msg[2] & 0x01 << 7) { // 检查msg[2]的最高位  
        speed_temp = (base_value << 16 | msg[2] << 8 | msg[3]); // 如果最高位为1，则按这种方式计算速度  
    } else {  
        speed_temp = (msg[2] << 8) |( msg[3]); // 如果最高位为0，则按这种方式计算速度  
    }  
    return (float)speed_temp; // 注意：原函数返回类型为float，所以这里进行了强制类型转换  
}
void Abs_Encoder_Process(ST_ENCODER *pEncoder, uint32_t value)
{
    pEncoder->siPreRawValue = pEncoder->siRawValue;
    pEncoder->siRawValue = value;
    pEncoder->siDiff = pEncoder->siRawValue - pEncoder->siPreRawValue;

    if (pEncoder->siDiff > (pEncoder->siNumber / 2))
    {
        pEncoder->siDiff -= pEncoder->siNumber;
    }
    else if (pEncoder->siDiff < -(pEncoder->siNumber / 2))
    {
        pEncoder->siDiff += pEncoder->siNumber;
    }

    pEncoder->siSumValue += pEncoder->siDiff;
		if(pEncoder->state == 0)//清零
		{
			pEncoder->siSumValue = 0;
			pEncoder->state = 1;
		}
}



uint32_t FloatToUint(const float x, const float x_min, const float x_max, const uint8_t bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x-offset)*((float)((1<<bits)-1))/span);
}
float UintToFloat(const int x_int, const float x_min, const float x_max, const uint8_t bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//用于将MotorCMD中的float数据转换为CAN协议中发送的uint数据
void FloatsToUints(MotorCMD *param, uint8_t *data)
{
    uint16_t _position = FloatToUint(param->position_, POSITION_MIN, POSITION_MAX, SEND_POSITION_LENGTH);
    uint16_t _velocity = FloatToUint(param->velocity_, VELOCITY_MIN, VELOCITY_MAX, SEND_VELOCITY_LENGTH);
    uint16_t _torque = FloatToUint(param->torque_, TORQUE_MIN, TORQUE_MAX, SEND_TORQUE_LENGTH);
    uint16_t _kp = FloatToUint(param->kp_, KP_MIN, KP_MAX, SEND_KP_LENGTH);
    uint16_t _kd = FloatToUint(param->kd_, KD_MIN, KD_MAX, SEND_KD_LENGTH);
    data[0] = _position;
    data[1] = _position >> 8;
    data[2] = _velocity;
    data[3] = ((_velocity >> 8) & 0x3f)| ((_kp & 0x03) << 6);
    data[4] = _kp >> 2;
    data[5] = _kd;
    data[6] = _torque;
    data[7] = _torque >> 8;
}

//用于将CAN协议中收到的uint数据转换为MotorDATA中的float数据
void UintsToFloats(uint8_t rxdata[8], MotorDATA *data)
{
	const ReceivedMotionData *pcan_data = (const ReceivedMotionData*)rxdata;
    data->position_ = UintToFloat(pcan_data->position, POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
    data->velocity_ = UintToFloat(pcan_data->velocity, VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
    data->torque_ = UintToFloat(pcan_data->torque, TORQUE_MIN, TORQUE_MAX, RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)pcan_data->temp_flag;
    if(data->flag_ == kMotorTempFlag){
        data->temp_ = UintToFloat(pcan_data->temperature, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
    else{
        data->temp_ = UintToFloat(pcan_data->temperature, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
}

void ctrl_motor(CAN_HandleTypeDef *hcan,uint16_t id, MotorCMD *param,uint16_t dlc)
{
	uint8_t Data[8];
	FloatsToUints(param,Data);
	CANx_SendStdData(hcan,id,Data,dlc);
}

uint8_t RxMsg_CAN1[8];  
uint8_t RxMsg_CAN2[8]; 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if (hcan->Instance == CAN1)
    {                
				system_monitor.can_rec_cnt[0]++;   
        CAN_RxHeaderTypeDef RxHeader;                                           
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN1);        
        switch (RxHeader.StdId)
        {       
					case 0x201:                        
					  leftup_motor.angle =  GetAngle_V6(&RxHeader, RxMsg_CAN1);
						wheel_encoder_velt_filter.leftup_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
						LpFilter(&wheel_encoder_velt_filter.leftup_velt);
						leftup_motor.anglev = wheel_encoder_velt_filter.leftup_velt.out;
						
						chassis_run.leftup.fpFB = leftup_motor.anglev;
						system_monitor.motor_LU_cnt++;
						break;
					case 0x202:
						rightup_motor.angle =  GetAngle_V6(&RxHeader, RxMsg_CAN1);
					  wheel_encoder_velt_filter.rightup_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
						LpFilter(&wheel_encoder_velt_filter.rightup_velt);
						rightup_motor.anglev = wheel_encoder_velt_filter.rightup_velt.out;
					
						chassis_run.rightup.fpFB = rightup_motor.anglev;
						system_monitor.motor_RU_cnt++;
						break;
					case 0x203:
						rightdown_motor.angle =  GetAngle_V6(&RxHeader, RxMsg_CAN1);
					  wheel_encoder_velt_filter.rightdown_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
						LpFilter(&wheel_encoder_velt_filter.rightdown_velt);
						rightdown_motor.anglev = wheel_encoder_velt_filter.rightdown_velt.out ;
						
						chassis_run.rightdown.fpFB = rightdown_motor.anglev;
						system_monitor.motor_RD_cnt++;
						break;
					case 0x204:
						leftdown_motor.angle =  GetAngle_V6(&RxHeader, RxMsg_CAN1);
					  wheel_encoder_velt_filter.leftdown_velt.in = GetSpeed_V6(&RxHeader, RxMsg_CAN1);
						LpFilter(&wheel_encoder_velt_filter.leftdown_velt);
						leftdown_motor.anglev = wheel_encoder_velt_filter.leftdown_velt.out;
					
						chassis_run.leftdown.fpFB = leftdown_motor.anglev;
						system_monitor.motor_LD_cnt++;
						break;
					default:
							break;
        }
    }
		 if (hcan->Instance == CAN2)
    {
        system_monitor.can_rec_cnt[1]++;                                       
        CAN_RxHeaderTypeDef RxHeader;                                           
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN2);        
        switch (RxHeader.StdId)
        {        
					case 0x201:
						left_xc5000_motor.Motor_Data.angle = GetAngle_V6(&RxHeader, RxMsg_CAN2) * RADIAN;
						if(left_xc5000_motor.Motor_Data.flag_init == 0)
						{
							left_xc5000_motor.Motor_Data.init_angle = left_xc5000_motor.Motor_Data.angle;
							left_xc5000_motor.Motor_Data.flag_init = 1;
						}
						left_xc5000_motor.Real_Delta_Pos = left_xc5000_motor.Motor_Data.angle - left_xc5000_motor.Motor_Data.init_angle;
						left_xc5000_motor.Outer_Pid.fpFB = left_xc5000_motor.Real_Delta_Pos;
						
						left_xc5000_motor.Encoder_Filter.in = GetSpeed_V6(&RxHeader, RxMsg_CAN2);
						LpFilter(&left_xc5000_motor.Encoder_Filter);
						left_xc5000_motor.Motor_Data.anglev = left_xc5000_motor.Encoder_Filter.out;
						left_xc5000_motor.Inner_Pid.fpFB = left_xc5000_motor.Motor_Data.anglev;
						
						system_monitor.motor_left_xc5000_cnt++;
						break;
						
					case 0x202:
						right_xc5000_motor.Motor_Data.angle = GetAngle_V6(&RxHeader, RxMsg_CAN2) * RADIAN;
						if(right_xc5000_motor.Motor_Data.flag_init == 0)
						{
							right_xc5000_motor.Motor_Data.init_angle = right_xc5000_motor.Motor_Data.angle;
							right_xc5000_motor.Motor_Data.flag_init = 1;
						}
						right_xc5000_motor.Real_Delta_Pos = right_xc5000_motor.Motor_Data.angle - right_xc5000_motor.Motor_Data.init_angle;
						right_xc5000_motor.Outer_Pid.fpFB = right_xc5000_motor.Real_Delta_Pos;
						
						right_xc5000_motor.Encoder_Filter.in = GetSpeed_V6(&RxHeader, RxMsg_CAN2);
						LpFilter(&right_xc5000_motor.Encoder_Filter);
						right_xc5000_motor.Motor_Data.anglev = right_xc5000_motor.Encoder_Filter.out;
						right_xc5000_motor.Inner_Pid.fpFB = right_xc5000_motor.Motor_Data.anglev;
						
						system_monitor.motor_right_xc5000_cnt++;
						break;
						
					case 0x203:
						left_4219_motor.Motor_Data.angle = GetAngle_V6(&RxHeader, RxMsg_CAN2) * RADIAN;
						if(left_4219_motor.Motor_Data.flag_init == 0)
						{
							left_4219_motor.Motor_Data.init_angle = left_4219_motor.Motor_Data.angle;
							left_4219_motor.Motor_Data.flag_init = 1;
						}
						left_4219_motor.Real_Delta_Pos = left_4219_motor.Motor_Data.angle - left_4219_motor.Motor_Data.init_angle;
						left_4219_motor.Outer_Pid.fpFB = left_4219_motor.Real_Delta_Pos;
						
						left_4219_motor.Encoder_Filter.in = GetSpeed_V6(&RxHeader, RxMsg_CAN2);
						LpFilter(&left_4219_motor.Encoder_Filter);
						left_4219_motor.Motor_Data.anglev = left_4219_motor.Encoder_Filter.out;
						left_4219_motor.Inner_Pid.fpFB = left_4219_motor.Motor_Data.anglev;
						
						system_monitor.motor_left_4219_cnt++;
						break;
						
					case 0x204:
						right_4219_motor.Motor_Data.angle = GetAngle_V6(&RxHeader, RxMsg_CAN2) * RADIAN;
						if(right_4219_motor.Motor_Data.flag_init == 0)
						{
							right_4219_motor.Motor_Data.init_angle = right_4219_motor.Motor_Data.angle;
							right_4219_motor.Motor_Data.flag_init = 1;
						}
						right_4219_motor.Real_Delta_Pos = right_4219_motor.Motor_Data.angle - right_4219_motor.Motor_Data.init_angle;
						right_4219_motor.Outer_Pid.fpFB = right_4219_motor.Real_Delta_Pos;
						
						right_4219_motor.Encoder_Filter.in = GetSpeed_V6(&RxHeader, RxMsg_CAN2);
						LpFilter(&right_4219_motor.Encoder_Filter);
						right_4219_motor.Motor_Data.anglev = right_4219_motor.Encoder_Filter.out;
						right_4219_motor.Inner_Pid.fpFB = right_4219_motor.Motor_Data.anglev;
						
						system_monitor.motor_right_4219_cnt++;
						break;
						
					case 0x356:
						memcpy(&degreeA,&RxMsg_CAN2[0],4);
						memcpy(&degreeB,&RxMsg_CAN2[4],4);
						
						break;
					default:
							break;
        }
    }
}
