#include "CANBsp.h"
#include "can.h"
#include "Robot.h"
#include "dm_motor.h"
#include "J60_motor.h"
#include "LK_motor.h"
//CAN初始化函数
void can1_start(void) 
{
    CAN1_FILTER_CONFIG(&hcan1); //滤波器初始化
    HAL_CAN_Start(&hcan1); //启动CAN1
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING); //使能中断
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
 * @brief 配过滤器
 * @param CAN1
 * @retval None
 */
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan) 
{
		CAN_FilterTypeDef CAN_FilterConfigStructure;
    CAN_FilterConfigStructure.FilterBank = 0;// 使用过滤器组0
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;// 掩码模式
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;// 32位模式
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;// 过滤器ID高16位
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;// 过滤器掩码高16位
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;// 过滤器ID低16位
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;// 过滤器掩码低16位
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;// 匹配的消息存入FIFO0
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;// 从过滤器组14开始分配给CAN2(双CAN时)
    CAN_FilterConfigStructure.FilterActivation = ENABLE;// 启用此过滤器
		
    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can2_start(void) 
{
    CAN2_FILTER_CONFIG(&hcan2); //滤波器初始化
    HAL_CAN_Start(&hcan2); //启动CAN2
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING); //使能中断
	  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
 * @brief 配过滤器
 * @param CAN2
 * @retval None
 */
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan) 
{
	CAN_FilterTypeDef CAN_FilterConfigStructure;
    CAN_FilterConfigStructure.FilterBank = 14;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 28;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
		
    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t             rx_data[8];
    if(hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef RxHeader;     
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN1_RxBuf);  
        switch (RxHeader.StdId)   //J60
        {
					case 0x91:
						J60_ReceiveData(&leftShoulder,&leftShoulderReceive,CAN1_RxBuf);
						system_monitor.leftShoulder_cnt++;
					break;
					case 0x92:
						J60_ReceiveData(&rightShoulder,&rightShoulderReceive,CAN1_RxBuf);
						system_monitor.rightShoulder_cnt++;
					break;
					case 0x141://LK
						LK_8016_DataProcess(CAN1_RxBuf, &stretch_LK);
            Abs_Encoder_Process(&stretch_LK.LK_motor_encoder,stretch_LK.encoder_num);
						stretch_LK.angle=stretch_LK.LK_motor_encoder.siSumValue/(float)LK_siNumber*360.f;
						stretch_LK.anglev=stretch_LK.speed;
						system_monitor.stretch_LK_cnt++;
					break;
					default:
					break;
        }
		}
		if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN2_RxBuf);
        uint32_t StdId = RxHeader.StdId;
				static fp32 preAngle1,preAngle2,preAngle3,preAngle4,preAngle5;
				static fp32 preAnglev1,preAnglev2,preAnglev3,preAnglev4,preAnglev5;
        switch(StdId){
					case 0x201:
            left_2006_1.EncoderNum=GetEncoderNumber(&left_2006_1,CAN2_RxBuf);
						left_2006_1.encoder_speed=GetSpeed(&RxHeader,CAN2_RxBuf);
						Abs_Encoder_Process(&left_2006_1.motor_encoder,left_2006_1.EncoderNum);
						left_2006_1.angle=left_2006_1.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - left_2006_1.Start_Pos;
						if(!left_2006_1.getStartPos){
							left_2006_1.Start_Pos = left_2006_1.angle;
							left_2006_1.getStartPos = 1;
						}
						if(fabs(left_2006_1.angle-preAngle1)<10){
							preAngle1=left_2006_1.angle;
						}else{
							left_2006_1.angle = preAngle1;
						}
						left_2006_1.anglev=left_2006_1.encoder_speed/(float)M2006_uiGearRatio;
						if(fabs(left_2006_1.anglev-preAnglev1)<10){
							preAnglev1=left_2006_1.anglev;
						}else{
							left_2006_1.anglev = preAnglev1;
						}
						system_monitor.left_2006_1_cnt++;
						break;
					case 0x202:
            left_2006_2.EncoderNum=GetEncoderNumber(&left_2006_2,CAN2_RxBuf);
						left_2006_2.encoder_speed=GetSpeed(&RxHeader,CAN2_RxBuf);
						Abs_Encoder_Process(&left_2006_2.motor_encoder,left_2006_2.EncoderNum);
						left_2006_2.angle=left_2006_2.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - left_2006_2.Start_Pos;
						if(!left_2006_2.getStartPos){
								left_2006_2.Start_Pos = left_2006_2.angle;
								left_2006_2.getStartPos = 1;
							}
						if(fabs(left_2006_2.angle-preAngle2)<10){
							preAngle2=left_2006_2.angle;
						}else{
							left_2006_2.angle = preAngle2;
						}
						left_2006_2.anglev=left_2006_2.encoder_speed/(float)M2006_uiGearRatio;
						if(fabs(left_2006_2.anglev-preAnglev2)<10){
							preAnglev2=left_2006_2.anglev;
						}else{
							left_2006_2.anglev = preAnglev2;
						}
							system_monitor.left_2006_2_cnt++;
						break;
					case 0x203:
            right_2006_1.EncoderNum=GetEncoderNumber(&right_2006_1,CAN2_RxBuf);
						right_2006_1.encoder_speed=GetSpeed(&RxHeader,CAN2_RxBuf);
						Abs_Encoder_Process(&right_2006_1.motor_encoder,right_2006_1.EncoderNum);
						right_2006_1.angle=right_2006_1.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - right_2006_1.Start_Pos;
						if(!right_2006_1.getStartPos){
							right_2006_1.Start_Pos = right_2006_1.angle;
							right_2006_1.getStartPos = 1;
						}
						if(fabs(right_2006_1.angle-preAngle3)<10){
							preAngle3=right_2006_1.angle;
						}else{
							right_2006_1.angle = preAngle3;
						}
						right_2006_1.anglev=right_2006_1.encoder_speed/(float)M2006_uiGearRatio;
						if(fabs(right_2006_1.anglev-preAnglev3)<10){
							preAnglev3=right_2006_1.anglev;
						}else{
							right_2006_1.anglev = preAnglev3;
						}
						system_monitor.right_2006_1_cnt++;
						break;
					case 0x204:
            right_2006_2.EncoderNum=GetEncoderNumber(&right_2006_2,CAN2_RxBuf);
						right_2006_2.encoder_speed=GetSpeed(&RxHeader,CAN2_RxBuf);
						Abs_Encoder_Process(&right_2006_2.motor_encoder,right_2006_2.EncoderNum);
						right_2006_2.angle=right_2006_2.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - right_2006_2.Start_Pos;
						if(!right_2006_2.getStartPos){
							right_2006_2.Start_Pos = right_2006_2.angle;
							right_2006_2.getStartPos = 1;
						}
						if(fabs(right_2006_2.angle-preAngle4)<10){
							preAngle4=right_2006_2.angle;
						}else{
							right_2006_2.angle = preAngle4;
						}
						right_2006_2.anglev=right_2006_2.encoder_speed/(float)M2006_uiGearRatio;
						if(fabs(right_2006_2.anglev-preAnglev4)<10){
							preAnglev4=right_2006_2.anglev;
						}else{
							right_2006_2.anglev = preAnglev4;
						}
						system_monitor.right_2006_2_cnt++;
						break;
					case 0x205:
            stretch_2006.EncoderNum=GetEncoderNumber(&stretch_2006,CAN2_RxBuf);
						stretch_2006.encoder_speed=GetSpeed(&RxHeader,CAN2_RxBuf);
						Abs_Encoder_Process(&stretch_2006.motor_encoder,stretch_2006.EncoderNum);
						stretch_2006.angle=stretch_2006.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - stretch_2006.Start_Pos;
						if(!stretch_2006.getStartPos){
							stretch_2006.Start_Pos = stretch_2006.angle;
							stretch_2006.getStartPos = 1;
						}
						if(fabs(stretch_2006.angle-preAngle5)<10){
							preAngle5=stretch_2006.angle;
						}
						stretch_2006.anglev=stretch_2006.encoder_speed/(float)M2006_uiGearRatio;
						if(fabs(stretch_2006.anglev-preAnglev5)<10){
							preAnglev5=stretch_2006.anglev;
						}
						system_monitor.stretch_2006_cnt++;
						break;
					case 0x11:
						dm_motor_fbdata(&stretch_DM,CAN2_RxBuf);
						system_monitor.stretch_DM_cnt++;
						break;
					default:
						break;
				}
    }
}

void CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len){
	static CAN_TxHeaderTypeDef Tx_Header;
	Tx_Header.StdId = ID;
	Tx_Header.ExtId = 0;
	Tx_Header.IDE = CAN_ID_STD;
	Tx_Header.RTR = CAN_RTR_DATA;
	Tx_Header.DLC = Len;
	if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}

void CAN_Sendcurrent(CAN_HandleTypeDef *hcan, uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4)
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=(current1>>8);
	TxCurrent[1]= current1;
	TxCurrent[2]=(current2>>8);
	TxCurrent[3]= current2;
	TxCurrent[4]=(current3>>8);
	TxCurrent[5]= current3;
	TxCurrent[6]=(current4>>8);
	TxCurrent[7]= current4;

	CANx_SendstdData(hcan,id,TxCurrent,8);
}
void CAN_Tx_GasValue(CAN_HandleTypeDef *hcan, uint8_t *gas_value)
{
    uint8_t Tx[1] = {0};
    for (int i = 0; i < 6; i++)
    {
        if(gas_value[i]) Tx[0] += pow(2, i);
    }
    CANx_SendstdData(hcan, 0x300, Tx, 1);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	 if(hcan->Instance == CAN1){
		 CANx_SendstdData(&hcan1,0x300,AirOperaterCtrl,1);
	 }
}

/*****3508电机数据处理**********/
float GetEncoderNumber(ST_DJI_MOTOR* motor,uint8_t msg[8])
{
	motor->EncoderNum=(msg[0]<<8)|(msg[1]);
	return motor->EncoderNum;
}

float GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value=0xFFFF;
	if(msg[2]&0x01<<7)
	{
		speed_temp=(base_value<<16|msg[2]<<8|msg[3]);
	}
	else
	{
		speed_temp=(msg[2]<<8)|(msg[3]);
	}
	return speed_temp;
}

float GetCurrent(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value=0xFFFF;
	if(msg[4]&0x01<<7)
	{
		speed_temp=(base_value<<16|msg[4]<<8|msg[5]);
	}
	else
	{
		speed_temp=(msg[4]<<8)|(msg[5]);
	}
	return speed_temp;
}

void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value)
{
	pEncoder->siPreRawValue=pEncoder->siRawValue;
	pEncoder->siRawValue=value;
	pEncoder->siDiff=pEncoder->siRawValue-pEncoder->siPreRawValue;
	if(pEncoder->siDiff>(pEncoder->siNumber)/2)
	{
		pEncoder->siDiff-=pEncoder->siNumber;
	}
	else if(pEncoder->siDiff<-(pEncoder->siNumber)/2)
	{
		pEncoder->siDiff+=pEncoder->siNumber;
	}
	pEncoder->siSumValue+=pEncoder->siDiff;
	
}
