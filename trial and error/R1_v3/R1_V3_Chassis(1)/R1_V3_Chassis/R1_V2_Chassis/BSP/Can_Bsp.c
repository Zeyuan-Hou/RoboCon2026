#include "Can_Bsp.h"


//创建两个CAN的接收BUF
uint8_t RxMsg_CAN1[8];  
uint8_t RxMsg_CAN2[8]; 


//CAN初始化函数
void can1_start(void) 
{
    CAN1_FILTER_CONFIG(&hcan1); //滤波器初始化
    HAL_CAN_Start(&hcan1); //启动CAN1
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING); //使能中断

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



























//下面这个是给航模电机用的，搭配自研电调
void motor_SendCurent(uint32_t CANID, int16_t vol1, int16_t vol2, int16_t vol3, int16_t vol4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];

  tx_header.StdId = CANID;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (vol1>>8)&0xff;
  tx_data[1] =    (vol1)&0xff;
  tx_data[2] = (vol2>>8)&0xff;
  tx_data[3] =    (vol2)&0xff;
  tx_data[4] = (vol3>>8)&0xff;
  tx_data[5] =    (vol3)&0xff;
  tx_data[6] = (vol4>>8)&0xff;
  tx_data[7] =    (vol4)&0xff;

  if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
  {
    if(HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_data,(uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_data,(uint32_t *)CAN_TX_MAILBOX2);
    }
  } 
	system_monitor.can_send_cnt_chassis++;
}





























//下面两个函数合起来是控制3508用的
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
void CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len)
{
	CAN_TxHeaderTypeDef Tx_Header;
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=Len;
	if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}

































void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if(hcan->Instance == CAN2)
    {
			  system_monitor.can_rec_cnt[1]++;   
        CAN_RxHeaderTypeDef RxHeader;                                           
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN2);  
        switch (RxHeader.StdId)   //2006,3508:0x200 ; 6020:0x204
        {
					
        case 0x203:
          Motor_GetFeedback(&rightup_motor,RxMsg_CAN2);
				  chassis_run.rightup.fpFB = rightup_motor.anglev;
				
				  system_monitor.motor_RU_cnt++;
          break;
				case 0x204:
          Motor_GetFeedback(&rightdown_motor,RxMsg_CAN2);
				  chassis_run.rightdown.fpFB = rightdown_motor.anglev;
				
				  system_monitor.motor_RD_cnt++;
          break;
				case 0x201:
          Motor_GetFeedback(&leftdown_motor,RxMsg_CAN2);
				  chassis_run.leftdown.fpFB = leftdown_motor.anglev;
				
				  system_monitor.motor_LD_cnt++;
          break;
				case 0x202://0x200 + stMotorCtrl.uiSlaveID
          Motor_GetFeedback(&leftup_motor,RxMsg_CAN2);
				  chassis_run.leftup.fpFB = leftup_motor.anglev;//其实上一行的代码只是给这个FB赋值的中间量罢了
				
				  system_monitor.motor_LU_cnt++;
				break;
				default:
					break;

        }
		}
		if (hcan->Instance == CAN1)
    {
        system_monitor.can_rec_cnt[0]++;                                       
        CAN_RxHeaderTypeDef RxHeader;                                           
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN1);        
        switch (RxHeader.StdId)
        {        
		      


				
									

					
					
					case 0x356://处理随动轮的数据
					memcpy(&degreeA,&RxMsg_CAN1[0],4);
					memcpy(&degreeB,&RxMsg_CAN1[4],4);
					system_monitor.flw_cnt++;
					  break;
					
					case 0x201:
						
					//起重机3508_1
					Crane_3508_1.EncoderNum = GetEncoderNumber_(&Crane_3508_1,RxMsg_CAN1);
					Crane_3508_1.encoder_speed = GetSpeed(&RxHeader,RxMsg_CAN1);
					Abs_Encoder_Process(&Crane_3508_1.motor_encoder, Crane_3508_1.EncoderNum);
					Crane_3508_1.angle=Crane_3508_1.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio  ;
          Crane_3508_1.anglev=Crane_3508_1.encoder_speed/(float)M3508_uiGearRatio   ; // r/min
					system_monitor.Crane_3508_cnt[0]++;
						break;
	       case 0x202:
						
				 //起重机3508_2
					Crane_3508_2.EncoderNum = GetEncoderNumber_(&Crane_3508_2,RxMsg_CAN1);
					Crane_3508_2.encoder_speed = GetSpeed(&RxHeader,RxMsg_CAN1);
					Abs_Encoder_Process(&Crane_3508_2.motor_encoder, Crane_3508_2.EncoderNum);
					Crane_3508_2.angle=Crane_3508_2.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio  ;
          Crane_3508_2.anglev=Crane_3508_2.encoder_speed/(float)M3508_uiGearRatio   ; // r/min
					system_monitor.Crane_3508_cnt[1]++;
						break;
				 case 0x210: // 气动板DT35数据
            DT35_GET_Data(RxMsg_CAN1);//把数据赋值进入DT35_Data数组
				 
				 //初步获取未解包的DT35数据
						DT35_dis.Num_1 = (DT35_Data[1]<<8) | DT35_Data[0];
				    DT35_dis.Num_2 = (DT35_Data[3]<<8) | DT35_Data[2];
				    DT35_dis.Num_3 = (DT35_Data[5]<<8) | DT35_Data[4];
				    DT35_dis.Num_4 = (DT35_Data[7]<<8) | DT35_Data[6];
					
					//DT35的线性拟合
						DT35_distance.Num_1 = DT35_dis.Num_1*0.74-23.735;
						DT35_distance.Num_2 = DT35_dis.Num_2*0.693+39.641;
						DT35_distance.Num_3 = DT35_dis.Num_3*1.0005-132.26-19;
						DT35_distance.Num_4 = DT35_dis.Num_4*0.9988-134.53-20;
				 
            system_monitor.AirBoard_Receive_cnt++;
          default:
            break;
				}
			}
}











void Abs_Encoder_Process(ST_ENCODER *motor_encoder,FP32 encoder_pos)//3508用的
{
  motor_encoder->siPreRawValue = motor_encoder->siRawValue;
  motor_encoder->siRawValue = encoder_pos;
  motor_encoder->siDiff = motor_encoder->siRawValue - motor_encoder->siPreRawValue;
  if(motor_encoder->siDiff < -motor_encoder->siNumber/2)
    motor_encoder->siDiff += motor_encoder->siNumber;
  else if(motor_encoder->siDiff > motor_encoder->siNumber/2)
    motor_encoder->siDiff -= motor_encoder->siNumber;
  motor_encoder->siSumValue += motor_encoder->siDiff;
	
	  if(motor_encoder->state == 1)//上电时将线数清零
  {
      motor_encoder->siSumValue = 0;
      motor_encoder->state = 0;
  }
}

float GetEncoderNumber_(ST_MOTORT* motor,uint8_t msg[8])//3508用的
{
    motor->EncoderNum=(msg[0]<<8)|(msg[1]);
    return motor->EncoderNum;
}

float GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8])//3508用的
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















void Motor_GetFeedback(MOTOR *motor, uint8_t rx_data[8])//航模电机用的
{
 
  memcpy(&motor->angle,&rx_data[0],4);
  memcpy(&motor->anglev,&rx_data[4],4);
  
}



