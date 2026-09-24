#include "CAN_BSP.h"


void CAN_INIT(void) 
{
  bsp_fdcan1_init(); //过滤器初始化
	bsp_fdcan2_init();
	bsp_fdcan3_init();
	
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
}


/******************************************************************
 * @brief 初始化配置FDCAN1 各项功能函数，开启FDCAN 收发
 *
 * @brief 过滤器模式为 掩码模式 0x0000 - 0xffff 绑定过滤参数到fifo0
 *
 * @param
 *
 * @return
 *******************************************************************/
void bsp_fdcan1_init(void)
{
	/* 配置RX滤波器 */
	FDCAN_FilterTypeDef FDCAN_RXFilter={0};  
	FDCAN_RXFilter.IdType=FDCAN_STANDARD_ID;                //标准ID
	FDCAN_RXFilter.FilterIndex=0;                           //过滤器索引                   
	FDCAN_RXFilter.FilterType=FDCAN_FILTER_MASK;            //过滤器类型
	FDCAN_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;    //过滤器0关联到FIFO0  
	FDCAN_RXFilter.FilterID1=0x0000;                         //32位ID1
	FDCAN_RXFilter.FilterID2=0x0000;                         //32位ID2
 if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_RXFilter) != HAL_OK)
	{
		Error_Handler();
	}
 if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
 /* Start the FDCAN module */
 if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* 开启CAN 收发中断 */
 if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
			Error_Handler();
	}
}
/******************************************************************
 * @brief 初始化配置FDCAN1 各项功能函数，开启FDCAN 收发
 *
 * @brief 过滤器模式为 掩码模式 0x0000 - 0xffff 绑定过滤参数到fifo1
 *
 * @param
 *
 * @return
 *******************************************************************/
void bsp_fdcan2_init(void)
{
	/* 配置RX滤波器 */
	FDCAN_FilterTypeDef FDCAN_RXFilter={0};  
	FDCAN_RXFilter.IdType=FDCAN_STANDARD_ID;                //标准ID
	FDCAN_RXFilter.FilterIndex=0;                           //过滤器索引                   
	FDCAN_RXFilter.FilterType=FDCAN_FILTER_MASK;            //过滤器类型
	FDCAN_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;    //过滤器1关联到FIFO0
	FDCAN_RXFilter.FilterID1=0x0000;                         //32位ID1
	FDCAN_RXFilter.FilterID2=0x0000;                         //32位ID2
 if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_RXFilter) != HAL_OK)
	{
		Error_Handler();
	}
 if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
 /* Start the FDCAN module */
 if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
	/* 开启CAN 收发中断 */
 if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
			Error_Handler();
	}
}

void bsp_fdcan3_init(void)
{

	/* 配置RX滤波器 */
	FDCAN_FilterTypeDef FDCAN_RXFilter={0};  
	FDCAN_RXFilter.IdType=FDCAN_STANDARD_ID;                //标准ID
	FDCAN_RXFilter.FilterIndex=0;                           //过滤器索引                   
	FDCAN_RXFilter.FilterType=FDCAN_FILTER_MASK;            //过滤器类型
	FDCAN_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;    //过滤器1关联到FIFO0
	FDCAN_RXFilter.FilterID1=0x0000;                         //32位ID1
	FDCAN_RXFilter.FilterID2=0x0000;                         //32位ID2
 if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_RXFilter) != HAL_OK)
	{
		Error_Handler();
	}
 if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
 /* Start the FDCAN module */
 if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{
		Error_Handler();
	}
	/* 开启CAN 收发中断 */
 if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
			Error_Handler();
	}
}



	
void CAN_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier=ID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength=Len;

	
	
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
	
	while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0); // 等待有发送邮箱可用
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);
}



void CAN_SendCurrent(FDCAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
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
    CAN_SendStdData(hcan, id, TxCurrent, 8);
}


float GetSpeed(uint8_t msg[8])
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


float GetEncoderNumber(ST_MOTOR* motor,uint8_t msg[8])
{
	motor->EncoderNum=(msg[0]<<8)|(msg[1]);
	return motor->EncoderNum;
}


void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value)
{
	
	    if(pEncoder->state == 0) 
    {
        pEncoder->siPreRawValue = value;
        pEncoder->siRawValue = value;
        pEncoder->siDiff = 0;
        pEncoder->siSumValue = 0;
        pEncoder->state = 1;  // 标记为已初始化
        return;
    }
	
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



float GetSpeed_V6(uint8_t msg[8])
{
    float speed_temp;
		memcpy(&speed_temp,&msg[4],4);
    return speed_temp;
}


float GetAngle_V6_test(uint8_t msg[8],Angle_Processor *pangle)
{
		float angle_temp;
		memcpy(&angle_temp,&msg[0],4);
	
	    // 初始化
    if(!pangle->init) {
        pangle->pre_angle = angle_temp;
        pangle->total_angle = 0;
        pangle->init = 1;
        return pangle->total_angle;
    }
	
	// 计算差值并处理过零
    float diff = angle_temp - pangle->pre_angle;
    
    while(diff > 180.0f)
        diff -= 360.0f;
    
    while(diff < -180.0f)
        diff += 360.0f;
    
    // 累加总角度
    pangle->total_angle += diff;
    pangle->pre_angle = angle_temp;
   
    return pangle->total_angle;
}


//解算行程开关
void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states)
{
    for (uint8_t i = 0; i < 8; i++){
        key_states[i] = (travel_switch >> i) & 0x01;
    }
}


//电磁阀
uint8_t bin_array_to_u8(uint8_t *bits) {  // 用于存储最终转换结果的整型变量
    uint8_t result = 0;
    // 遍历二进制数组的每一位
    for (uint8_t i = 0; i < 6; i++) {
        // 每个位对应的权值为 2^i（因为第0位是最低位）
        result += bits[i] * (1 << i);  // 将当前位的值乘以对应的权值，并累加到结果中
    }
    return result;  // 返回转换后的整数值
}



uint8_t Can_rxBuffer1[BUFFER_SIZE]={0};//接收数组
uint8_t Can_rxBuffer2[BUFFER_SIZE]={0};//接收数组
uint8_t Can_rxBuffer3[BUFFER_SIZE]={0};//接收数组

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        // 使用传入的 hfdcan 句柄自动读取对应的硬件 FIFO
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
			
            // 根据 Instance 指针识别是哪路 CAN
            if (hfdcan->Instance == FDCAN1) {
							switch(RxHeader.Identifier)			
			{
									
					case 0x220:					
						memcpy(&travel_switch, RxData, sizeof(uint8_t));
						TravelSwitchDataDeal(travel_switch, travel_switch_mode);//每个行程开关遮挡时返回值不一样，待测
						monitor.rate_cnt.key++;
						break;
					
					case 0x210:
						DT35_temp.Num_1 = (RxData[1]<<8) | RxData[0];  //车左边
						DT35_temp.Num_2 = (RxData[3]<<8) | RxData[2];  //车右边
						DT35_temp.Num_3 = (RxData[5]<<8) | RxData[4];  //车后面
						DT35_temp.Num_4 = (RxData[7]<<8) | RxData[6];
						

						DT35_fact.Num_1 = 0.2948 * DT35_temp.Num_1 + 3.9219;  
						DT35_fact.Num_2 = 0.3673 * DT35_temp.Num_2 + 33.1505;  
						DT35_fact.Num_3 = 0.3848 * DT35_temp.Num_3 + 13.0859;  
 					
					
						DT35_Y_fact = DT35_fact.Num_2;  //红场的Y为右边
						DT35_X_fact = DT35_fact.Num_3;  //X为车后边
					
						monitor.rate_cnt.DT35++;
						break;
								
					case 0x201:                        
						DJI_MOTOR.DJI_1.EncoderNum = GetEncoderNumber(&DJI_MOTOR.DJI_1,RxData);
						DJI_MOTOR.DJI_1.encoder_speed = GetSpeed(RxData); 					 
						Abs_Encoder_Process(&DJI_MOTOR.DJI_1.motor_encoder,DJI_MOTOR.DJI_1.EncoderNum);
						DJI_MOTOR.DJI_1.angle=DJI_MOTOR.DJI_1.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio;
						DJI_MOTOR.DJI_1.anglev=DJI_MOTOR.DJI_1.encoder_speed/(float)M2006_uiGearRatio;
						dji_run.DJI_1.fpFB = DJI_MOTOR.DJI_1.anglev;
						monitor.rate_cnt.DJI_201++;	
						break;
			
					case 0x202:
						DJI_MOTOR.DJI_2.EncoderNum = GetEncoderNumber(&DJI_MOTOR.DJI_2,RxData);
						DJI_MOTOR.DJI_2.encoder_speed = GetSpeed(RxData); 					 
						Abs_Encoder_Process(&DJI_MOTOR.DJI_2.motor_encoder,DJI_MOTOR.DJI_2.EncoderNum);
						DJI_MOTOR.DJI_2.angle=DJI_MOTOR.DJI_2.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio;
						DJI_MOTOR.DJI_2.anglev=DJI_MOTOR.DJI_2.encoder_speed/(float)M2006_uiGearRatio;
						dji_run.DJI_2.fpFB = DJI_MOTOR.DJI_2.anglev;
						monitor.rate_cnt.DJI_202++;
						break;
			
        default:
						break;
			}
				}
          
												
             if (hfdcan->Instance == FDCAN2) {
								switch(RxHeader.Identifier)

			{			
					case 0x201:                        
					  motor_wheel.wheel_1.angle = GetAngle_V6_test(RxData,&motor_wheel.wheel_1.process);
						motor_wheel.wheel_1.anglev = GetSpeed_V6( RxData);
						chassis_run.wheel_1.fpFB = motor_wheel.wheel_1.anglev;
						monitor.rate_cnt.wheel_201++;						 
						break;
				 
					case 0x202:
						motor_wheel.wheel_2.angle = GetAngle_V6_test(RxData,&motor_wheel.wheel_2.process);
						motor_wheel.wheel_2.anglev = GetSpeed_V6(RxData);
						chassis_run.wheel_2.fpFB = motor_wheel.wheel_2.anglev;
						monitor.rate_cnt.wheel_202++;			
						break;
					
					case 0x203:
						motor_wheel.wheel_3.angle =  GetAngle_V6_test(RxData,&motor_wheel.wheel_3.process);
						motor_wheel.wheel_3.anglev = GetSpeed_V6(RxData);
						chassis_run.wheel_3.fpFB = motor_wheel.wheel_3.anglev;
						monitor.rate_cnt.wheel_203++;					
						break;
					
					case 0x204:
						motor_wheel.wheel_4.angle =  GetAngle_V6_test(RxData,&motor_wheel.wheel_4.process);
						motor_wheel.wheel_4.anglev =  GetSpeed_V6(RxData);
						chassis_run.wheel_4.fpFB = motor_wheel.wheel_4.anglev;
						monitor.rate_cnt.wheel_204++;					
						break;					
					
		
			default:
					break;
		}							
	 }	 
	}
 }
}


/******************************************************************
 * @brief can BusOff 恢复函数
 *
 * @brief 
 *
 * @param
 *
 * @return
 *******************************************************************/
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_ProtocolStatusTypeDef protocolStatus = {0};
    // 1. 获取当前协议状态（包含你之前问的那个结构体）
    HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
    
    // 2. 如果发现硬件已经处于 BusOff 状态
    if (protocolStatus.BusOff) {
        // 3. 核心：清除 INIT 位，通知硬件开始执行总线恢复
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

