#include "CAN_Bsp.h"
#include "J60_motor.h"
#include "DJI_motor.h"

void CAN_INIT(void) 
{
    bsp_fdcan1_init(); //过滤器初始化
		bsp_fdcan2_init();
		bsp_fdcan3_init();
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
	FDCAN_RXFilter.FilterID1=0x000;                         //32位ID1
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
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
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
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
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
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
}
/******************************************************************
* @brief FDCAN发送函数
 *
 * @brief函数使用方法备注
 *
 * @param can设备句柄
 *
 * @return
 *******************************************************************/


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

/******************************************************************
 * @brief FDCAN 接收回调函数
 *
 * @brief 
 *
 * @param
 *
 * @return
 *******************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        // 使用传入的 hfdcan 句柄自动读取对应的硬件 FIFO
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 经典 CAN 长度转换 (针对你的 1 字节测试)
            //uint32_t len = RxHeader.DataLength >> 16; 
//						uint32_t len = RxHeader.DataLength;
			
            // 根据 Instance 指针识别是哪路 CAN
            if (hfdcan->Instance == FDCAN1) {
								memcpy(CAN1_RxBuf,RxData,8);
								switch(RxHeader.Identifier){
									case 0x201:
										move2006.EncoderNum=GetEncoderNumber(&move2006,CAN1_RxBuf);
										move2006.encoder_speed=GetSpeed(CAN1_RxBuf);
										move2006.motor_current=GetCurrent(CAN1_RxBuf);
										Abs_Encoder_Process(&move2006.motor_encoder,move2006.EncoderNum);
										move2006.angle=move2006.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - move2006.Start_Pos;
										if(!move2006.getStartPos){
											move2006.Start_Pos = move2006.angle;
											move2006.getStartPos = 1;
										}
										move2006.anglev=move2006.encoder_speed/(float)M2006_uiGearRatio;
										systemMonitor.cntMonitor.Move_2006++;
										break;
									case 0x202:
										claw3508.EncoderNum=GetEncoderNumber(&claw3508,CAN1_RxBuf);
										claw3508.encoder_speed=GetSpeed(CAN1_RxBuf);
										claw3508.motor_current=GetCurrent(CAN1_RxBuf);
										Abs_Encoder_Process(&claw3508.motor_encoder,claw3508.EncoderNum);
										claw3508.angle=claw3508.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio - claw3508.Start_Pos;
										if(!claw3508.getStartPos){
											claw3508.Start_Pos = claw3508.angle;
											claw3508.getStartPos = 1;
										}
										claw3508.anglev=claw3508.encoder_speed/(float)M3508_uiGearRatio;
										systemMonitor.cntMonitor.Claw_3508++;
										break;
									case 0x203:
										friction3508.EncoderNum=GetEncoderNumber(&friction3508,CAN1_RxBuf);
										friction3508.encoder_speed=GetSpeed(CAN1_RxBuf);
										friction3508.motor_current=GetCurrent(CAN1_RxBuf);
										Abs_Encoder_Process(&friction3508.motor_encoder,friction3508.EncoderNum);
										friction3508.angle=friction3508.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio - friction3508.Start_Pos;
										if(!friction3508.getStartPos){
											friction3508.Start_Pos = friction3508.angle;
											friction3508.getStartPos = 1;
										}
										friction3508.anglev=friction3508.encoder_speed/(float)M3508_uiGearRatio;
										systemMonitor.cntMonitor.Friction_3508++;
										break;
									case 0x204:
										wrist3508.EncoderNum=GetEncoderNumber(&wrist3508,CAN1_RxBuf);
										wrist3508.encoder_speed=GetSpeed(CAN1_RxBuf);
										wrist3508.motor_current=GetCurrent(CAN1_RxBuf);
										Abs_Encoder_Process(&wrist3508.motor_encoder,wrist3508.EncoderNum);
										wrist3508.angle=wrist3508.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio - wrist3508.Start_Pos;
										if(!wrist3508.getStartPos){
											wrist3508.Start_Pos = wrist3508.angle;
											wrist3508.getStartPos = 1;
										}
										wrist3508.anglev=wrist3508.encoder_speed/(float)M3508_uiGearRatio;
										systemMonitor.cntMonitor.Wrist_3508++;
										break;
									case 0x91:
										J60_ReceiveData(&gimbalJ60,&gimbalJ60Receive,CAN1_RxBuf);
										systemMonitor.cntMonitor.Gimbal_J60++;
										break;
									default:
										break;
								}
                    
            }
            else if (hfdcan->Instance == FDCAN2) {
								memcpy(CAN2_RxBuf,RxData,8);
                switch(RxHeader.Identifier){
									case 0x201:
										platform_L2006.EncoderNum=GetEncoderNumber(&platform_L2006,CAN2_RxBuf);
										platform_L2006.encoder_speed=GetSpeed(CAN2_RxBuf);
										platform_L2006.motor_current=GetCurrent(CAN2_RxBuf);
										Abs_Encoder_Process(&platform_L2006.motor_encoder,platform_L2006.EncoderNum);
										platform_L2006.angle=platform_L2006.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - platform_L2006.Start_Pos;
										if(!platform_L2006.getStartPos){
											platform_L2006.Start_Pos = platform_L2006.angle;
											platform_L2006.getStartPos = 1;
										}
										platform_L2006.anglev=platform_L2006.encoder_speed/(float)M2006_uiGearRatio;
										systemMonitor.cntMonitor.Platform_L2006++;
										break;
									case 0x202:
										platform_R2006.EncoderNum=GetEncoderNumber(&platform_R2006,CAN2_RxBuf);
										platform_R2006.encoder_speed=GetSpeed(CAN2_RxBuf);
										platform_R2006.motor_current=GetCurrent(CAN2_RxBuf);
										Abs_Encoder_Process(&platform_R2006.motor_encoder,platform_R2006.EncoderNum);
										platform_R2006.angle=platform_R2006.motor_encoder.siSumValue/(float)8192*360.f/(float)M2006_uiGearRatio - platform_R2006.Start_Pos;
										if(!platform_R2006.getStartPos){
											platform_R2006.Start_Pos = platform_R2006.angle;
											platform_R2006.getStartPos = 1;
										}
										platform_R2006.anglev=platform_R2006.encoder_speed/(float)M2006_uiGearRatio;
										systemMonitor.cntMonitor.Platform_R2006++;
										break;
									case 0x91:
										J60_ReceiveData(&shoulderJ60,&shoulderJ60Receive,CAN2_RxBuf);
										systemMonitor.cntMonitor.Shoulder_J60++;
										break;
									case 0x92:
										J60_ReceiveData(&elbowJ60,&elbowJ60Receive,CAN2_RxBuf);
										systemMonitor.cntMonitor.Elbow_J60++;
										break;
									default:
										break;
								}
                    
            }
            else if (hfdcan->Instance == FDCAN3) {
								memcpy(CAN3_RxBuf,RxData,8);
                switch(RxHeader.Identifier){
									case 0x202:
										friction3508_sub.EncoderNum=GetEncoderNumber(&friction3508_sub,CAN3_RxBuf);
										friction3508_sub.encoder_speed=GetSpeed(CAN3_RxBuf);
//										friction3508_sub.motor_current=GetCurrent(CAN3_RxBuf);
										Abs_Encoder_Process(&friction3508_sub.motor_encoder,friction3508_sub.EncoderNum);
										friction3508_sub.angle=friction3508_sub.motor_encoder.siSumValue/(float)8192*360.f/(float)M3508_uiGearRatio - friction3508_sub.Start_Pos;
										if(!friction3508_sub.getStartPos){
											friction3508_sub.Start_Pos = friction3508_sub.angle;
											friction3508_sub.getStartPos = 1;
										}
										friction3508_sub.anglev=friction3508_sub.encoder_speed/(float)M3508_uiGearRatio;
										break;
									case 0x210:
										Dt35_DataReceive(&airOperator,RxData);
										systemMonitor.cntMonitor.AirOperater++;
										break;
									case 0x220:
										left_limit=CAN3_RxBuf[0];
										right_limit=CAN3_RxBuf[1];
										systemMonitor.cntMonitor.AirOperater++;
									default:
										break;
								}  
            }
        }
    }
}

//bus-off 修复实现函数
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_ProtocolStatusTypeDef protocolStatus = {};
    HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
    if (protocolStatus.BusOff) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    if (hfdcan == &hfdcan1) {
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
            CAN_bus_off_check_reset(hfdcan);
        }
    }
		if (hfdcan == &hfdcan2) {
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
            CAN_bus_off_check_reset(hfdcan);
        }
    }
		if (hfdcan == &hfdcan3) {
        if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
            CAN_bus_off_check_reset(hfdcan);
        }
    }
}
//DJI_Control
void CAN_Sendcurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4)
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

	CAN_SendStdData(hfdcan,id,TxCurrent,8);
}



