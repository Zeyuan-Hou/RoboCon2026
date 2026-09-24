#include "air_operated.h"

//气动板相关全局变量
uint32_t g_uiAirValve=0;	   //气缸有关全局变量
uint32_t g_uiAirValve_Plus=0;	   //气缸有关全局变量
uint32_t g_uiAirValvePre=0;  //气缸上一次状态
uint16_t g_usSwitch=0;     //开关有关全局变量
uint16_t g_usSwitchPre = 0;//保存上一时刻开关值

uint16_t g_usSwitch1=0;     //开关有关全局变量1
uint16_t g_usSwitchPre1 = 0;//保存上一时刻开关值1


uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
  uint8_t transmit_mailbox = 0;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
  assert_param(IS_CAN_RTR(TxMessage->RTR));
  assert_param(IS_CAN_DLC(TxMessage->DLC));
  /* Select one empty transmit mailbox */
  if ((CANx->TSR&TSR_TME0) == TSR_TME0)
  {
    transmit_mailbox = 0;
  }
  else if ((CANx->TSR&TSR_TME1) == TSR_TME1)
  {
    transmit_mailbox = 1;
  }
  else if ((CANx->TSR&TSR_TME2) == TSR_TME2)
  {
    transmit_mailbox = 2;
  }
  else
  {
    transmit_mailbox = CAN_NO_MB;
  }
  if (transmit_mailbox != CAN_NO_MB)
  {
    /* Set up the Id */
    CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
    if (TxMessage->IDE == CAN_ID_STD)
    {
      assert_param(IS_CAN_STDID(TxMessage->StdId));  
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId << 21) | TxMessage->RTR);
    }
    else
    {
      assert_param(IS_CAN_EXTID(TxMessage->ExtId));
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->ExtId<<3) | TxMessage->IDE | 
                                               TxMessage->RTR);
    }
    
    /* Set up the DLC */
    TxMessage->DLC &= (uint8_t)0x0000000F;
    CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;		
	CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFEFF;		//有改动。这一行原来全是F.

    /* Set up the data field */
    CANx->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)TxMessage->Data[3] << 24) | 
                                             ((uint32_t)TxMessage->Data[2] << 16) |
                                             ((uint32_t)TxMessage->Data[1] << 8) | 
                                             ((uint32_t)TxMessage->Data[0]));
    CANx->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)TxMessage->Data[7] << 24) | 
                                             ((uint32_t)TxMessage->Data[6] << 16) |
                                             ((uint32_t)TxMessage->Data[5] << 8) |
                                             ((uint32_t)TxMessage->Data[4]));
    /* Request transmission */
    CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
  }
  return transmit_mailbox;
}
/********************************************************************************************************
气动板有关函数
*********************************************************************************************************/
/**************************************************************
函数名:SendAirMsgByCan1()
函数功能: 通过CAN1发送气缸的变量
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void SendAirMsgByCan1(uint32_t* pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0,0,0,0,0,0,0,0};
	static uint32_t s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
	*((uint32_t*)TxMessage.Data) = s_ucLastAir;
	CAN_Transmit(CAN1, &TxMessage);
}

/**************************************************************
函数名:SendAirMsgByCan2_Plus()
函数功能: 通过CAN2发送气缸的变量
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void SendAirMsgByCan2_Plus(uint32_t* pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID_Plus, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0,0,0,0,0,0,0,0};
	static uint32_t s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
	*((uint32_t*)TxMessage.Data) = s_ucLastAir;
	CAN_Transmit(CAN2, &TxMessage);
}

/**************************************************************
函数名:Send_LED_Mode(uint8_t mode)
函数功能: 全彩LED控制
输入: mode LED闪烁模式
输出：无
备注: mode :0-3    3:red  2:green 1:blue  0:默认呼吸灯
***************************************************************/
void Send_LED_Mode(uint8_t mode)
{
	static CanTxMsg TxMessage = {0x50, 0x00, CAN_ID_STD, CAN_RTR_DATA, 1, 0,0,0,0,0,0,0,0};
	static uint8_t s_ucLastAir = 0;	
	s_ucLastAir = mode;
	TxMessage.Data[0] = s_ucLastAir;
	CAN_Transmit(CAN2, &TxMessage);
}

/**************************************************************
函数名:UpdateSwitchValue()
函数功能: 更新的值
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void UpdateSwitchValue(uint16_t*pusSwitch,CanRxMsg * pRxMsg)
{
//	*pusSwitch =*((uint16_t*) &(pRxMsg->Data[0]));
		*pusSwitch = pRxMsg->Data[0]<<8 | pRxMsg->Data[1];
    //memcpy(pusSwitch,pRxMsg->Data,2);
}

/**************************************************************
函数名:SendServoMsgByCan2()
函数功能: 通过CAN2发送舵机的变量
输入: chan 要控制的舵机ID
	  value 舵机信号的数值
输出：无
备注: 
***************************************************************/
void SendServoMsgByCan2(int16_t value1, int16_t value2, int16_t value3, int16_t value4)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 8, 0,0,0,0,0,0,0,0};
//	static uint8_t s_aucLastPwm[4] = {75,75,75,75};
//	if()//值改变时才发送
//	{
//		s_aucLastPwm[chan] = value;
	  memcpy(TxMessage.Data, &value1,2 * sizeof(uint8_t));
		memcpy(TxMessage.Data+2, &value2,2 * sizeof(uint8_t));
	  memcpy(TxMessage.Data+4, &value3,2 * sizeof(uint8_t));
		memcpy(TxMessage.Data+6, &value4,2 * sizeof(uint8_t));
//		TxMessage.Data[0] = (uint8_t)value1>>8;
//		TxMessage.Data[1] = (uint8_t)value1;
//	  TxMessage.Data[2] = (uint8_t)value2>>8;
//		TxMessage.Data[3] = (uint8_t)value2;
//		TxMessage.Data[4] = (uint8_t)value3>>8;
//		TxMessage.Data[5] = (uint8_t)value3;
//	  TxMessage.Data[6] = (uint8_t)value4>>8;
//		TxMessage.Data[7] = (uint8_t)value4;
	 
		CAN_Transmit(CAN2, &TxMessage);
//		}
}
/**************************************************************
函数名:SendServoMsgByCan2_Plus()
函数功能: 通过CAN2发送舵机的变量
输入: chan 要控制的舵机ID
	  value 舵机信号的数值
输出：无
备注: 
***************************************************************/
void SendServoMsgByCan2_Plus(uint8_t chan,uint8_t value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID_Plus, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static uint8_t s_aucLastPwm[4] = {150,150,150,150};
	if(s_aucLastPwm[chan] != value)//值改变时才发送
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN_Transmit(CAN2, &TxMessage);
	}
}
