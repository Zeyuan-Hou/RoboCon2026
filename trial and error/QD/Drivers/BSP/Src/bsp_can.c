#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"


void CAN_INIT(void) 
{
    bsp_fdcan1_init(); //过滤器初始化
    bsp_fdcan2_init();
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
	FDCAN_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;    //过滤器1关联到FIFO1  
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
 if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
			Error_Handler();
	}
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
 * @brief FDCAN1 接收回调函数
 *
 * @brief 
 *
 * @param
 *
 * @return
 *******************************************************************/
//FD CAN1 收取中断
#define BUFFER_SIZE 8
uint8_t Can_rxBuffer1[BUFFER_SIZE]={0};//接收数组
uint8_t Can_rxBuffer2[BUFFER_SIZE]={0};//接收数组




void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{	
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)   //FIFO0新数据中断
  {
		FDCAN_RxHeaderTypeDef   RxHeader; // 用来保存接收到的数据帧头部信息
		uint8_t                 RxData[8]; // 用来保存接收数据端数据
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
		switch(RxHeader.Identifier)
		{
			case 0x300:
				for(int i=0; i<RxHeader.DataLength; i++)
				{
					Can_rxBuffer1[i] = RxData[i];
				}
				TempValveStat = Can_rxBuffer1[0];
				VAL_COM_cnt ++;
				break;
			
			default:
				break;							
		}		
		/* 提取FIFO0中接收到的数据 */
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  }
}

//FD CAN2 收取中断
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{	
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)   //FIFO1新数据中断
  {
		FDCAN_RxHeaderTypeDef   RxHeader; // 用来保存接收到的数据帧头部信息
		uint8_t                 RxData[8]; // 用来保存接收数据端数据
		HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader, RxData);
		switch(RxHeader.Identifier)
		{

//			case 0x300:
//				for(int i=0; i<RxHeader.DataLength; i++)
//				{
//					Can_rxBuffer1[i] = RxData[i];
//				}
//				TempValveStat = Can_rxBuffer1[0];
//				VAL_COM_cnt ++;
//				break;
//			case 0x111:
//				for(int i=0; i<RxHeader.DataLength; i++)
//				{
//					Can_rxBuffer2[i] = RxData[i];
//				}
//				TempValveStat = Can_rxBuffer2[0];
//				break;
			
			default:
				break;
		}
		/* 提取FIFO1中接收到的数据 */
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	}

}



