#include "QD.h"

/// @brief CAN通信发送数据
/// @param hcan 选择对应的CAN句柄
/// @param ID  CANID
/// @param pData 数据包数组
/// @param Len 发送的数据长度（字节数）
void QD_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len)
{
  CAN_TxHeaderTypeDef Tx_Header;
  Tx_Header.StdId=ID;//CANID
  Tx_Header.ExtId=0;//扩展帧ID
  Tx_Header.IDE=CAN_ID_STD;
  Tx_Header.RTR=CAN_RTR_DATA;
  Tx_Header.DLC=Len;//发送的数据长度

  if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK)
  {
    if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX1)!=HAL_OK)
    {      HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX2);//发送数据，发送成功则返回HAL_OK，否则进入下一层
    }
  }
}

/// @brief 将控制电磁阀的六位uint8数组转换为发送给气动板的一位uint8数组，[i]对应权值2^i
/// @param bits 控制电磁阀的六位uint8数组
/// @return 发送给气动板的一位uint8数组
uint8_t Bin_Array_To_u8(uint8_t *bits)
{  // 用于存储最终转换结果的整型变量
    uint8_t result = 0;
    // 遍历二进制数组的每一位
    for (uint8_t i = 0; i < 6; i++) 
	 {
        // 每个位对应的权值为 2^i（因为第0位是最低位）
        result += bits[i] * (1 << i);  // 将当前位的值乘以对应的权值，并累加到结果中
   }
    return result;  // 返回转换后的整数值
}

/// @brief 解算电磁阀控制值并发送给气动板
/// @param hcan 选择对应的CAN句柄
void CAN_SendValveType(CAN_HandleTypeDef *hcan)
{
    AirCtrl[0]=Bin_Array_To_u8(AirOperaterCtrlBuf);//解算出发送给气动板控制电磁阀的一位uint8数组
	QD_CANx_SendstdData(hcan,0x300,AirCtrl,1);//将一位数组发送给气动板
}

/// @brief 将控制电磁阀的十位uint8数组转换为发送给气动板的三位uint8数组
/// @param bits 控制电磁阀的六位uint8数组
/// @return 发送给气动板的一位uint8数组
void Bin_Array_To_AirCtrl_New(uint8_t *bits,uint8_t *airctrl)
{  
	
    // 遍历二进制数组的每一位
    for (uint8_t i = 0; i < 3; i++) 
	{
        airctrl[i] = ((bits[4*i+3] & 0x03) << 6) |   /* 低两位移到 bit7~6 */
        ((bits[4*i+2] & 0x03) << 4) |   /* 低两位移到 bit5~4 */
        ((bits[4*i+1] & 0x03) << 2) |   /* 低两位移到 bit3~2 */
        (bits[4*i+0] & 0x03);           /* 低两位留在 bit1~0 */
	}
}

void CAN_SendValveType_New(CAN_HandleTypeDef *hcan)
{
    Bin_Array_To_AirCtrl_New(AirOperaterCtrlBuf, AirCtrl);
    QD_CANx_SendstdData(hcan, 0x300, AirCtrl, 3);
}

/// @brief 将从气动板接收到的DT35原始值存储到缓冲区中
/// @param pData 由CAN通信接收的DT35原始数据
void DT35_GET_Data(uint8_t *pData)
{
	
	DT35_Digital_Data[0]=pData[0]|pData[1]<<8;
    DT35_Digital_Data[1]=pData[2]|pData[3]<<8;
    DT35_Digital_Data[2]=pData[4]|pData[5]<<8;
    DT35_Digital_Data[3]=pData[6]|pData[7]<<8;
    for(uint16_t i=0;i<8;i++)
    {
      DT35_Data[i]=pData[i];
    }
}

