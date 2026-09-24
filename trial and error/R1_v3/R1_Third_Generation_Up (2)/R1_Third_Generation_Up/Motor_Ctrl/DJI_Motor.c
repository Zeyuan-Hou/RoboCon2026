#include "DJI_Motor.h"

/// @brief CAN通信发送数据
/// @param hcan 选择对应的CAN句柄
/// @param ID  CANID
/// @param pData 数据包数组
/// @param Len 发送的数据长度（字节数）
void DJI_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len)
{
  CAN_TxHeaderTypeDef Tx_Header;
  Tx_Header.StdId=ID;//CANID
  Tx_Header.ExtId=0;//扩展帧ID
  Tx_Header.IDE=CAN_ID_STD;
  Tx_Header.RTR=CAN_RTR_DATA;
  Tx_Header.DLC=Len;//发送的数据长度

    uint32_t TxMailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, &TxMailbox);
    if (status != HAL_OK)
    {
        // 可以尝试其他邮箱，但通常不需要，因为 HAL_CAN_AddTxMessage 会自动选择一个空闲邮箱
        // 如果失败，可在此处添加错误处理或重试逻辑
    }
    else
    {
        System_Monitor.Cnt_DJI_Send++;
    }

//   if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK)
//   {
//     if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX1)!=HAL_OK)
//     {      HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t *)CAN_TX_MAILBOX2);//发送数据，发送成功则返回HAL_OK，否则进入下一层
//     }
//   }
}

/// @brief 获得大疆电机的编码器值
/// @param motor 大疆电机结构体
/// @param msg 通过CAN通信接收的大疆反馈数据包
/// @return 编码器值
float DJI_GetEncoderNumber(ST_MOTOR* motor,uint8_t msg[8])
{
    motor->EncoderNum=(msg[0]<<8)|(msg[1]);
    return motor->EncoderNum;
}

/// @brief 获得大疆电机的速度值
/// @param pcanRxMsg CAN接收消息头
/// @param msg 通过CAN通信接收的大疆反馈数据包
/// @return 速度值
float DJI_GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg, uint8_t msg[8])
{
    int32_t speed_temp;
    int32_t base_value = 0xFFFF;
    if(msg[2]& 0x01<<7)
    {
        speed_temp=(base_value<<16|msg[2]<<8|msg[3]);
    }
    else
    {
        speed_temp=(msg[2]<<8)|(msg[3]);
    }
    return speed_temp;
}

/// @brief 获得大疆电机的电流值
/// @param pcanRxMsg CAN接收消息头
/// @param msg 通过CAN通信接收的大疆反馈数据包
/// @return 电流值
float DJI_GetCurrent(CAN_RxHeaderTypeDef* pcanRxMsg, uint8_t msg[8])
{
    int32_t speed_temp;
    int32_t base_value = 0xFFFF;
    if(msg[4]& 0x01<<7)
    {
        speed_temp=(base_value<<16|msg[4]<<8|msg[5]);
    }
    else
    {
        speed_temp=(msg[4]<<8)|(msg[5]);
    }
    return speed_temp;
}

/// @brief 发送目标电流给大疆电机
/// @param hcan 对应的CAN句柄
/// @param id 发送的ID
/// @param current 目标电流,[0][1][2][3]对应电机ID1234或5678
void DJI_CAN_Sendcurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t *current)
{
    uint8_t TxCurrent[8];
    for (uint16_t i = 0; i < 4;i++)
    {
        TxCurrent[2 * i] = (current[i] >> 8);
        TxCurrent[2*i+1] = current[i];
    }
    DJI_CANx_SendstdData(hcan, id, TxCurrent, 8);
}

/// @brief 计算大疆电机编码器累积值，处理编码器值超限问题，当编码器值变化超过上限值的一半时，认为超限，进行处理
/// @param pEncoder CAN接收消息头
/// @param value 当前编码器值
void DJI_Abs_Encoder_Process(ST_ENCODER* pEncoder, uint32_t value)
{
    pEncoder->siPreRawValue =pEncoder->siRawValue;
    pEncoder->siRawValue =value;
    pEncoder->siDiff = pEncoder->siRawValue - pEncoder->siPreRawValue;
    if(pEncoder->siDiff>(pEncoder->siNumber)/2)
    {
        pEncoder->siDiff -= pEncoder->siNumber;
    }
    else if(pEncoder->siDiff<-(pEncoder->siNumber)/2)
    {
        pEncoder->siDiff += pEncoder->siNumber;
    }
    pEncoder->siSumValue +=pEncoder->siDiff;
  if(pEncoder->state == 1)//上电时将线数清零
  {
      pEncoder->siSumValue = 0;
      pEncoder->state = 0;
  }
}

void DJI_MotorCtrl(void)
{
    //存取杆机械臂第二个关节的2006电机,ID1
    PID_CascadeCalc(&PoleArm_Joint2_2006_PID, MotorInput.PoleArm_Joint2_2006, PoleArm_Joint2_2006.angle, PoleArm_Joint2_2006.anglev);//计算PID
    float FrictionTemp1 = FrictionFeedforward(&Friction_PoleArm_Joint2, PoleArm_Joint2_2006.anglev);//计算摩擦前馈
    if (fabs((double)FrictionTemp1)>=MAX_DJIFrictionFeedForward)
    {
            FrictionTemp1=0;
    }
    Current1_4[0] = PoleArm_Joint2_2006_PID.output + Gravity_PoleArm.Output_Tor[1] + FrictionTemp1;//PID+重力前馈+摩擦前馈

    //存取杆机械臂第一个关节（靠近基座）的3508电机,ID2
    PID_CascadeCalc(&PoleArm_Joint1_3508_PID, MotorInput.PoleArm_Joint1_3508, PoleArm_Joint1_3508.angle, PoleArm_Joint1_3508.anglev);//计算PID
    float FrictionTemp2 = FrictionFeedforward(&Friction_PoleArm_Joint1, PoleArm_Joint1_3508.anglev);//计算摩擦前馈
    if (fabs((double)FrictionTemp2)>=MAX_DJIFrictionFeedForward)
    {
            FrictionTemp2=0;
    }
    Current1_4[1] = PoleArm_Joint1_3508_PID.output + Gravity_PoleArm.Output_Tor[0] + FrictionTemp2;//PID+重力前馈+摩擦前馈

    //驱动摩擦轮的3508电机,ID3
//   PID_CascadeCalc(&PoleArm_FrictionWheel_3508_PID, MotorInput.PoleArm_FrictionWheel_3508, PoleArm_FrictionWheel_3508.angle, PoleArm_FrictionWheel_3508.anglev);//计算PID
	PID_Calc(&PoleArm_FrictionWheel_3508_PID.inner, MotorInput.PoleArm_FrictionWheel_3508, PoleArm_FrictionWheel_3508.anglev);
    float FrictionTemp3 = FrictionFeedforward(&Friction_PoleArm_FrictionWheel, PoleArm_FrictionWheel_3508.anglev);//计算摩擦前馈
    if (fabs((double)FrictionTemp3)>=MAX_DJIFrictionFeedForward)
    {
            FrictionTemp3=0;
    }
    Current1_4[2] = PoleArm_FrictionWheel_3508_PID.inner.fpU + FrictionTemp3;//PID+摩擦前馈

    //存取块机械臂第三个关节的3508电机,ID4
    PID_CascadeCalc(&BlockArm_Joint3_3508_PID, MotorInput.BlockArm_Joint3_3508, BlockArm_Joint3_3508.angle, BlockArm_Joint3_3508.anglev);//计算PID
    float FrictionTemp4 = FrictionFeedforward(&Friction_BlockArm_Joint3, BlockArm_Joint3_3508.anglev);//计算摩擦前馈
    if (fabs((double)FrictionTemp4)>=MAX_DJIFrictionFeedForward)
    {
            FrictionTemp4=0;
    }
    Current1_4[3] = BlockArm_Joint3_3508_PID.output+Gravity_BlockArm.Output_Tor[2]+FrictionTemp4;//PID+重力前馈+摩擦前馈


    //用于微调对接的2006电机,ID5
//    PID_CascadeCalc(&PoleArm_Adjustment_2006_PID, MotorInput.PoleArm_Adjustment_2006, PoleArm_Adjustment_2006.angle, PoleArm_Adjustment_2006.anglev);//计算PID
    PID_Calc(&PoleArm_Adjustment_2006_PID.inner, MotorInput.PoleArm_Adjustment_2006, PoleArm_Adjustment_2006.anglev);
	float FrictionTemp5 = FrictionFeedforward(&Friction_PoleArm_Adjustment,PoleArm_Adjustment_2006_PID.inner.fpDes);//计算摩擦前馈
    if (fabs((double)FrictionTemp5)>=MAX_DJIFrictionFeedForward)
    {
            FrictionTemp5=0;
    }
    Current5_8[0] = PoleArm_Adjustment_2006_PID.inner.fpU + FrictionTemp5;//PID+摩擦前馈

    DJI_CAN_Sendcurrent(&hcan1,0x200,Current1_4);
    DJI_CAN_Sendcurrent(&hcan1,0x1FF,Current5_8);

}
