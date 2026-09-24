#include "DM_Motor.h"

///////////////////达妙J4310/////////////////////////////

/// @brief CAN通信发送数据
/// @param hcan 选择对应的CAN句柄
/// @param ID  CANID
/// @param pData 数据包数组
/// @param Len 发送的数据长度（字节数）
void DM_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len)
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
        System_Monitor.Cnt_DM_Send++;
    }
}


//////电机使能/////
void DM_Init(void)
{
	uint8_t buffer[8];
	buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = 0xFF;
    buffer[3] = 0xFF;
    buffer[4] = 0xFF;
    buffer[5] = 0xFF;
    buffer[6] = 0xFF;
    buffer[7] = 0xFC;
	
	DM_CANx_SendstdData(&hcan1, 0x01,buffer, 8);        
};

//控制帧打包
void DM_Pack_Control(DM_CMD *control, uint8_t *data)
{
	 
    // 将浮点参数转换为原始整数值
    uint16_t p_des_raw = FloatToUint(control->p_des*PI/180 , DM_POSITION_MIN, DM_POSITION_MAX, DM_SEND_POSITION_LENGTH);
    uint16_t v_des_raw = FloatToUint(control->v_des, DM_VELOCITY_MIN, DM_VELOCITY_MAX, DM_SEND_VELOCITY_LENGTH);
    uint16_t kp_raw = FloatToUint(control->kp, DM_KP_MIN, DM_KP_MAX, DM_SEND_KP_LENGTH);
    uint16_t kd_raw = FloatToUint(control->kd, DM_KD_MIN, DM_KD_MAX, DM_SEND_KD_LENGTH);
    uint16_t t_fi_raw = FloatToUint(control->t_fi, DM_TORQUE_MIN, DM_TORQUE_MAX, DM_SEND_TORQUE_LENGTH);
	
    
    // 参数范围检查
    kp_raw = (kp_raw > 500) ? 500 : kp_raw;
    kd_raw = (kd_raw > 5) ? 5 : kd_raw;
    
    // 打包数据到CAN帧
    // D[0]: p_des[15:8]
    data[0] = (p_des_raw >> 8);
    // D[1]: p_des[7:0]
    data[1] = p_des_raw;
    
    // D[2]: v_des[11:4]
    data[2] = (v_des_raw >> 4);
    // D[3]: v_des[3:0] Kp[11:8]
    data[3] = ((v_des_raw & 0x0F) << 4) | (kp_raw >> 8) ;
    
    // D[4]: Kp[7:0]
    data[4] = kp_raw;
    
    // D[5]: Kd[11:4]
    data[5] = (kd_raw >> 4) ;
    // D[6]: Kd[3:0] t_fi[11:8]
    data[6] = ((kd_raw & 0x0F) << 4) | (t_fi_raw >> 8) ;
    
    // D[7]: t_fi[7:0]
    data[7] = t_fi_raw ;
}

//反馈帧解包
void DM_Unpack_Feedback(uint8_t *data, DM_DATA *feedback)
{
    // D[0]: ID[ERR<<4] - 高4位是ERR，低4位是ID
    feedback->err = (data[0] >> 4) & 0x0F;
    feedback->id = data[0] & 0x0F;
    
    // 解包位置（16位有符号）
    uint16_t pos_raw = (uint16_t)((data[1] << 8) | data[2]);
    feedback->pos = UintToFloat(pos_raw ,FEEDBACK_POSITION_MIN, FEEDBACK_POSITION_MAX, FEEDBACK_POSITION_LENGTH)*180/PI;
    
    // 解包速度（12位有符号）
    uint16_t vel_raw = (uint16_t)((data[3] << 4) | ((data[4] >> 4) & 0x0F));
    // 12位有符号数符号扩展
    if (vel_raw & 0x0800) {
        vel_raw |= 0xF000;
    }
    feedback->vel = UintToFloat(vel_raw , FEEDBACK_VELOCITY_MIN, FEEDBACK_VELOCITY_MAX, FEEDBACK_VELOCITY_LENGTH)*DEG;
    
    // 解包扭矩（12位有符号）
    uint16_t torque_raw = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
    // 12位有符号数符号扩展
    if (torque_raw & 0x0800) {
        torque_raw |= 0xF000;
    }
    feedback->torque =  UintToFloat(torque_raw , FEEDBACK_TORQUE_MIN, FEEDBACK_TORQUE_MAX, FEEDBACK_TORQUE_LENGTH);
    
    // 温度数据（直接读取，单位已经是℃）
    feedback->t_mos = (float)data[6];
    feedback->t_rotor = (float)data[7];
}

// void DM_MotorCtrl(CAN_HandleTypeDef *hcan, uint32_t id,DM_CMD *control)//发送函数
// {
// 	// 计算反作用力矩前馈
// //	Yaw_Feedforward_Update(BlockArm_Joint1_J60Data.torque_,BlockArm_Joint2_J60Data.torque_,&Yaw_ff,&Gravity_BlockArm);
// //	BlockArm_Gimbal_DMCMD.t_fi=Yaw_ff.t_ff;
//     BlockArm_Gimbal_DMCMD.p_des=MotorInput.BlockArm_Gimbal_DM;
//     DM_Pack_Control(control, DM_Data);
// 	DM_CANx_SendstdData(hcan, id,DM_Data, 8);
// }
