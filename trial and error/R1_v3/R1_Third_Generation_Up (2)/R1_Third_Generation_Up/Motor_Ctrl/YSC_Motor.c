#include "YSC_Motor.h"

/// @brief 云深处电机(J60等)的数据发送函数
/// @param motor_cmd 云深处电机控制结构体
void YSC_Motor_CAN_Send_Data(MotorCMD *motor_cmd)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  tx_header.StdId = YSC_Get_CANID(motor_cmd);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = YSC_Get_DLC(motor_cmd);
    
  if(motor_cmd->cmd_ == CONTROL_MOTOR)
    YSC_FloatsToUints(motor_cmd, tx_data);

  if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
  {
    if(HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_data,(uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
        HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_data,(uint32_t *)CAN_TX_MAILBOX2);
    }
  }
  System_Monitor.Cnt_J60_Send++; 
}

/// @brief 用于将MotorCMD中的float数据转换为CAN协议中发送的uint数据
/// @param param 云深处电机控制结构体
/// @param data 要发送的数据数组
void YSC_FloatsToUints(const MotorCMD *param, uint8_t *data)
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

float position_temp = 0;
/// @brief 用于将CAN协议中收到的uint数据转换为MotorDATA中的float数据,存储到接收数据结构体中
/// @param rxdata 存储接收数据的数组
/// @param data 云深处电机接收数据结构体,解算得角度值和速度值为角度制
void YSC_UintsToFloats(uint8_t *rxdata, MotorDATA *data)
{
    uint64_t raw_data = 0;
    for (int i = 0; i < 8; i++) {
        raw_data |= ((uint64_t)rxdata[i] << (i * 8));
    }
    uint32_t rawposition_ = (raw_data >> 0) & 0xFFFFF;      
    uint32_t rawvelocity_ = (raw_data >> 20) & 0xFFFFF;     
    uint16_t raw_torque = (raw_data >> 40) & 0xFFFF;        
    uint8_t temp_flag = (raw_data >> 56) & 0x1;             
    uint8_t raw_temperature = (raw_data >> 57) & 0x7F;      
    data->position_ = ((rawposition_ * YSC_RANGE / YSC_MAX_RAW_VALUE) + YSC_OFFSET)/PI*180.0f;
    data->velocity_ = ((rawvelocity_ * YSC_RANGE / YSC_MAX_RAW_VALUE) + YSC_OFFSET)/PI*180.0f;
    data->torque_ = (raw_torque * YSC_TORQUE_RANGE / YSC_TORQUE_MAX_RAW_VALUE) + YSC_TORQUE_OFFSET;
    data->flag_ = (temp_flag == 1);
    data->temp_ = (raw_temperature * YSC_TEMP_RANGE / YSC_TEMP_MAX_RAW_VALUE) + YSC_TEMP_OFFSET;
}

/// @brief 结合motor_id和cmd形成CAN协议中发送的id(小端序，Bit0-Bit4为电机id，Bit5-Bit10为命令索引值)
/// @param cmd 云深处电机控制结构体
/// @return 生成的CANid
uint16_t YSC_Get_CANID(MotorCMD *cmd)
{
    uint16_t canid = (cmd->cmd_ << 5) | cmd->motor_id_;
    return canid;
}

/// @brief 用于根据MotorCMD进行所发送can帧中DLC的填充
/// @param motor_cmd 云深处电机控制结构体
/// @return DLC值
uint8_t YSC_Get_DLC(MotorCMD *motor_cmd)
{
    uint8_t can_dlc = 0;
    switch (motor_cmd->cmd_)
    {
    case ENABLE_MOTOR:
        can_dlc = SEND_DLC_ENABLE_MOTOR;
        break;
    case DISABLE_MOTOR:
        can_dlc = SEND_DLC_DISABLE_MOTOR;
        break;
    case SET_HOME:
        can_dlc = SEND_DLC_SET_HOME;
        break;
    case ERROR_RESET:
        can_dlc = SEND_DLC_ERROR_RESET;
        break;
    case CONTROL_MOTOR:
        can_dlc = SEND_DLC_CONTROL_MOTOR;
        break;
    case GET_STATUS_WORD:
        can_dlc = SEND_DLC_GET_STATUS_WORD;
        break;
    default:
        break;
    }
    return can_dlc;
}

/// @brief 用于根据收到的can帧进行MotorDATA的填充
/// @param motor_cmd 云深处电机控制结构体
/// @param data 云深处电机接收数据结构体
/// @param rxdata CAN通信接收数组
void YSC_Motor_Rcv_Data(MotorCMD *motor_cmd,MotorDATA *data,uint8_t *rxdata)
{
    data->motor_id_ = motor_cmd->motor_id_;
    data->cmd_ = motor_cmd->cmd_;
    switch (data->cmd_)
    {
    case ENABLE_MOTOR:
        break;
    case DISABLE_MOTOR:
        break;
    case SET_HOME:
        break;
    case ERROR_RESET:
        break;
    case CONTROL_MOTOR:
        YSC_UintsToFloats(rxdata, data);
        break;    
    case GET_STATUS_WORD:
        // data->error_ = (frame_ret->data[0] << 8) | frame_ret->data[1];
        break;
    default:
        break;
    }
}

/// @brief 用于往MotorCMD写入普通命令，如使能、失能等
/// @param motor_cmd 云深处电机控制结构体
/// @param motor_id 对应电机的id
/// @param cmd 功能对应的命令索引值(查手册)
void YSC_SetNormalCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
}

/// @brief 用于往MotorCMD写入控制命令
/// @param motor_cmd 云深处电机控制结构体
/// @param motor_id 要控制的电机的id
/// @param cmd 功能对应的命令索引值
/// @param position 目标角度，角度值
/// @param velocity 目标速度，角度值
/// @param torque 目标力矩，N/m
/// @param kp (0-1023)
/// @param kd (0-255)
void YSC_SetMotorCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd,float torque,float position,float velocity)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
    motor_cmd->position_ = position*PI/180.0f;//用户输入的pos为角度，发送弧度
    motor_cmd->velocity_ = velocity*PI/180.0f;//用户输入的vel为角度，发送弧度
    motor_cmd->torque_ = torque;
}

/// @brief 电机初始化，使能电机
/// @param motor_cmd 云深处电机控制结构体
/// @param motor_id 对应电机的id
void YSC_Motor_Init(MotorCMD *motor_cmd,uint8_t motor_id)
{
    motor_cmd->motor_id_= motor_id;//使能电机，cmd为2
    YSC_SetNormalCMD(motor_cmd,motor_cmd->motor_id_,ENABLE_MOTOR);
    YSC_Motor_CAN_Send_Data(motor_cmd);
}

/// @brief ，设置目标角度值和目标力矩，发送给电机
/// @param motor_cmd 云深处电机控制结构体
/// @param motor_data 云深处电机接收数据结构体
void YSC_MotorCtrl(MotorCMD *motor_cmd, MotorDATA *motor_data)
{
    if (motor_cmd->motor_id_ == 1)
    {

        YSC_Pos[0]=MotorInput.BlockArm_Joint1_J60;
        float TempFricFeed = FrictionFeedforward(&Friction_BlockArm_Joint1, motor_data->velocity_);
        if (fabs((double)TempFricFeed)>=MAX_J60FrictionFeedForward)
        {
            TempFricFeed=0;
        }
        YSC_Tor[0] =Gravity_BlockArm.Output_Tor[0] + TempFricFeed;

        YSC_SetMotorCMD(motor_cmd, motor_cmd->motor_id_, CONTROL_MOTOR, YSC_Tor[0],YSC_Pos[0],YSC_Vel[0]);
    }
    else if (motor_cmd->motor_id_ == 2)
    {
        YSC_Pos[1]=MotorInput.BlockArm_Joint2_J60;
        float TempFricFeed = FrictionFeedforward(&Friction_BlockArm_Joint2, motor_data->velocity_);
        if (fabs((double)TempFricFeed)>=MAX_J60FrictionFeedForward)
        {
            TempFricFeed=0;
        }
        YSC_Tor[1] = Gravity_BlockArm.Output_Tor[1] + TempFricFeed;

        YSC_SetMotorCMD(motor_cmd, motor_cmd->motor_id_, CONTROL_MOTOR, YSC_Tor[1],YSC_Pos[1],YSC_Vel[1]);
    }
    else if (motor_cmd->motor_id_ == 3)
    {
        YSC_Pos[2]=MotorInput.BlockArm_Gimbal_J60;
        float TempFricFeed = FrictionFeedforward(&Friction_BlockArm_Gimbal, motor_data->velocity_);
        if (fabs((double)TempFricFeed)>=MAX_J60FrictionFeedForward)
        {
            TempFricFeed=0;
        }
        YSC_Tor[2] =  TempFricFeed;

        YSC_SetMotorCMD(motor_cmd, motor_cmd->motor_id_, CONTROL_MOTOR, YSC_Tor[2],YSC_Pos[2],YSC_Vel[2]);
    }
    YSC_Motor_CAN_Send_Data(motor_cmd);
}
