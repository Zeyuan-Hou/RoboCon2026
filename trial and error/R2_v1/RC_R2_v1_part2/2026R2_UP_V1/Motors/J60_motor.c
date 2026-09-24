#include "J60_motor.h"
#include "can.h"
#include "CANBsp.h"

/**J60_Func**/
uint32_t FloatToUint(const float x, const float x_min, const float x_max, const uint8_t bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x-offset)*((float)((1<<bits)-1))/span);
}
float UintToFloat(const int x_int, const float x_min, const float x_max, const uint8_t bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
//pack data transmitted to j60
void FloatsToUints(const MotorCMD *param, uint8_t *data)
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

 
//uncpack data received in main
void UintsToFloats(MotorDATA *data,uint8_t *RxBuf)
{
    uint32_t data32[5];
		uint64_t raw_64bit = 
        ((uint64_t)RxBuf[7] << 56) | 
        ((uint64_t)RxBuf[6] << 48) | 
        ((uint64_t)RxBuf[5] << 40) | 
        ((uint64_t)RxBuf[4] << 32) | 
        ((uint64_t)RxBuf[3] << 24) | 
        ((uint64_t)RxBuf[2] << 16) | 
        ((uint64_t)RxBuf[1] << 8)  | 
        (uint64_t)RxBuf[0];
		data32[0] =  (raw_64bit >> 0) & 0xFFFFF;
		data32[1] = (raw_64bit >> 20) & 0xFFFFF;
		data32[2] = (raw_64bit >> 40) & 0xFFFF;
		data32[3] = (raw_64bit >> 56) & 0x1;
		data32[4] = (RxBuf[7] >> 1) & 0x7F; 
	
    data->position_ = UintToFloat(data32[0], POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
    data->velocity_ = UintToFloat(data32[1], VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
    data->torque_ = UintToFloat(data32[2], TORQUE_MIN, TORQUE_MAX, RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)data32[3];
    if(data->flag_ == kMotorTempFlag){
        data->temp_ = UintToFloat(data32[4], MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
    else{
        data->temp_ = UintToFloat(data32[4], DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
}

void J60_ReceiveData(DEEP_MOTOR *motor,MotorDATA *data,uint8_t *RxBuf){
	UintsToFloats(data,RxBuf);
	motor->position_=data->position_;
	motor->velocity_=data->velocity_;
	motor->torque_=data->torque_;
	motor->temp_=data->temp_;
}

uint16_t FormCanId(uint8_t cmd, uint8_t motor_id){
    return (cmd << CAN_ID_SHIFT_BITS) | motor_id;
}

void SetMotionCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd, float position, float velocity, float torque, float kp, float kd){
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
    motor_cmd->position_ = position;
    motor_cmd->velocity_ = velocity;
    motor_cmd->torque_ = torque;
    motor_cmd->kp_ = kp;
    motor_cmd->kd_ = kd;
}
void CAN_Send_DeepMsg(MotorCMD *motorCMD)
{
	FloatsToUints(motorCMD,CAN1_TxBuf);
	uint32_t ID =0x80+motorCMD->motor_id_;//控制模式下CAN发送的ID
	CANx_SendstdData(&hcan1,ID,CAN1_TxBuf,8);
}

void Deep_Init(DEEP_MOTOR *motor,MotorCMD *motorCMD){
	SetMotionCMD(motorCMD,motor->motor_id_,2,0,0,0,0,0);
	FloatsToUints(motorCMD,CAN1_TxBuf);
	uint32_t ID =FormCanId((uint8_t)2,motor->motor_id_);
	CANx_SendstdData(&hcan1,ID,CAN1_TxBuf,0);
}

