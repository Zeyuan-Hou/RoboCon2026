#include "J60_sdk.h"

//包括使能、失能、MIT控制、机械臂J60的重力前馈

// 用于往MotorCMD写入普通命令
void J60_SetNormalCMD(J60_MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
}

// 用于往MotorCMD写入控制命令
void J60_SetMotionCMD(J60_MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd, float position, float velocity, float torque, float kp, float kd)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
    motor_cmd->position_ = position;
    motor_cmd->velocity_ = velocity;
    motor_cmd->torque_ = torque;
    motor_cmd->kp_ = kp;
    motor_cmd->kd_ = kd;
}

void J60_Unpack_Feedback(const uint8_t can_rx[8], J60_MotorDATA *data, uint32_t StdId)
{
    const J60_ReceivedMotionData *rx_msg = (const J60_ReceivedMotionData *)can_rx;
    data->motor_id_ = StdId & 0x0f;
    data->cmd_ = (StdId >> 5) & 0x3f;
    data->position_ = uint_to_float(rx_msg->position, J60_POSITION_MIN, J60_POSITION_MAX, J60_RECEIVE_POSITION_LENGTH);
    data->velocity_ = uint_to_float(rx_msg->velocity, J60_VELOCITY_MIN, J60_VELOCITY_MAX, J60_RECEIVE_VELOCITY_LENGTH);
    data->torque_ = uint_to_float(rx_msg->torque, J60_TORQUE_MIN, J60_TORQUE_MAX, J60_RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)rx_msg->temp_flag;
    if(data->flag_ == 1) data->temp_ = uint_to_float(rx_msg->temperature, J60_MOTOR_TEMP_MIN, J60_MOTOR_TEMP_MAX, J60_RECEIVE_TEMP_LENGTH);
    else data->temp_ = uint_to_float(rx_msg->temperature, J60_DRIVER_TEMP_MIN, J60_DRIVER_TEMP_MAX, J60_RECEIVE_TEMP_LENGTH);

    if ((fabs(data->torque_) > 50.f || data->temp_ > 150.f) && data->status != PROTECTED){
        data->status = PROBLEM;
    }else if(fabs(data->torque_) <= 50.f && data->temp_ <= 150.f){
        data->status = OK;
        data->j60_problem_time = 0;
    }
}

uint16_t J60_FormCanId(uint8_t cmd, uint8_t motor_id)
{
    return (cmd << 5) | motor_id;
}

void J60_MOTOR_ENABLE(CAN_HandleTypeDef *hcan, J60_MotorCMD *data)
{
    data->cmd_ = J60_ENABLE_CMD;
    CAN_SendData_J60(hcan, data);
}

void J60_MOTOR_DISABLE(CAN_HandleTypeDef *hcan, J60_MotorCMD *data)
{
    data->cmd_ = J60_DISABLE_CMD;
    CAN_SendData_J60(hcan, data);
}

void J60_LOOP_CONTROL(CAN_HandleTypeDef *hcan, J60_MotorCMD *data, float position, float velocity, float torque, float kp, float kd, J60_MotorDATA *feedback)
{
    static uint8_t flag_exit_ctrl_mode;

    if(feedback->status == PROTECTED){
        data->cmd_ = J60_CONTROL_CMD;
        data->position_ = 0.f;
        data->velocity_ = 0.f;
        data->torque_ = 0.f;
        data->kp_ = 0.f;
        data->kd_ = 2.f;
        flag_exit_ctrl_mode = 0;
        CAN_SendData_J60(hcan, data);
    }
    else{
        if (flag_exit_ctrl_mode == 0){
            data->position_ = feedback->position_;
            data->j60_td.x1 = feedback->position_;
            data->j60_td.x2 = 0;
            position = feedback->position_;
            flag_exit_ctrl_mode = 1;
        }

        data->cmd_ = J60_CONTROL_CMD;
        data->j60_td.aim = position;
        CalTD(&data->j60_td);
        data->position_ = data->j60_td.x1;
        data->velocity_ = velocity;
        data->torque_ = torque;
        data->kp_ = kp;
        data->kd_ = kd;
        CAN_SendData_J60(hcan, data);
    }
}

float J60_feedforward_G_torque(float torque_k1, float pos_j60, float torque_k2, float pos_dm)
{
    return torque_k1 * cosf(pos_j60 + pos_dm) + torque_k2 * cosf(pos_j60);
}
