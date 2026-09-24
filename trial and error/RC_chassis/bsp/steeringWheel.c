#include "steeringWheel.h"

void ControlLoop(ST_MOTOR *motor)
{
    motor->innerFeedback = motor->anglev;
    motor->outerFeedback = motor->angle;

    switch (motor->ControlLoop_State)
    {
    case SPEED_LOOP:
        PID_Calc(&motor->pid_inner, motor->Input, motor->innerFeedback);
        motor->motor_current = motor->pid_inner.fpU;
        break;
    case POSITION_LOOP:
        PID_Calc(&motor->pid_outer, motor->Input, motor->outerFeedback);
        motor->motor_current = motor->pid_outer.fpU;
        // Position Loop Control
        break;
    case MULTIPLE_LOOP:
        motor->td.aim = motor->Input;
        CalTD(&(motor->td)); // smoothen input curve

        PID_Calc(&motor->pid_outer, motor->td.x1, motor->outerFeedback);
        PID_Calc(&motor->pid_inner, motor->pid_outer.fpU + motor->td.x2 * 0.3f, motor->innerFeedback); // smoothen inner_output(anglev) curve
        motor->motor_current = motor->pid_inner.fpU;
        // Multiple Loop Control
        break;
    default:
        break;
    }

    switch (motor->motor_type){
        case M6C18:
            switch (motor->motor_id){
            case 0x001:// 0x000+id M6C18
                CAN_SendCurrent_V6(&hcan1, 0x200, motor->motor_current, motor->motor_id);
                break;
            case 0x002:
                CAN_SendCurrent_V6(&hcan1, 0x200, motor->motor_current, motor->motor_id);
                break;
            case 0x003:
                CAN_SendCurrent_V6(&hcan1, 0x200, motor->motor_current, motor->motor_id);
                break;
            case 0x004:
                CAN_SendCurrent_V6(&hcan1, 0x200, motor->motor_current, motor->motor_id);
                break;
            default:
                break;
            }
            break;

        case M6020:
            switch (motor->motor_id){
            case 0x001:// 0x200+id M6020
                CAN_SendCurrent(&hcan1, 0X1FF, motor->motor_current, 0, 0, 0);
                break;
            case 0x002:
                CAN_SendCurrent(&hcan1, 0X1FF, 0, motor->motor_current, 0, 0);
                break;
            case 0x003:
                CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, motor->motor_current, 0);
                break;
            case 0x004:
                CAN_SendCurrent(&hcan1, 0X1FF, 0, 0, 0, motor->motor_current);
                break;
            case 0x006:
                CAN_SendCurrent(&hcan1, 0X2FF, 0, motor->motor_current, 0, 0);
                break;
            default:
                break;
            }
            break;
        default:    
            break;
    }
}
