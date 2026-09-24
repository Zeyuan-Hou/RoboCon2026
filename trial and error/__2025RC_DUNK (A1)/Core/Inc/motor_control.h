#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "ROBOT.h"
#include "pid.h"
#include "math_algorithm.h"
#include "algorithm.h"
int jump_motor_pos_mode(void);
void jump_motor_pos_ctrl(void);
void jump_motor_damp_mode(void);
void jump_motor_control(ST_Jump_Motor_Ctrl *motor_ctrl);
void arm_motor_control(void);
#endif

