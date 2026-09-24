#ifndef _UP_DOWN_STEP_H_
#define _UP_DOWN_STEP_H_

#include "Type.h"
#include "PID.h"
#include "GO1_MOTOR.h"

void GO1_pid_change(void);
void up_logic_200(void);
void up_logic_400(void);
void down_logic_200(void);
void down_logic_400(void);
void up_down_logic(void);
void GO_ctrl_logic(void);
void test_lift(float lift_rad);
void GO1_fold(void);
void test_lift_two(float lift_rad);
float Cal_Angle_Error(float current, float target);
void combine(void);

void test_height(void);
#endif

