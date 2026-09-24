#ifndef ___CHASSIS_H___
#define ___CHASSIS_H___

#include "robot.h"
#include "steer_wheels.h"
#include "v6_motor.h"
#include "dji_motor.h"
#include "qd.h"
#include "vision.h"

void Chassis_Run(void);
void Chassis_Change(CHASSIS_STATE newstate);
void chassis_init(void);
void chassis_clear(void);
void chassis_remote_yaw_clear(void);
void chassis_pid_calc(void);
void chassis_protection(void);
void para_change_withNav(void);
void pid_change_with_vel(ST_PID *pid, float kp1, float kp2);
void chassis_LED(Error_t err_type);
void pid_change_with_vel(ST_PID *pid, float kp1, float kp2);
void intg_clear_with_brake(float min_des, float min_e);













#endif // ___CHASSIS_H___




