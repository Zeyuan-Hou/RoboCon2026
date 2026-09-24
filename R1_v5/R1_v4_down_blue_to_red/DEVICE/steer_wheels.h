#ifndef ___STEER_WHEELS_H___
#define ___STEER_WHEELS_H___

#include "robot.h"



void SteerWheels_distribute(STEER_WHEELS* wheels, ST_VEL *local_vel);
void steer_optimize(STEER_WHEEL *wheel, float Q_des, float vel_des);
void GlobalVel_To_Local(ST_VEL *local_vel, ST_VEL *global_vel, float fpQ);
void SteerLock(STEER_WHEELS* wheels, float leftup_angle, float rightup_angle, float leftdown_angle, float rightdown_angle);
void SteerFixed(STEER_WHEELS *wheels, float angle, float vel, float w);
void SteerWheels_solve(STEER_WHEELS* wheels, ST_VEL *cur_vel, float *residual_sse);
void SteerWheels_FfCalc(STEER_WHEELS *wheels, ST_VEL *local_vel, float ff_v_k1, float ff_v_k2, float ff_w_k1, float ff_w_k2);
void SteerWheels_solve_withGyro(STEER_WHEELS *wheels, ST_VEL *cur_vel, float *residual_sse, float omega);






#endif // ___STEER_WHEELS_H___









