#ifndef ___LOCATION_H___
#define ___LOCATION_H___

#include "robot.h"
#include "vision.h"
#include "steer_wheels.h"
#include "gyro.h"


void Location_dt35_vision(void);
void Dt35_Calc(float des);
uint8_t angle_judge(float des);
uint8_t range_judge(float x_min, float x_max, float y_min, float y_max, float yaw_des);
void PosKF_Predict(PosKF_t *kf, ST_VEL *cur_vel, float residual_sse);
void PosKF_Update(PosKF_t *kf, ST_POS *obs_pos);
void Delayed_PosKF_Update(ST_POS *radar, uint8_t delayed_ticks);
void Delayed_PosKF_Predict(uint16_t cur_delay);
void Delayed_PosKF_Input(ST_VEL *cur_vel, float residualSSE);
void Delayed_PosKF_Init(ST_POS *pos);
void Dt35_position(ST_POS *pos);
void onlydt35_position(ST_POS *pos);

#endif // ___LOCATION_H___







