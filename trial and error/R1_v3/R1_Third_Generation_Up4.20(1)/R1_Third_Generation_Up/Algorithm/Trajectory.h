#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <stdbool.h>
#include "Types.h"


uint8_t Traj_Angle_Init(Trajectory* traj, float q0, float q1, float v_max, float T) ;
void Traj_Angle_Update(Trajectory* traj, float* pos, float* vel, float* acc);

uint8_t Traj_Line_Init(Trajectory* traj,
                          float xs, float ys, float xe, float ye,
                          float Vs, float Ve,
                          float V_max, float a_max);

void Traj_Line_Update(Trajectory* traj, ARM_BACKSOLVING* arm); 

uint8_t Traj_Arc_Init(Trajectory* traj,
                         float cx, float cy, float r,
                         float start_angle, float sweep_angle,
                         float Vs, float Ve,
                         float V_max, float a_max);
void Traj_Arc_Update(Trajectory* traj, ARM_BACKSOLVING* arm);
uint8_t BlockArm_BackSolve(ARM_BACKSOLVING *params);

#endif
