#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "ROBOT.h"
void ramp_signal(float* p_Output, float DesValue, float Step);
void set_bit1(u8 *data ,u8 n);
void set_bit0(u8 *data ,u8 n);
void LESO_Order1(ST_LESO_1order * leso_1order, float y,float U0);
void LESO_Order2(ST_LESO_2order * leso_2order, float y,float U0);
float plan_of_time(float start_pos, float end_pos, float time);
void Luenberger_observer_uniform_velocity_model( ST_Luenberger_observer* observer);
void LpFilter(ST_LPF *lpf);

void G_vofa_watch(void);
#endif


