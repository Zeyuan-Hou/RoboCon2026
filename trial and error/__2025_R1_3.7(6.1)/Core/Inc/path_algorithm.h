#ifndef __PATH_ALGORITHM_H
#define __PATH_ALGORITHM_H
#include "ROBOT.h"
#include "math_algorithm.h"

void SET_NAV_PATH_PERMUTATION(u8 number);
void SET_NAV_PATH_AUTO(u8 number);
void Path_Permutation(ST_VECTOR* Point_Start, ST_VECTOR* Point_Inc, ST_VECTOR* V_Start, ST_VECTOR* V_End,
		                        PATH_TYPE* Path_Type, fp32* A, fp32* R, fp32* T, u8 num_module);
void NavPosition(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void Rotation_Permutation(fp32* Rotation_Start,fp32* W_Start,fp32* Rotation_Inc,fp32* W_End
													,fp32* A_W,fp32* T,u8 num_modele_w );
void NavRotation(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void CheckPathEnd(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void Path_Permutation_Set_1(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav);
void Path_Permutation_Set_2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav);
void path_permutation_choose(ST_Nav *p_nav);
static void Path_Calibration(ST_Nav *p_nav);
void path_choose(ST_Nav *p_nav);
extern u8 flag_rotation;

extern fp32 target_q;
void Aim_Yaw(void);
#endif

