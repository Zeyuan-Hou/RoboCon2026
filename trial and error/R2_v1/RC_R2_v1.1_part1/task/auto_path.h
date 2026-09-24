#ifndef __AUTO_PATH_H__
#define __AUTO_PATH_H__

#include "global_declare.h"
#include "math.h"
#include "algorithm.h"

extern uint8_t flag_path;
extern 	float dis, delta;
void UP_DOWN_Velt_Set(ST_Nav* pNav);
void SET_NAV_PATH_PERMUTATION(void);
void Path_Permutation(ST_VECTOR* Point_Start, ST_VECTOR* Point_Inc, ST_VECTOR* V_Start, ST_VECTOR* V_End,
		                        PATH_TYPE* Path_Type, float* A, float* R, float* T, uint8_t num_module);
void NavPosition(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void NavRotation(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void CheckPathEnd(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta);
void path_permutation_choose(ST_Nav *p_nav);
void Path_Permutation_Set_1_1(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //一区路径1
void Rotation_Permutation_Set_1(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav);
void Rotation_Permutation(float* Rotation_Start,float* W_Start,float* Rotation_Inc,float* W_End
													,float* A_W,float* T_W,uint8_t num_modele_w );
void Path_Permutation_Set_1_2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //一区路径2
 void Rotation_Permutation_Set_2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav);
void Path_Permutation_Set_1_0(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //一区到梅林1
void Path_Permutation_Set_1_3(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //一区到梅林3
void Path_Permutation_Set_1to2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //梅林1到2
void Path_Permutation_Set_3to2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //梅林3到2
void Rotation_Permutation_Set_3(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav);
void Path_Permutation_Set_3_10(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //梅林10到3区
void Path_Permutation_Set_3_12(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav); //梅林12到3区
void Path_Permutation_Set_3_retry(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav);

void path_point_choose(ST_Nav *p_nav);
void Point_Set_1(PATH_POINT *p_point);
void Point_to_Point(PATH_POINT *p_point);

void path_get_block(ST_Nav *p_nav);
void Point_Permutation(PATH_POINT *p_point);
#endif
