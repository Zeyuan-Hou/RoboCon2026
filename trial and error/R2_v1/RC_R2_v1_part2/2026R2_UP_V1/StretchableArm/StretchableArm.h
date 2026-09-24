#ifndef __STRETCHABLEARM_H__
#define __STRETCHABLEARM_H__
#include "stm32f4xx.h"

#define T2_WITHOUT_KFS_DM 0
#define T2_WITH_KFS_DM 0

#define K_WITHOUT_KFS 0.75	//original
#define K_WITH_KFS 0	//with KFS
#define B_WITHOUT_KFS 0
#define B_WITH_KFS 0
#define K_T_to_A 0

#define RAD0_DM -0.185//-11degree
#define RAD0_LK -0.719//-41degree

void gravityCompensation_SAtest(void);
void gravityCompensation_SA(uint8_t state);
#endif 
