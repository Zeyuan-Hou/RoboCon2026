#ifndef __DOUBLEARM_H__
#define __DOUBLEARM_H__
#include "Robot.h"

#define T_ORIGINAL_J60					0
#define T_WITH_WEAPON_J60 			0
#define T_WITH_KFS_J60					0
#define RAD_ORIGINAL_LEFT   		0
#define RAD_WITH_WEAPON_LEFT 		0
#define RAD_WITH_KFS_LEFT   		0
#define RAD_ORIGINAL_RIGHT   		0
#define RAD_WITH_WEAPON_RIGHT 	0
#define RAD_WITH_KFS_RIGHT   		0

void gravityCompensation_DAtest(uint8_t state);
void gravityCompensation_DA(uint8_t state);

#endif
