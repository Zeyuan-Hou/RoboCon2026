#ifndef __DRIB_H__
#define __DRIB_H__

#include "air_operated.h"
#include "ROBOT.h"
#include "cmsis_os.h"

//#define Drib_Left_Inc 5000//6000//8000
//#define Drib_Right_Inc -4500//-5500//-6200
//#define Drib_Left_Rev -5000
//#define Drib_Right_Rev 4800
#define Load_Left 1000
#define Load_Right -1000
extern u8 start_from_drib;
void Dribble(void);
void Load_ball(void);
void Dribble_Load(void);
#endif
