#ifndef __FOOT_H__
#define __FOOT_H__

#include "global_declare.h"
#include "algorithm.h"

#define Foot_Lengthen() {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);}
#define Foot_Shorten() {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1); HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);}

void Foot_WorkLoop(void);

#endif
