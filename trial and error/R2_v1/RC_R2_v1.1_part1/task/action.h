#ifndef ACTION_H
#define ACTION_H

#include "global_declare.h"
#include "auto_path.h"
#include "board_communicate.h"
#include "vision.h"
#include "foot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h" 
extern int i;
void Action_choose(void);
void Part1_action(void);
void Part2_123_action(void);
void Part2_action(void);
void Part0_INIT(void);
void Part2_to_3(void);
void Part3_action(void);
void Part3_begin_2(void);
void Part3_begin(void);
void Part2_begin(void);
void Part_3_to_R1(void);
void Part_3_Retry(void);
#endif
