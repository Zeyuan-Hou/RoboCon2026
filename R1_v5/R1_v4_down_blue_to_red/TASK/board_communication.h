#ifndef ___BOARD_COMMUNICATION_H
#define ___BOARD_COMMUNICATION_H

#include "robot.h"
#include "route_plan.h"
#include "navigation.h"



void Receive_from_Upper(uint8_t *data);
void Send_to_Upper(void);
void Send_to_Vofa(void);
void Process_from_upper(void);












#endif // ___BOARD_COMMUNICATION_H





