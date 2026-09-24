#ifndef ___QD_H___
#define ___QD_H___

#include "robot.h"

#define DT35_LEFT_NUM 2
#define DT35_RIGHT_NUM 3
#define DT35_FRONT_NUM 1
#define DT35_BACK_NUM 0

void Dt35_DataReceive(QD_BOARD *qd , uint8_t *data);
void Ts_DataReceive(QD_BOARD *qd , uint8_t *data);











#endif // ___QD_H___







