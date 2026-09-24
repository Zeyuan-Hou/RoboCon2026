#ifndef VISION_H
#define VISION_H

#include "global_declare.h"
#include "board_communicate.h"
#include "locate.h"
#include "usart.h"


void Vision_Data_Deal(VISION_DATA_AUTO *p_vision_data_rec);
void Vision_Transmit(void);
#endif
