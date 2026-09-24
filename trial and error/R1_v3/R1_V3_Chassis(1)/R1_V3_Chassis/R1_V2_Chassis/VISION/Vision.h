#ifndef __VISION_H__
#define __VISION_H__

#include "Types.h"
#include <string.h>
#include "usart.h"

void Vision_Data_Deal(ST_VISION_DATA* p_vision_data);
void LocationFilter(void);
void Vision_location(ST_Nav *p_nav);
void Vision_Transmit(void);
void Vision_Location(void);
#endif
