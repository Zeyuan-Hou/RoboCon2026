#ifndef __LOCATE_H__
#define __LOCATE_H__

#include "global_declare.h"

void Vision_location(ST_Nav *p_nav);
void LocationFilter(void);
void Vision_Data_Deal(VISION_DATA_AUTO *p_vision_data);

#endif
