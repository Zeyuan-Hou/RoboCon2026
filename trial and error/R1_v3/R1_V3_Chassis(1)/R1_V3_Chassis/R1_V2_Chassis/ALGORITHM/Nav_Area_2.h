#ifndef __NAV_AREA_2_H__
#define __NAV_AREA_2_H__

#include "Nav_algorithm.h"
#include "Types.h"


void NavArcMoving(Arc *arc,ST_Nav *p_nav);
uint16_t divide(POINT* point);
void Nav_Area_2_Task(ST_Nav *p_nav);
int8_t Spot_Judge(void);
uint16_t exchange_spot(uint16_t spot);



#endif // __NAV_AREA_2_H__
