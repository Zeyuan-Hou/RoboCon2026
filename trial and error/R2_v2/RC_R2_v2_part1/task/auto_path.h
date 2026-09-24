#ifndef __AUTO_PATH_H__
#define __AUTO_PATH_H__

#include "global_declare.h"
#include "math.h"
#include "algorithm.h"

void UP_DOWN_Velt_Set(ST_Nav* pNav);
void SET_NAV_PATH_PERMUTATION(void);
void path_point_choose(ST_Nav *p_nav);
void Point_to_Point(PATH_POINT *p_point);

#endif
