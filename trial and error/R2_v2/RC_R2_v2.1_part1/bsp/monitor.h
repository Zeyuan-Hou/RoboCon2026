#ifndef __MONITOR_H__
#define __MONITOR_H__

#include "global_declare.h"
#include <math.h>

void System_Monitor(ST_SYSTEM_MONITOR *monitor);
void Monitor(uint16_t rps, uint16_t *error, uint16_t decision, uint16_t range);

#endif
