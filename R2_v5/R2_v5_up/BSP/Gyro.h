#ifndef __GYRO_H__
#define __GYRO_H__

#include "Global_Variables.h"
#include <stddef.h>
#include <string.h>

/*------------------------------------------------------------------------------------------------------------*/
extern protocol_info_t g_output_info;

/*------------------------------------------------Functions declare--------------------------------------------*/
int analysis_data(unsigned char *data, short len);
void clear_data(int clr_len);

#endif
