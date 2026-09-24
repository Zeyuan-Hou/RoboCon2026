#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <string.h>
#include "Global_Variables.h"
#include "Debug.h"

void PackDataToZGT(void);
void UnpackDataFromZGT(void);
void receive_IR_feedback(void);

#endif
