#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__
#include "Robot.h"

u8 update_ii_action(u8* refresh);
u8 autoUpdate_ii_upAction(u8* data,fp32 vel);
void StateMachine(void);
u16 serachRowAndLine(u32 ID);
u8 ableToGetKFS(u8 id,fp32 vel,u8 mode);
void update_KFS_msgs(u8 id);




#endif // __STATEMACHINE_H__
