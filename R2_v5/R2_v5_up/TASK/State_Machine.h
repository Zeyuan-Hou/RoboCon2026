#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include "Global_Variables.h"
#include "Algorithm.h"

#define Clamp_Up() { airOperator.airOperatorTxBuf[5] = 0; }
#define Clamp_Down() { airOperator.airOperatorTxBuf[5] = 1; }
#define Clamp_Open() { airOperator.airOperatorTxBuf[4] = 0; }
#define Clamp_Close() { airOperator.airOperatorTxBuf[4] = 1; }
#define Collect_KFS_Pull() { airOperator.airOperatorTxBuf[0] = 1, airOperator.airOperatorTxBuf[2] = 0; }
#define Collect_KFS_Push() { airOperator.airOperatorTxBuf[0] = 0, airOperator.airOperatorTxBuf[2] = 1; }
#define Collect_KFS_IDLE() { airOperator.airOperatorTxBuf[0] = 0, airOperator.airOperatorTxBuf[2] = 0; }
#define Store_KFS_Pull() { airOperator.airOperatorTxBuf[1] = 1, airOperator.airOperatorTxBuf[3] = 0; }
#define Store_KFS_Push() { airOperator.airOperatorTxBuf[1] = 0, airOperator.airOperatorTxBuf[3] = 1; }
#define Store_KFS_IDLE() { airOperator.airOperatorTxBuf[1] = 0, airOperator.airOperatorTxBuf[3] = 0; }

void StateMachine(void);

void Init_Task(void);
void Init_Pick_Task(void);
void Recover_Task(void);

void Get_Weapon_Task(void);
void Combine_Prepare_Task(void);
void Combine_End_Task(void);

void Collect_Prepare_Task(void);
void Collect_KFS_Task(void);
void Pull_KFS_Temporary_Task(void);
void Pull_KFS_Hold_Task(void);
void Throw_KFS_Task(void);
void Store_KFS_Task(void);

void Place_Mid_KFS_Task_Near(void);
void Place_Mid_KFS_Task_Far(void);
void Get_KFS_Behind_Task(void);
void Place_Top_KFS_Task_Near(void);
void Place_Top_KFS_Task_Far(void);
void Get_KFS_From_R1_Task(void);
void Get_KFS_From_R1_Back_Task(void);

void Re_Get_Weapon_Task(void);

void Transfer_For_400_Task(void);
void Transfer_For_Temporary_Task(void);
void Transfer_For_Top_Task(void);
void Transfer_From_Top_Task(void);

void Init_For_3_Task(void);

void Trial_Task(void);

void Get_Init_Pos(void);
void All_Push_Task(void);

#endif
