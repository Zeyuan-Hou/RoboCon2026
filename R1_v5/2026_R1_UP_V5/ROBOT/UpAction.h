#ifndef __UP_ACTION_H__
#define __UP_ACTION_H__

#include "Robot.h"
//0
#define CLOSE_0 1
#define OPEN_0 0
//1
#define CLOSE_1 1
#define OPEN_1 0
//9
#define CLOSE_2 1
#define OPEN_2 0
//2
#define RUN_MAIN 1
#define STOP_MAIN 0
//3
#define SUCK_MAIN 0
#define SPIT_MAIN 1
//4
#define RUN_SUB_0 1
#define STOP_SUB_0 0
//5
#define SUCK_SUB_0 0
#define SPIT_SUB_0 1
//6
#define RUN_SUB_1 1
#define STOP_SUB_1 0
//7
#define SUCK_SUB_1 0
#define SPIT_SUB_1 1
//8
#define PUSH_OUT 1
#define PULL_IN 0


#define STANDBY_C3508 0.f
#define STANDBY_M2006 0.f
#define STANDBY_F3508 0.f

#define INIT_C3508 10.f
#define INIT_M2006 -290.f
#define INIT_F3508 0.f

#define GET_WEAPON_C3508 73.f
#define GRAB_HALFWEAPON_C3508 90.f
#define STORE_WEAPON_C3508 181.f

#define COMBINE_WEAPON_F3508 (-250)
#define USE_WEAPON_F3508 (-500)

#define STANDBY_SJ60 -0.1f
#define STANDBY_EJ60 -0.1f
#define STANDBY_W3508 0.f
#define STANDBY_GJ60 0.0226f

#define INIT_SJ60 -0.68f
#define INIT_EJ60 0.32f
#define INIT_W3508 -200.f
#define INIT_GJ60 0.0226f

#define GET_GROUND_KFS_SJ60 -1.89f
#define GET_GROUND_KFS_EJ60 0.57f
#define GET_GROUND_KFS_W3508 -410.f

#define GET_LOW_KFS_SJ60 -2.03f
#define RAD1_LOW 0.605f
#define GET_LOW_KFS_EJ60 0.77f//1.25f
#define RAD2_LOW -0.929f
#define GET_LOW_KFS_W3508 -200.f

#define GET_HIGH_KFS_SJ60 -1.72f
#define RAD1_HIGH 0.895f
#define GET_HIGH_KFS_EJ60 0.42f//0.71f
#define RAD2_HIGH -0.389f
#define GET_HIGH_KFS_W3508 -200.f

#define GET_TOP_KFS_SJ60 -1.5f
#define RAD1_TOP 1.205f
#define GET_TOP_KFS_EJ60 0.1f//0.28f
#define RAD2_TOP 0.041f
#define GET_TOP_KFS_W3508 -200.f

#define HOLD_KFS_SJ60 -0.64f
#define HOLD_KFS_EJ60 0.37f//0.29f
#define HOLD_KFS_W3508 0.f

//#define GET_STORED_KFS_SJ60 -0.73f//-0.654f
//#define GET_STORED_KFS_EJ60 -0.1f//0.193f
//#define GET_STORED_KFS_W3508 -193.f
//#define GET_SUB0_KFS_GJ60 -2.63f
//#define GET_SUB1_KFS_GJ60 2.52f
#define GET_STORED_KFS_SJ60 -0.35f//-1.f//-0.654f
#define GET_STORED_KFS_EJ60 0.007f//0//0.193f
#define GET_STORED_KFS_W3508 -185.f//-160.f
#define GET_SUB0_KFS_GJ60 2.24f// 2.2f//2.5f
#define GET_SUB1_KFS_GJ60 -2.18f//-2.62f

#define USE_KFS_SJ60 -1.55f
#define USE_KFS_EJ60 0.37f//0.55fs
#define USE_KFS_W3508 -185.f
#define USE_KFS_GJ60_L -1.57f
#define USE_KFS_GJ60_R 1.59f

#define HANDOVER_KFS_SJ60 -0.07f
#define HANDOVER_KFS_EJ60 -0.52f
#define HANDOVER_KFS_W3508 9.5f
#define HANDOVER_KFS_GJ60 -0.134f//2.61f
#define HANDOVER_KFS_GJ60_RIGHT 0.14f

#define STANDBY_L2006 0
#define STANDBY_R2006 0
#define IN_L2006 -20
#define IN_R2006 20
#define OUT_L2006 1370
#define OUT_R2006 -1370

void UpAction_weapon(void);

void init_get_weapon(void);
void deinit_get_weapon(void);
void get_weapon(void);
void store_weapon(void);
void combine_weapon(void);
void move_weapon_out(void);
void use_weapon(void);

u8 checkPosition(void);
void UpAction_KFS(void);

void init_KFS(void);
void deinit_KFS(void);
void get_and_store_KFS(void);
void get_KFS(void);
void get_stored_KFS(void);
void store_KFS(void);
void use_KFS(void);
void handover_KFS(void);

void UpAction_platform(void);
void init_platform(void);
void platform_move_in(void);
void platform_move_out(void);
#endif // __UP_ACTION_H__
