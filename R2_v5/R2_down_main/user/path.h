#ifndef _PATH_H_
#define _PATH_H_

#include "navigation.h"
#include "Type.h"
#include "math_algorithm.h"
#include "chassis.h"
#include "up_down_step.h"

/* 端头坐标  X,Y方向上DT35的距离 */
#define HEAD_1_X   	 61    /* 矛尖1 */
#define HEAD_1_Y     212

#define HEAD_2_X     63    /* 拳1 */
#define HEAD_2_Y     400

#define HEAD_3_X     62    /* 掌1 */
#define HEAD_3_Y     603

#define HEAD_4_X     61    /* 掌2 */
#define HEAD_4_Y     792

#define HEAD_5_X     59    /* 拳2 */
#define HEAD_5_Y     1000

#define HEAD_6_X     60
#define HEAD_6_Y     1196

#define HEAD_RAD 		0.01 				//夹头角度
//雷达粗略进入一个范围
#define HEAD_AREA_X  272		
#define HEAD_AREA_Y  -433
#define HEAD_AREA_RAD 0.00


//起点位置（大概）
#define start_x -265
#define start_y 57
#define start_rad 1.57

/* 对接位置 */
#define DOCK_X       -415
#define DOCK_Y       -636
#define DOCK_ANGLE    -3.12   /* 转180°面朝R1 */


//让路中间点
#define Move_Aside_X 386
#define Move_Aside_Y -495
#define Move_Aside_RAD 1.57





//从矛头架回到启动区  角度不变，面朝矛头架重试
#define Retry_Dock_X -265
#define Retry_Dock_Y 57
#define Retry_Dock_RAD 1.57





//二区取KFS角度
#define GET_KFS_RAD 1.57







/* 二区：梅林台阶和三个入口 */

/* 入口坐标（3个） */
#define ENTRY_1_X    -3097
#define ENTRY_1_Y    -2206

#define ENTRY_2_X    -1903
#define ENTRY_2_Y    -2230

#define ENTRY_3_X     -712
#define ENTRY_3_Y   -2193

//三个入口台阶在一区映射
#define ENTRY_ONE_AREA_1X -3074
#define ENTRY_ONE_AREA_1Y -975

#define ENTRY_ONE_AREA_2X -1904
#define ENTRY_ONE_AREA_2Y -991

#define ENTRY_ONE_AREA_3X -712
#define ENTRY_ONE_AREA_3Y -1099



//12号中点 x -715 y -8003 	w  1.57
//11号中点 x -1925 y -8003 w 1.586
//10号中点 x -3121 y -8003 w 1.58

/* 梅林11或12台阶结束点*/  //即10号台阶中点
#define MELIN_X_11      -3120
#define MELIN_Y_11      -8103
#define MELIN_ANGLE_11  1.58f

/* 梅林10台阶结束点*/     //约10号中点右400左右
#define MELIN_X_10      -3564
#define MELIN_Y_10     -8103
#define MELIN_ANGLE_10  1.58f


//斜坡起点中点 x -4155 y -8614

/* 上斜坡起点*/    //约中点偏左30cm
#define RAMP_STAER_X_10 -4055   //更靠中间10cm  //差值506
#define RAMP_STAER_x_11 -4055     //差值864
#define RAMP_STAER_Y -8814    //向上20cm  //差值903

/* 上斜坡终点 */
#define RAMP_END_Y       -10296  //结束前20cm
#define RAMP_END_Y_REAL  -10496  //结束

/*斜坡曲线终点 */
#define RAMP_CURVE_END_X  -2961    //差值1161
#define RAMP_CURVE_END_Y  -10766  //差值463


/* 三区 */
/* 九宫格中层三个格子 近 */
#define GRID_1_X     		 	455   /* 1列 */
#define GRID_1_Y     	 	 -9731
#define GRID_2_X      	  472   /* 2列 */
#define GRID_2_Y       	 -10274
#define GRID_3_X     		 	474   /* 3列 */
#define GRID_3_Y         -10817

#define GRID_RAD  3.133   //放置KFS角度



//启动区
//x -4373 y-10963 -3.14


//10 九宫启动
//x -4311 -6887 -3.14



void path_point_choose(ST_Nav *p_nav);
void Navigate_Task(void);
void all_path_logic(void);
void path_1(void);
void path_2(void);
void path_3(void);
void choose_action_arm(void);
void start_key(void);
void choose_point_only(void);
void path_4(void);
void choose_action_arm_repeat(void);
#endif
