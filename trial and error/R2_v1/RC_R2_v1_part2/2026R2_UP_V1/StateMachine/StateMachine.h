#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#define DM_SHIFT 2.57f
/*************************
						 气动板位置宏变量
							*********************************/

#define PULL 0
#define PUSH 1
#define LIL_SUCK 0
#define LIL_SPIT 1
#define BIG_SUCK 0
#define BIG_SPIT 1
#define PUMP_RUN 1
#define PUMP_STOP 0
#define OPEN 1
#define CLOSE 0

/*************************
						 伸缩臂位置宏变量
							*********************************/

#define UP_KFS 0
#define DOWN_KFS 1

#define SBSA_LK  -135
#define SBSA_DJI 0
#define SBSA_DM  -0.4f+2.57f

#define RTGK_LK  -75
#define RTGK_DJI -80
#define RTGK_DM  -0.4f+2.57f

//Get KFS From UP MeiLin
#define GKFU_LK  -32
#define GKFU_DJI 340
#define GKFU_DM  -0.1f+2.57f

//Get KFS From DOWN MeiLin
#define GKFD_LK  -19.5f
#define GKFD_DJI 320
#define GKFD_DM  -1.5+2.57f

//Leave KFS On UP MeiLin
#define LKOU_LK  -135
#define LKOU_DJI 280
#define LKOU_DM  0.78+2.57f

//Leave KFS On DOWN MeiLin
#define LKOD_LK  -135
#define LKOD_DJI 280
#define LKOD_DM  0.78+2.57f

//Handover KFS With DoubleArms
#define HKWD_LK  -110
#define HKWD_DJI -210
#define HKWD_DM  1.2f+2.57f

//Use KFS 
#define UK_LK  -90
#define UK_DJI 360
#define UK_DM  -0.876f+2.57f

#define HK_LK  -110
#define HK_DJI 0
#define HK_DM  -1.f+2.57f

/*************************
							双臂位置宏变量
							*********************************/

#define SBDA_L_ROLL 0
#define SBDA_L_PITCH 0
#define SBDA_R_ROLL 0
#define SBDA_R_PITCH 0
#define SBDA_L_J60 0.2f
#define SBDA_R_J60 -0.2f

#define RTGW_L_ROLL 90
#define RTGW_L_PITCH -80.f
#define RTGW_R_ROLL -90
#define RTGW_R_PITCH 81.f
#define RTGW_L_J60 1.54f
#define RTGW_R_J60 -1.49f

#define WC_L_ROLL -90
#define WC_L_PITCH -75
#define WC_R_ROLL 90
#define WC_R_PITCH 70
#define WC_L_J60 0.141f
#define WC_R_J60 -0.04f

#define RTSK_L_ROLL -180
#define RTSK_L_PITCH -63
#define RTSK_R_ROLL 180
#define RTSK_R_PITCH 60
#define RTSK_L_J60 1.106f
#define RTSK_R_J60 -1.016f


typedef enum{
	STANDBY_SA,
	READY_TO_GET_KFS,
	HOLD_KFS
}SA_STATE;
typedef enum{
	STANDBY_DA,
	READY_TO_GRAB_WEAPON,
	WAIT_COMBINE,
	READY_TO_STORAGE_KFS,
	STORAGE_KFS
}DA_STATE;
typedef enum{
	UNCPLT,
	CPLT
}CPLT_STATE;
typedef struct{
	SA_STATE SA_State;
	CPLT_STATE Cplt_State;
}SA;
typedef struct{
	DA_STATE DA_State;
	CPLT_STATE Cplt_State;
}DA;
typedef struct{
	DA DA;
	SA SA;
}TOTAL_STATE;
void stateMachine(void);
#endif

