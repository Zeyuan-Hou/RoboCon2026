#include "Robot.h"
u8 delay_timer_flag=0;
u32 delay_timer=0;
u8 CAN1_RxBuf[8],CAN1_TxBuf[8],CAN2_RxBuf[8],CAN2_TxBuf[8],CAN3_RxBuf[8],CAN3_TxBuf[8];

u8 uart4_rxbuf[20],uart4_txbuf[23],//板件通讯
		usart1_rxbuf[122],vision_rec[122],usart1_txbuf[3],//视觉
		usart2_rxbuf[13],usart2_txbuf[24],//遥控器
		IRModuleTxBuffer[9]={0},IRModuleRxBuffer[9]={0};
ST_JS_VALUE remoteRec;
ST_VEL remote_vel;
ST_POS robotPos,deltaPos;
//use for whole robot

u8 part_flag=10,part_step=0,lil_step=0,refresh=0,part1_refresh=0,part2_refresh=0;
u32 timer=0;


//use for up action
u8 get_KFS_height=0;//0:ground 1:low 2:high 3:top
u8 store_KFS_orientation=0;//0:sub0 1:sub1
u8 use_KFS_orientaton=0;//0:left 1:right
u8 get_KFS_ID=0;//1~12
u8 use_KFS_ID=0;//0:left 1:straight 2:right

u8 weapon_compensation_mode=0,KFS_compensation_mode=0;
u8 weapon_player_action=KEEP_QUIET;
u8 KFS_master_action=KEEP_QUIET;
u8 platform_action=KEEP_QUIET;
u8 weapon_player_state=STAND_BY;
u8 KFS_master_state=STAND_BY;
u8 platform_state=STAND_BY;

u8 QD_show_1=0,//0:无意义 1~6：取头1~6 7：结束对接
    QD_show_2=0,//0:无意义  8：等待5s 9：等待10s
		QD_show_final_1=0,QD_show_final_2;//让视觉展示

II_ROBOT ii_robot={0};
u8 get_KFS_source=0;//1:预输入，0：一个个输
u8 input_mode=1;//0:不接受输入值，1：接受输入值

u8 weapon_cplt=1,KFS_cplt=1,platform_cplt=1;

u8 complete_weapon=0,complete_KFS=0,complete_platform=0,chassis_move=0;

u8 move2006_mode=0,move2006_align=0;//0:角度环 1：视觉环

u8 friction3508_mode=0;//0：程序控 1：遥控器控


//communicate with chassis
u8 chassis_action,//0:STANDBY 1:INIT 2:MANUAL 3:NAV 4:LOCK 5:Nav By QRCode
    nav_cplt,//同3代r2：导航完成后，下层通知上层一次，使其置1
    weapon_num,cmf=0;
fp32 chassis_vel_rec1=0,chassis_vel_rec2=0;
fp32 nav_leaved_time=0;
fp32 nav_progress;//0~1：导航进程 2：导航完成
u16 nav_target,pre_target;//同3代r2：0x00000
u8 get_start=0,get_allowed=0;//开始做吸块准备动作

//use for compensation
fp32 shoulder_angle, elbow_angle, wrist_angle, gimbal_angle,weapon_wrist_angle,weapon_move_angle,weapon_push_angle,platform_angle;
fp32 shoulder_torque, elbow_torque, wrist_torque, gimbal_torque,weapon_wrist_torque;
u32 sm_cnt1, sm_cnt2, sm_cnt3;

//Motor Define
//DJI Motors
ST_DJI_MOTOR move2006 ={.motor_encoder={.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x201, .getStartPos = 0},
			claw3508 ={.motor_encoder={ .siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x202, .getStartPos = 0},
			friction3508 ={.motor_encoder={ .siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x203, .getStartPos = 0},
			friction3508_sub ={.motor_encoder={ .siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x203, .getStartPos = 0},
			wrist3508 ={.motor_encoder={ .siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x204, .getStartPos = 0},
			//CAN2
			platform_L2006 ={.motor_encoder={.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x201, .getStartPos = 0},
			platform_R2006 ={.motor_encoder={.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x202, .getStartPos = 0};

u8 claw_mode=0,claw_pin=0;
//J60 Motors
DEEP_MOTOR gimbalJ60={.motor_id_=0x01,.MaxPos=MY_J60_MAX_POS,.MinPos=MY_J60_MIN_POS},
						//CAN2
						shoulderJ60={.motor_id_=0x01,.MaxPos=MY_J60_MAX_POS,.MinPos=MY_J60_MIN_POS},
						elbowJ60={.motor_id_=0x02,.MaxPos=MY_J60_MAX_POS,.MinPos=MY_J60_MIN_POS};
MotorCMD gimbalJ60CMD,shoulderJ60CMD,elbowJ60CMD;
MotorDATA gimbalJ60Receive,shoulderJ60Receive,elbowJ60Receive;
fp32 gimbal_torque=0,shoulder_torque=0,elbow_torque=0;
ST_TD gimbalTD={
	.x1=0,
	.x2=0,
	.x=0,
	.r=1,
	.h=1,
	.T=0.001f,
	.aim=0
};
NotchFilter gimbalFilter={0};
//Air_Operator						
AIR_OPERATOR airOperator={0};
u8 left_limit=0,right_limit=0;

SYSTEM_MONITOR systemMonitor;				
u8 upFpsError,downFpsError;

#define DT35_FRONT_K 0.7423656138137f
#define DT35_FRONT_B 16.0673827302061f
#define DT35_BACK_K 0.6863957502845f
#define DT35_BACK_B -8.7496533087917f
#define DT35_LEFT_K 0.9900092710861f
#define DT35_LEFT_B -147.8026725701187f
#define DT35_RIGHT_K 0.9934274054964f
#define DT35_RIGHT_B -116.1475367779938f

void Dt35_DataReceive(AIR_OPERATOR *qd , uint8_t *data){
//		static uint16_t pre_voltage[4];
    for(uint8_t i=0;i<4;i++){
        memcpy(qd->dt35_origin+i,data+i*2,2);
//				if(qd->dt35[i] - pre_voltage[i]<100 && qd->dt35[i] - pre_voltage[i]>-100) pre_voltage[i] = qd->dt35[i];
//				else qd->dt35[i] = pre_voltage[i];
    }
    qd->dt35_front = DT35_FRONT_K*qd->dt35_origin[3]+DT35_FRONT_B;
    qd->dt35_back = DT35_BACK_K*qd->dt35_origin[1]+DT35_BACK_B;
    qd->dt35_left = DT35_LEFT_K*qd->dt35_origin[2]+DT35_LEFT_B;
    qd->dt35_right = DT35_RIGHT_K*qd->dt35_origin[0]+DT35_RIGHT_B;
		qd->dt35[3]=qd->dt35_front;
		qd->dt35[1]=qd->dt35_back;
		qd->dt35[2]=qd->dt35_left;
		qd->dt35[0]=qd->dt35_right;
}
