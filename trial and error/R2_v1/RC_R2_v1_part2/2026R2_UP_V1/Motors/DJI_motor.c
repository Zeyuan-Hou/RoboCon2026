#include "DJI_motor.h"
#include "pid_algorithm.h"
/**DJI_Func**/
/****
Input:motor's target pos
Output:current sent to motor
Brief:use Cascade PID
****/
uint8_t DJI_Pos_Ctrl(ST_DJI_MOTOR *pstMotor,fp32 tPos)
{
	pstMotor->motor_td.aim=tPos;
	CalTD(&pstMotor->motor_td);
	PID_Calc(&pstMotor->motor_pid.outer,pstMotor->motor_td.x1,pstMotor->angle);
	PID_Calc(&pstMotor->motor_pid.inner,pstMotor->motor_pid.outer.fpU,pstMotor->anglev);
	if(fabsf(pstMotor->angle-tPos)<0.8f){
		return 1;
	}else{
		return 0;
	}
}
uint8_t DJI_Pos_CtrlWithoutPID(ST_DJI_MOTOR *pstMotor,fp32 tPos){
	pstMotor->outerTarget=tPos;
	if(fabsf(pstMotor->angle-tPos)<1.5f){
		return 1;
	}else{
		return 0;
	}
}
uint8_t DJIMotorStart(void)
{
	//set pid params 
	PID_Init(&left_2006_1.motor_pid.outer,8.f,0.05f,0.1f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&left_2006_1.motor_pid.inner,250,0,25,0,800,1000,8000,8000,4000);
	PID_Init(&left_2006_2.motor_pid.outer,8.3f,0.05f,0.11f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&left_2006_2.motor_pid.inner,240,0,26,0,800,1000,8000,8000,4000);
	PID_Init(&right_2006_1.motor_pid.outer,10.f,0.05f,0.1f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&right_2006_1.motor_pid.inner,340,0,35,0,800,1000,8000,8000,4000);
	PID_Init(&right_2006_2.motor_pid.outer,10.f,0.05f,0.1f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&right_2006_2.motor_pid.inner,340,0,35,0,800,1000,8000,8000,4000);
	PID_Init(&stretch_2006.motor_pid.outer,8.f,0.05f,0.1f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&stretch_2006.motor_pid.inner,300,0,30,0,800,1000,8000,8000,4000);
	return 1;
}


/************Two Aris Wrist******************/
uint8_t wristCtrl_L(fp32 roll_t,fp32 pitch_t,lilWrist* pstWrist){
	pstWrist->pitchCtrl=pitch_t;
	pstWrist->rollCtrl=roll_t;
	pstWrist->pitchRec=0.5f*(pstWrist->motor1->angle-pstWrist->motor2->angle);
	pstWrist->rollRec=0.25f*(pstWrist->motor1->angle+pstWrist->motor2->angle);
	if(fabsf(pstWrist->motor1->angle-2*roll_t-pitch_t)<1.f&&fabsf(pstWrist->motor2->angle-2*roll_t+pitch_t)<1.f){
		return 1;
	}else{
		return 0;
	}
};
uint8_t wristCtrl_R(fp32 roll_t,fp32 pitch_t,lilWrist* pstWrist){
	pstWrist->pitchCtrl=pitch_t;
	pstWrist->rollCtrl=roll_t;
	pstWrist->pitchRec=0.5f*(pstWrist->motor1->angle-pstWrist->motor2->angle);
	pstWrist->rollRec=0.25f*(pstWrist->motor1->angle+pstWrist->motor2->angle);
	if(fabsf(pstWrist->motor1->angle-2*roll_t-pitch_t)<0.5f&&fabsf(pstWrist->motor2->angle-2*roll_t+pitch_t)<0.5f){
		return 1;
	}else{
		return 0;
	}
};

