#include "DJI_motor.h"
#include "MathAlgorithm.h"
#include "ROBOT.h"

uint8_t DJIMotorStart(void)
{
	//set pid params 
	PID_Init(&move2006.motor_pid.outer,1.3f,0,0.2f,0.f,200.f,1000.f,60.f,60.f,0.f);
	PID_Init(&move2006.motor_pid.inner,140,3,20,0,60,1000,6000,5000,0);
	PID_Init(&friction3508.motor_pid.inner,240,0.05,14,0,800,1000,8000,8000,4000);
	PID_Init(&friction3508_sub.motor_pid.inner,240,0.05,14,0,800,1000,8000,8000,4000);
	PID_Init(&claw3508.motor_pid.outer,6.f,0.01f,0.6f,0.f,100.f,350.f,800.f,800.f,200.f);
	PID_Init(&claw3508.motor_pid.inner,200.f,0.1f,16.f,0,800,1000,8000,8000,4000);
	
	PID_Init(&wrist3508.motor_pid.outer,6.f,0.015f,1.f,0.f,100.f,800.f,400.f,800.f,100.f);
	PID_Init(&wrist3508.motor_pid.inner,240,0.05,14,0,800,1000,8000,8000,4000);
	
	PID_Init(&platform_L2006.motor_pid.outer,6.f,0.015f,0.1f,0.f,200.f,1000.f,800.f,800.f,400.f);
	PID_Init(&platform_L2006.motor_pid.inner,250,0,28,0,1600,1000,8000,8000,4000);
	PID_Init(&platform_R2006.motor_pid.outer,6.f,0.015f,0.1f,0.f,200.f,1000.f,800.f,800.f,400.f);
	PID_Init(&platform_R2006.motor_pid.inner,250,0,28,0,1600,1000,8000,8000,4000);
	return 1;
}

void DJIMotorControl(ST_DJI_MOTOR* motor)//电机控制
{
	// motor->motor_td.aim = motor->outerTarget;
	// CalTD(&motor->motor_td);
    // PID_Calc(&motor->motor_pid.outer,motor->motor_td.x1,motor->angle);
    PID_Calc(&motor->motor_pid.outer,motor->outerTarget,motor->angle);
		motor->innerTarget = motor->motor_pid.outer.fpU;
		PID_Calc(&motor->motor_pid.inner,motor->innerTarget,motor->anglev);
}
void DJIMotorControl_RecordingToForce(ST_DJI_MOTOR* motor,fp32 checkRange)//电机力反馈控制
{
	if(motor->angle>checkRange){
		if(motor->motor_current>7999){
			PID_Calc(&motor->motor_pid.inner,0,motor->anglev);
			motor->motor_current = motor->motor_pid.inner.fpU;
		}else{
			PID_Calc(&motor->motor_pid.inner,motor->innerTarget,motor->anglev);
			motor->motor_current = motor->motor_pid.inner.fpU;
		}
	}else{
		PID_Calc(&motor->motor_pid.inner,motor->innerTarget,motor->anglev);
    motor->motor_current = motor->motor_pid.inner.fpU;
	}
}
u8 DJIMotorControlByVision(ST_DJI_MOTOR *motor){
	u8 align=0;
	static u16 cnt=0;
	if(fabsf(deltaPos.pos_y)<0.0001f){
		motor->motor_pid.inner.fpU *=0.97f;
	}else{
		PID_Calc(&motor->motor_pid.outer,0,deltaPos.pos_y);
		motor->innerTarget =-motor->motor_pid.outer.fpU;
		PID_Calc(&motor->motor_pid.inner,motor->innerTarget,motor->anglev);
	}
	if(motor->angle<-580.f){
		motor->motor_pid.inner.fpU *=(motor->motor_pid.inner.fpU>0);
	}
	if(motor->angle>0.f){
		motor->motor_pid.inner.fpU *=(motor->motor_pid.inner.fpU<0);
	}
	if((deltaPos.pos_y>-0.6f&&deltaPos.pos_y<0.6f)&&(deltaPos.pos_y>0.0001f||deltaPos.pos_y<-0.0001f)){
		cnt++;
	}else{
		cnt=0;
	}
	if(cnt>500){
		align = 1;
	}
	return align;
}

void DJIVelControl(ST_DJI_MOTOR* motor)//电机控制
{
	// motor->motor_td.aim = motor->outerTarget;
	// CalTD(&motor->motor_td);
    // PID_Calc(&motor->motor_pid.outer,motor->motor_td.x1,motor->angle);
	PID_Calc(&motor->motor_pid.inner,motor->innerTarget,motor->anglev);
}
void DJIVelControlRemote(ST_DJI_MOTOR* motor)//电机控制
{
		fp32 vel=0;
	// motor->motor_td.aim = motor->outerTarget;
	// CalTD(&motor->motor_td);
    // PID_Calc(&motor->motor_pid.outer,motor->motor_td.x1,motor->angle);
		if(remoteRec.usJsRight_Y<500||remoteRec.usJsRight_Y>3500){
			vel=remoteRec.usJsRight_Y-2000;
			if(vel>0){
				vel=(fp32)(vel-1500)/500*200;
			}else{
				vel=(fp32)(vel+1500)/500*200;
			}
		}
		PID_Calc(&motor->motor_pid.inner,vel,motor->anglev);
}
/*****3508电机数据处理**********/
float GetEncoderNumber(ST_DJI_MOTOR* motor,uint8_t msg[8])
{
	motor->EncoderNum=(msg[0]<<8)|(msg[1]);
	return motor->EncoderNum;
}

float GetSpeed(uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value=0xFFFF;
	if(msg[2]&0x01<<7)
	{
		speed_temp=(base_value<<16|msg[2]<<8|msg[3]);
	}
	else
	{
		speed_temp=(msg[2]<<8)|(msg[3]);
	}
	return speed_temp;
}

float GetCurrent(uint8_t msg[8])
{
	int32_t speed_temp;
	int32_t base_value=0xFFFF;
	if(msg[4]&0x01<<7)
	{
		speed_temp=(base_value<<16|msg[4]<<8|msg[5]);
	}
	else
	{
		speed_temp=(msg[4]<<8)|(msg[5]);
	}
	return speed_temp;
}

void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value)
{
	pEncoder->siPreRawValue=pEncoder->siRawValue;
	pEncoder->siRawValue=value;
	pEncoder->siDiff=pEncoder->siRawValue-pEncoder->siPreRawValue;
	if(pEncoder->siDiff>(pEncoder->siNumber)/2)
	{
		pEncoder->siDiff-=pEncoder->siNumber;
	}
	else if(pEncoder->siDiff<-(pEncoder->siNumber)/2)
	{
		pEncoder->siDiff+=pEncoder->siNumber;
	}
	pEncoder->siSumValue+=pEncoder->siDiff;
	
}
