#include "Robot.h"
uint8_t action_DA ,  action_SA  ,height_flag,new_height_flag/*判断方块放到上方还是下方或从上方还是下方取*/;
/**
0:回复初始动作			|		 恢复初始动作
1:准备取武器头			|		 准备取方块
2:左臂开始取武器头	|		 取方块
3.右臂开始取武器头	|	   扔方块
4:准备存方块				|		 存方块
5:存方块						|		 放中层方块
6:NONE						|		 取存着的方块
7:NONE						|		 转向后扔方块
8:NONE						|		 站起后取方块
11:NONE						|		 放上层方块feedback_SA = 12 when finished
**/
uint8_t feedback_DA,feedback_SA;
//equal to n,doing action n;equal to n*10,have done action n;
//etc,"feedback_DA==4" means DoubleArm is getting ready to storage KFS,
//while "feedBack_DA==40" means DoubleArm is ready to storage KFS

//system monitor
SYSTEM_MONITOR system_monitor;
//CAN Communicate
uint8_t CAN1_RxBuf[8],CAN1_TxBuf[8],CAN2_RxBuf[8],CAN2_TxBuf[8];
//Within Board Communicate
uint8_t RxBufFromZGT[17],TxBufToZGT[17];
uint8_t Tx_Completed_Flag=1;
uint8_t leaveKFSFlag;
//gravity Compensation
uint8_t gravityCompensation_DA_state;
uint8_t gravityCompensation_SA_state;

//AirOperator Control
uint8_t AirOperaterCtrlBuf[6] = {0};
uint8_t AirOperaterCtrl[1]={0};
//J60 variables
DEEP_MOTOR leftShoulder={.motor_id_=0x01,.MaxPos=MY_J60_MAX_POS,.MinPos=MY_J60_MIN_POS},
					rightShoulder={.motor_id_=0x02,.MaxPos=MY_J60_MAX_POS,.MinPos=MY_J60_MIN_POS};
MotorCMD leftShoulderCMD,rightShoulderCMD;
MotorDATA leftShoulderReceive,rightShoulderReceive;
fp32 gTorqueLeft,gTorqueRight;
//DJI variables
ST_DJI_MOTOR left_2006_1={.motor_encoder={ .siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x201, .getStartPos = 0},
						left_2006_2={.motor_encoder={ .siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x202, .getStartPos = 0},
						right_2006_1={.motor_encoder={ .siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x203, .getStartPos = 0},
						right_2006_2={.motor_encoder={ .siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=8000,.h=0.1,.T=0.001}, .id=0x204, .getStartPos = 0},
						stretch_2006={.motor_encoder={ .siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r=10000,.h=0.01,.T=0.001}, .id=0x205, .getStartPos = 0};
//int16_t can2_cur1,can2_cur2,can2_cur3,can2_cur4,can2_cur5;
lilWrist wrist_L={.motor1 = &left_2006_1,.motor2 = &left_2006_2};
fp32 roll_L,pitch_L;
lilWrist wrist_R={.motor1 = &right_2006_1,.motor2 = &right_2006_2};
fp32 roll_R,pitch_R;
//DM variables
Motor_DM stretch_DM={

	.id = 0x01,
	.mst_id = 0x11,
	.ctrl.mode 	= 1,
	.ctrl.vel_set 	= 0.0f,
	.ctrl.pos_set 	= 0.0f,
	.ctrl.kd_set 	= 0.0f,
	.ctrl.kp_set= 0.0f
};
fp32 gTorqueDM;
//LK variable
Motor_LK_8016 stretch_LK = {.LK_ID = 0x141,.LK_motor_encoder = {.siNumber = 65536.f, .siSumValue = 0, .siRawValue = 0,.state=1}};
ST_TD LK1_TD = {0,0,0,   2500,      0.004,   0.001,  0} ;
int16_t /*can1_cur_LK,*/g_cur_LK;
/**Common Func**/			
						
/*******************************************************************************************
LPF function
*******************************************************************************************/
void LpFilter(ST_LPF *lpf)
{
    
    // fir_a = 1 / (1 + 截止频率 * 采样时间)
		//fir_a越大，滤波后信号与原始信号越接近；
		//fir_a越小，滤波后信号越平滑，但延迟也越大；
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);

    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;

    lpf->preout = lpf->out;
}

/*******************************************************************
get absolute value
********************************************************************/
int32_t my_intabs(int32_t num)
{
	if(num >= 0)
	{
		return num;
	}
	else 
	{
		return -num;
	}
}
fp32 my_fp32abs(fp32 num)
{
	if(num >= 0)
	{
		return num;
	}
	else 
	{
		return -num;
	}
}
/*******************************************************************
limit angle(RAD) in [-PI,PI]
********************************************************************/
fp32 ConvertAngle(fp32 fpAngA)
{
    do
    {
        if (fpAngA >= PI)
        {
            fpAngA -= PI2;
        }
        else if (fpAngA < -PI)
        {
            fpAngA += PI2;
        }
    } while (fpAngA >= PI || fpAngA < -PI);
    return fpAngA;
}
/*******************************************************************
limit float value by set fpMin and fpMax
********************************************************************/
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax)
{
    if(fpValue < fpMin)
    {
        return fpMin;
    }
    else if(fpValue > fpMax)
    {
        return fpMax;
    }
    else
    {
        return fpValue;
    }
}

/*******************************************************************
GENARATE RAMP SIGNAL
********************************************************************/
fp32 rampSignalFP(fp32 start,fp32 end,uint32_t time,uint32_t whole_time){
	fp32 temp = start+(end-start)*time/whole_time;
	if(time<=0){
		return start;
	}else if(time>0&&time<=whole_time){
		return temp;
	}else{
		return end;
	}
}

/*******************************************************************
GENARATE CURVE SIGNAL
********************************************************************/
fp32 curveSignalFP(fp32 start,fp32 end,uint32_t time,uint32_t whole_time){
	fp32 a = 2*(start-end)/(whole_time*whole_time);
	fp32 u0=end-start;
	if(time>0&&time<=whole_time/2){
		return start+a*time*time;
	}else if(time>whole_time/2&&time<=whole_time){
		return start + u0 - a*(time-whole_time)*(time-whole_time);
	}else if(time>whole_time){
		return end;
	}else{
		return start;
	}
}


/**************气泵*****************/
void pushAndPull(uint8_t flag1,uint8_t flag2){
	if(flag2 == 0){
		switch(flag1){
			case 1:
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
				break;
			case 0:
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
				break;
			default:
				break;
		}
	}else{
		switch(flag1){
			case 1:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
				break;
			case 0:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
				break;
			default:
				break;
		}
	}
}

//Convert binary numbers to decimal numbers to control  the Air-operator
uint8_t bin_array_to_u8(uint8_t *bits) {  
    uint8_t result = 0;
    for (uint8_t i = 0; i < 6; i++) {
        result += bits[i] * (1 << i); 
    }
    return result; 
}

