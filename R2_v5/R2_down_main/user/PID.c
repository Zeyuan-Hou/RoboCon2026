#include "PID.h"

void PID_Calc(ST_PID *pStPID, float target, float feedback)
{
    pStPID->fpPreE = pStPID->fpE;
    pStPID->fpDes = target;
    pStPID->fpFB = feedback; 

    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
    
    //死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpE = 0;
    }
		
		if (fabs(pStPID->fpE) < 25.0f)  // 阈值实测调整
			{
				pStPID->fpSumE += pStPID->fpE;
			}
			
		// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
		
		
    pStPID->fpSumE += pStPID->fpE; // 计算偏差累积

		// 计算总误差，并且限幅在[-fpSumEMax,fpSumEMax]里
		pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
		
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

		pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
		
    // 计算D项输出，并且限幅在[-fpUdMax,fpUdMax]里
    pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpFB - pStPID->fpPreFB), -pStPID->fpUdMax, pStPID->fpUdMax);
		pStPID->fpPreFB = pStPID->fpFB;
    // 计算总输出
    pStPID->fpU = pStPID->fpUp + pStPID->fpUi + pStPID->fpUd; 

    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}

void PID_Calc_NEW(ST_PID *pStPID)
{
    pStPID->fpPreE = pStPID->fpE;

    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
    
    //死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpE = 0;
    }
		
		// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
		
		
    pStPID->fpSumE += pStPID->fpE; // 计算偏差累积

		// 计算总误差，并且限幅在[-fpSumEMax,fpSumEMax]里
		pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
		
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

		pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
		
    // 计算D项输出，并且限幅在[-fpUdMax,fpUdMax]里
    pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

    // 计算总输出
    pStPID->fpU = pStPID->fpUp + pStPID->fpUi + pStPID->fpUd; 

    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}

void PID_CascadeCalc_special(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback)
{
    pStCasPID->outer.fpDes = outer_target;
		pStCasPID->outer.fpFB = outer_feedback;
    pStCasPID->inner.fpFB = inner_feedback;
    
    
    PID_Calc(&pStCasPID->outer, pStCasPID->outer.fpDes, pStCasPID->outer.fpFB);
    PID_Calc(&pStCasPID->inner, -pStCasPID->outer.fpU , pStCasPID->inner.fpFB);
    
    pStCasPID->output = pStCasPID->inner.fpU;
}

//void PID_Calc_NEW_wheel(ST_PID *pStPID)  //有前馈
//{

//	
//		// 前馈项计算
//    // 针对速度及加速度的前馈控制，分别使用当前目标值和目标值变化率进行计算
//		pStPID->friction = ClipFloat(400.f * pStPID->fpDes,-2400,2400);
//	
//		pStPID->fpUff = pStPID->fpDes * pStPID->fpKffv + (pStPID->fpDes - pStPID->prev_des) * pStPID->fpKffa + pStPID->friction + flag_wheel_slope * (pStPID->slope_forward + ClipFloat(robot_pos.fpPosQ *15000,-3000,3000));
//	
//		pStPID->prev_des = pStPID->fpDes;
//	
//    pStPID->fpPreE = pStPID->fpE;

//    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
//    
//    //死区，消除微小抖动
//    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
//    {       
//        pStPID->fpE = 0;
//    }
//		
//		//小目标防止车不动
//		if(fabs(pStPID->fpDes) <= 5&&fabs(pStPID->fpDes) >= 1){pStPID->flag_forward_precise = 1;}
//			else{pStPID->flag_forward_precise=0;}
//			
//		
//		// 误差限幅
//    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
//		
//		if (fabs(pStPID->fpE) < 40.0f)  // 阈值实测调整
//			{
//				pStPID->fpSumE += pStPID->fpE;
//			}
//		// 计算总误差，并且限幅在[-fpSumEMax,fpSumEMax]里
//		pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
//		
//    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
//    pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

//		pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
//		
//    // 计算D项输出，并且限幅在[-fpUdMax,fpUdMax]里
//    pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

//    // 计算总输出
//    pStPID->fpU = pStPID->fpUff + pStPID->fpUp + pStPID->fpUi + pStPID->fpUd + pStPID->fpforward_0 * Sgn(pStPID->fpDes) * pStPID->flag_forward_precise; 
//			
//    // PID运算总限幅
//    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
//		
//}




void PID_Calc_NEW_wheel(ST_PID *pStPID)  //无前馈
{

		// 前馈项计算
    // 针对速度及加速度的前馈控制，分别使用当前目标值和目标值变化率进行计算
		if(ramp_test_flag==1||path_state_3==8)
		{
				pStPID->friction = ClipFloat(100.f * pStPID->fpDes,-3000,3000);
		}
		else
		{
				pStPID->friction = ClipFloat(200.f * pStPID->fpDes,-4000,4000);
		}
//		if(test_forward==0&&(ramp_test_flag==1||path_state_3==8))
//		{
//				pStPID->friction=0.2*ClipFloat(100.f * pStPID->fpDes,-5500,5500);
//		}
		pStPID->fpUff = pStPID->fpDes * pStPID->fpKffv  + pStPID->friction;
	
	
		pStPID->prev_des = pStPID->fpDes;
	
    pStPID->fpPreE = pStPID->fpE;

    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
    
    //死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpE = 0;
    }
		
		//小目标防止车不动
		if(fabs(pStPID->fpDes) <= 5&&fabs(pStPID->fpDes) >= 1){pStPID->flag_forward_precise = 1;}
			else{pStPID->flag_forward_precise=0;}
			
		
		// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
		
		if (fabs(pStPID->fpE) < 80.0f)  // 阈值实测调整
			{
				pStPID->fpSumE += pStPID->fpE;
			}
		// 计算总误差，并且限幅在[-fpSumEMax,fpSumEMax]里
		pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
		
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

		pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
		
    // 计算D项输出，并且限幅在[-fpUdMax,fpUdMax]里
    pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

//    // 计算总输出
//			if(ramp_state==2)
//			{
//				pStPID->fpU_before = pStPID->fpUff + pStPID->fpUp + pStPID->fpUi + pStPID->fpUd + pStPID->fpforward_0 * Sgn(pStPID->fpDes) * pStPID->flag_forward_precise+pStPID->ramp_foward; 
//			}
//			else
//			{
				pStPID->fpU_before= pStPID->fpUff + pStPID->fpUp + pStPID->fpUi + pStPID->fpUd + pStPID->fpforward_0 * Sgn(pStPID->fpDes) * pStPID->flag_forward_precise; 
//			}
    
			
			
    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU_before, -pStPID->fpUMax, pStPID->fpUMax);
		
}

void PID_CascadeCalc(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback)
{
    pStCasPID->outer.fpDes = outer_target;
		pStCasPID->outer.fpFB = outer_feedback;
    pStCasPID->inner.fpFB = inner_feedback;
    
    
    PID_Calc(&pStCasPID->outer, pStCasPID->outer.fpDes, pStCasPID->outer.fpFB);
    PID_Calc(&pStCasPID->inner, pStCasPID->outer.fpU , pStCasPID->inner.fpFB);

    float out = pStCasPID->inner.fpU;
    pStCasPID->output = ClipFloat(out,-pStCasPID->inner.fpUMax,pStCasPID->inner.fpUMax);
}


void CalTD(ST_TD *pStTD)
{
    float d, d0, y, a0, a = 0, fhan;
    pStTD->x = pStTD->x1 - pStTD->aim;
    // 计算当前位置与目标位置的误差

    d = pStTD->r * pStTD->h; // 计算调节因子 d
    d0 = pStTD->h * d;       // 时间相关项 d0
    y = pStTD->x + pStTD->h * pStTD->x2;
    // 组合当前位置误差和速度误差

    a0 = sqrt(d * d + 8 * pStTD->r * fabs(y));
    // 计算带有非线性特性的中间量 a0

    if (fabs(y) > d0)
        a = pStTD->x2 + (a0 - d) * Sgn(y) / 2;
    // 误差较大时，采用非线性调节

    else
        a = pStTD->x2 + y / pStTD->h;
    // 误差较小时，采用线性调节

    if (fabs(a) > d)
        fhan = -1 * pStTD->r * Sgn(a);
    else
        fhan = -1 * pStTD->r * a / d;
    // 控制速度更新的方向和幅度

    pStTD->x1 += pStTD->T * pStTD->x2;
    // 更新位置

    pStTD->x2 += pStTD->T * fhan;
    // 更新速度
}
int Sgn(float x) 
{
    if (x > 0) {
        return 1;  
    } else if (x < 0) {
        return -1; 
    } else {
        return 0;   
    }
}

float Cal_Angle_Error(float current, float target) 
{
    float error = target - current;
    // 处理角度跃变，确保误差在[-180, 180)范围内
    if (error >= PI) 
	{
        error -= 2 * PI;
    } 
	else if (error < -PI) 
	{
        error += 2 * PI;
    }
    return error;
}
//位置环中的角度环

void PID_Calc_Angle(ST_PID *pid, float reference, float feedback)
{
	pid->fpPreE = pid->fpE;
    pid->fpDes = reference;
    pid->fpFB = feedback;
    pid->fpE = Cal_Angle_Error(pid->fpFB, pid->fpDes);

    if (fabs(pid->fpE) < pid->fpEMin)
    {
        pid->fpE = 0;
    }
    pid->fpE = ClipFloat(pid->fpE, -pid->fpEMax, pid->fpEMax);

    pid->fpSumE += pid->fpE;
    pid->fpSumE = ClipFloat(pid->fpSumE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = pid->fpE * pid->fpKp;
    pid->fpUp = ClipFloat(pid->fpUp, -pid->fpUpMax, pid->fpUpMax);

    pid->fpUd = (pid->fpE - pid->fpPreE) * pid->fpKd;
    pid->fpUd = ClipFloat(pid->fpUd, -pid->fpUdMax, pid->fpUdMax);

    pid->fpUi = pid->fpSumE * pid->fpKi;

    pid->fpU = pid->fpUp + pid->fpUd + pid->fpUi;
    pid->fpU = ClipFloat(pid->fpU, -pid->fpUMax, pid->fpUMax);
}

