#include "pid.h"
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

void PID_CascadeCalc(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback)
{
    pStCasPID->outer.fpDes = outer_target;
    pStCasPID->inner.fpFB = inner_feedback;
    pStCasPID->outer.fpFB = outer_feedback;
    
    PID_Calc(&pStCasPID->outer, pStCasPID->outer.fpDes, pStCasPID->outer.fpFB);
    PID_Calc(&pStCasPID->inner, pStCasPID->outer.fpU , pStCasPID->inner.fpFB);
    
    pStCasPID->output = pStCasPID->inner.fpU;
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
