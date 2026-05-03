#include "MathAlgorithm.h"
#include "ROBOT.h"
/*******************************************************************
PID Function
********************************************************************/

void PID_Init(ST_PID *pid, float p, float i, float d, float EMin, float EMax, float SumEMax, float UMax, float UpMax, float UdMax)
{
	pid->fpKp = p;
	pid->fpKi = i;
	pid->fpKd = d;
	pid->fpEMax = EMax;
	pid->fpEMin = EMin;
	pid->fpSumEMax = SumEMax;
	pid->fpUpMax = UpMax;
	pid->fpUdMax = UdMax;
	pid->fpUMax = UMax;
}
void PID_Calc_NEW(ST_PID *pStPID)
{
	pStPID->fpPreE = pStPID->fpE;
	pStPID->fpE = pStPID->fpDes - pStPID->fpFB;
	if (fabs(pStPID->fpE) <= pStPID->fpEMin)
	{
		pStPID->fpE = 0;
	}
	pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
	pStPID->fpSumE += pStPID->fpE;
	pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
	pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);
	pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
	pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);
	pStPID->fpU = pStPID->fpUp + pStPID->fpUi + pStPID->fpUd;
	pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}

void PID_Calc(ST_PID *pid, float fpDes, float fpFB)
{
	pid->fpDes = fpDes;
	pid->fpFB = fpFB;
	PID_Calc_NEW(pid);
}

/*******************************************************************
TD Function
********************************************************************/

float Sgn(float x)
{
	return (x > 0) - (x < 0);
}
void CalTD(ST_TD *pStTD)
{
	float d, d0, y, a0, a = 0, fhan;
	pStTD->x = pStTD->x1 - pStTD->aim;
	d = pStTD->r * pStTD->h;
	d0 = pStTD->h * d;
	y = pStTD->x + pStTD->h * pStTD->x2;
	a0 = sqrt(d * d + 8 * pStTD->r * fabs(y));

	if (fabs(y) > d0)
		a = pStTD->x2 + (a0 - d) * Sgn(y) / 2;
	else
		a = pStTD->x2 + y / pStTD->h;

	if (fabs(a) > d)
		fhan = -1 * pStTD->r * Sgn(a);
	else
		fhan = -1 * pStTD->r * a / d;

	pStTD->x1 += pStTD->T * pStTD->x2;
	pStTD->x2 += pStTD->T * fhan;
}

/*******************************************************************
limit float value by set fpMin and fpMax
********************************************************************/
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax)
{
	if (fpValue < fpMin)
	{
		return fpMin;
	}
	else if (fpValue > fpMax)
	{
		return fpMax;
	}
	else
	{
		return fpValue;
	}
}

/*******************************************************************
GENARATE SIGNAL
********************************************************************/
fp32 rampSignalFP(int32_t time, uint32_t whole_time)
{
	fp32 temp = (float)time / (float)whole_time;
	if (time <= 0)
	{
		return 0;
	}
	else if (time > 0 && time <= whole_time)
	{
		return temp;
	}
	else
	{
		return 1;
	}
}

fp32 curveSignalFP(int32_t time, uint32_t whole_time)
{
	fp32 temp = time / whole_time;
	if (time <= 0)
	{
		return 0;
	}
	else if (time > 0 && time <= whole_time)
	{
		return -2 * temp * temp * temp + 3 * temp * temp;
	}
	else
	{
		return 1;
	}
}

// Convert binary numbers to decimal numbers to control  the Air-operator
uint8_t bin_array_to_u8(uint8_t *bits)
{
	uint8_t result = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		result += bits[i] * (1 << i);
	}
	return result;
}
