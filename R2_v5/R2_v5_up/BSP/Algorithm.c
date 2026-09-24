#include "Algorithm.h"

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

void PID_Calc(ST_PID *pid, float fpDes, float fpFB)
{
	pid->fpDes = fpDes;
	pid->fpFB = fpFB;
	pid->fpPreE = pid->fpE;
	pid->fpE = pid->fpDes - pid->fpFB;

	if (fabs(pid->fpE) <= pid->fpEMin)
	{
		pid->fpE = 0;
	}
	pid->fpE = ClipFloat(pid->fpE, -pid->fpEMax, pid->fpEMax);

	pid->fpSumE += pid->fpE;
	pid->fpSumE = ClipFloat(pid->fpSumE, -pid->fpSumEMax, pid->fpSumEMax);

	pid->fpUp = ClipFloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
	pid->fpUi = pid->fpKi * pid->fpSumE;
	pid->fpUd = ClipFloat(pid->fpKd * (pid->fpE - pid->fpPreE), -pid->fpUdMax, pid->fpUdMax);

	pid->fpU = pid->fpUp + pid->fpUi + pid->fpUd;
	pid->fpU = ClipFloat(pid->fpU, -pid->fpUMax, pid->fpUMax);
}

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

float ClipFloat(float fpValue, float fpMin, float fpMax)
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

float linear_target_curve(float init_pos, float target_pos, int32_t tim, int32_t total_tim)
{
	float t = (float)tim / (float)total_tim;
	if (t <= 0)
	{
		return init_pos;
	}
	else if (t > 0 && t <= 1)
	{
		return init_pos + (target_pos - init_pos) * t;
	}
	else
	{
		return target_pos;
	}
}

float sin_target_curve(float init_pos, float target_pos, int32_t tim, int32_t total_tim)
{
	// 构造aim_pos关于时间的函数，将[current_pos, goal_pos]区间依照[-π/2, π/2]区间的sin函数曲线映射到[t0, t0+interval_ms]区间
	float t = (float)tim / (float)total_tim;
	if (t <= 0)
	{
		return init_pos;
	}
	else if (t > 0 && t <= 1)
	{
		return init_pos + (target_pos - init_pos) / 2.f * sinf(t * PI - PI / 2.f) + (target_pos - init_pos) / 2.f;
	}
	else
	{
		return target_pos;
	}
}

uint8_t bin_array_to_u8(uint8_t *bits)
{
	uint8_t result = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		result += bits[i] * (1 << i);
	}
	return result;
}
