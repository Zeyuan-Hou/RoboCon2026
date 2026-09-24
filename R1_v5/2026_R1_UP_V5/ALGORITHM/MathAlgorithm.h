#ifndef __MATHALGORITHM_H__
#define __MATHALGORITHM_H__

#include "math.h"
#include "Robot.h"

/*******************************************************************
PID Function
********************************************************************/				
						
void PID_Init(ST_PID *pid, float p, float i, float d, float EMin, float EMax, float SumEMax, float UMax, float UpMax, float UdMax);
void PID_Calc_NEW(ST_PID *pStPID);

void PID_Calc(ST_PID *pid,float fpDes,float fpFB);		
		
/*******************************************************************
TD Function
********************************************************************/

float Sgn(float x) ;
void CalTD(ST_TD *pStTD);

/*******************************************************************
limit float value by set fpMin and fpMax
********************************************************************/
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax);

/*******************************************************************
GENARATE SIGNAL
********************************************************************/
fp32 rampSignalFP(int32_t time,uint32_t whole_time);
fp32 rampSignalFP_1(int32_t time,uint32_t whole_time);
fp32 curveSignalFP(int32_t time,uint32_t whole_time);
fp32 newCurveSignalFP(int32_t time,uint32_t whole_time);
fp32 newCurveDotSignalFP(int32_t time,uint32_t whole_time);
//Convert binary numbers to decimal numbers to control  the Air-operator
void bin_array_to_u8(uint8_t *ori_bits,uint8_t *new_bits) ;

void LpFilter(ST_LPF *lpf);

void NotchFilter_Init(NotchFilter* f, float fc, float bw, float fs);
float NotchFilter_Update(NotchFilter* f, float input);

float StepSweep_Update(void);

void ZVD_Init(ZVD_Shaper *zvd, float wn, float zeta);
float ZVD_Run(ZVD_Shaper *zvd, float input);

void pack_2bit_data_simple(const uint8_t *src, uint8_t *dst);

void controlPushPull(u8 mode);

#endif
