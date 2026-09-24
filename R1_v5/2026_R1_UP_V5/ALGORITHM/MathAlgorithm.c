#include "MathAlgorithm.h"

/*******************************************************************
PID Function
********************************************************************/				
						
void PID_Init(ST_PID *pid, float p, float i, float d, float EMin, float EMax, float SumEMax, float UMax, float UpMax, float UdMax)
{
    pid->fpKp = p;
    pid->fpKi = i;
    pid->fpKd= d;
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

void PID_Calc(ST_PID *pid,float fpDes,float fpFB)
{
	pid->fpDes = fpDes;
	pid->fpFB = fpFB;
	PID_Calc_NEW(pid);
}						
		
/*******************************************************************
TD Function
********************************************************************/

float Sgn(float x) {
    return (x > 0) - (x < 0);
}
void CalTD(ST_TD *pStTD)
{
	float d,d0,y,a0,a=0,fhan;
	pStTD->x = pStTD->x1 - pStTD->aim;
	d = pStTD->r * pStTD->h;
	d0 = pStTD->h * d;
	y = pStTD->x + pStTD->h * pStTD->x2;
	a0 = sqrt(d * d + 8 * pStTD->r * fabs(y));

	if(fabs(y) > d0)
		a = pStTD->x2 + (a0 - d) * Sgn(y) / 2;
	else
		a = pStTD->x2 + y / pStTD->h;

	if(fabs(a) > d)
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
GENARATE SIGNAL
********************************************************************/
fp32 rampSignalFP(int32_t time,uint32_t whole_time){
	fp32 temp = (float)time/(float)whole_time;
	if(time<=0){
		return 0;
	}else if(time>0&&time<=whole_time){
		return temp;
	}else{
		return 1;
	}
}

fp32 rampSignalFP_1(int32_t time,uint32_t whole_time){
	fp32 temp = (float)time/(float)whole_time;
	if(time<=0){
		return 0;
	}else if(time>0&&time<=whole_time/2){
		return 1.5f*temp;
	}else if(time>whole_time/2&&time<=whole_time){
		return 0.5f*(temp-0.5f)+0.75f;
	}else{
		return 1;
	}
}

fp32 curveSignalFP(int32_t time,uint32_t whole_time){
    fp32 temp =(float)time/(float)whole_time;
	if(time<=0){
		return 0;
	}else if(time>0&&time<=whole_time){
		return -2*temp*temp*temp+3*temp*temp;
	}else{
		return 1;
	}
}

fp32 newCurveSignalFP(int32_t time,uint32_t whole_time){
    fp32 temp = (float)time/(float)whole_time;
	if(time<=0){
		return 0;
	}else if(time>0&&time<=whole_time){
		return 6*temp*temp*temp*temp*temp-15*temp*temp*temp*temp+10*temp*temp*temp;
	}else{
		return 1;
	}
}
fp32 newCurveDotSignalFP(int32_t time,uint32_t whole_time){
    fp32 temp = (float)time/(float)whole_time;
	if(time<=0){
		return 0;
	}else if(time>0&&time<=whole_time){
		return 30*temp*temp*temp*temp-60*temp*temp*temp*temp+30*temp*temp;
	}else{
		return 1;
	}
}
//Convert binary numbers to decimal numbers to control  the Air-operator
void bin_array_to_u8(uint8_t *ori_bits,uint8_t *new_bits){  
		new_bits[0]=0;
		new_bits[1]=0;
		new_bits[2]=0;
    switch(ori_bits[3]){
			case 0:
				new_bits[0]+=0;
				break;
			case 1:
				new_bits[0]+=128;
				break;
			case 2:
				new_bits[0]+=64;
				break;
			default:
				break;
		}
		switch(ori_bits[2]){
			case 0:
				new_bits[0]+=0;
				break;
			case 1:
				new_bits[0]+=32;
				break;
			case 2:
				new_bits[0]+=16;
				break;
			default:
				break;
		}
		switch(ori_bits[1]){
			case 0:
				new_bits[0]+=0;
				break;
			case 1:
				new_bits[0]+=8;
				break;
			case 2:
				new_bits[0]+=4;
				break;
			default:
				break;
		}
		switch(ori_bits[0]){
			case 0:
				new_bits[0]+=0;
				break;
			case 1:
				new_bits[0]+=2;
				break;
			case 2:
				new_bits[0]+=1;
				break;
			default:
				break;
		}
		switch(ori_bits[7]){
			case 0:
				new_bits[1]+=0;
				break;
			case 1:
				new_bits[1]+=128;
				break;
			case 2:
				new_bits[1]+=64;
				break;
			default:
				break;
		}
		switch(ori_bits[6]){
			case 0:
				new_bits[1]+=0;
				break;
			case 1:
				new_bits[1]+=32;
				break;
			case 2:
				new_bits[1]+=16;
				break;
			default:
				break;
		}
		switch(ori_bits[5]){
			case 0:
				new_bits[1]+=0;
				break;
			case 1:
				new_bits[1]+=8;
				break;
			case 2:
				new_bits[1]+=4;
				break;
			default:
				break;
		}
		switch(ori_bits[4]){
			case 0:
				new_bits[1]+=0;
				break;
			case 1:
				new_bits[1]+=2;
				break;
			case 2:
				new_bits[1]+=1;
				break;
			default:
				break;
		}
		switch(ori_bits[9]){
			case 0:
				new_bits[2]+=0;
				break;
			case 1:
				new_bits[2]+=8;
				break;
			case 2:
				new_bits[2]+=4;
				break;
			default:
				break;
		}
		switch(ori_bits[8]){
			case 0:
				new_bits[2]+=0;
				break;
			case 1:
				new_bits[2]+=2;
				break;
			case 2:
				new_bits[2]+=1;
				break;
			default:
				break;
		}
		
}

void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * 2 * PI * lpf->samp_tim);
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
    lpf->preout = lpf->out;
}


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/**
 * @brief 初始化陷波滤波器
 * @param f 陷波滤波器结构体指针
 * @param fc 陷波中心频率 (Hz) -> 即您云台机械臂的谐振频率
 * @param bw 陷波带宽 (Hz) -> 决定滤波器的“杀伤范围”，通常设为 fc 的 5%~10%
 * @param fs 采样频率 (Hz) -> 您的MIT控制循环频率
 */
void NotchFilter_Init(NotchFilter* f, float fc, float bw, float fs) {
    float wc = 2.0f * M_PI * fc;       // 陷波中心角频率 (rad/s)
    float wbw = 2.0f * M_PI * bw;      // 陷波带宽角频率 (rad/s)
    float T = 1.0f / fs;               // 采样周期
    
    // 双线性变换预计算系数
    float a = wc * wc;
    float b = wbw * wc;
    float c = 4.0f / (T * T);
    float d = 2.0f * wc / T;
    
    float denom = c + b + a; // 公共分母
    
    // 分子系数
    f->b0 = (c + a) / denom;
    f->b1 = 2.0f * (a - c) / denom;
    f->b2 = (c + a) / denom;
    
    // 分母系数
    f->a1 = 2.0f * (a - c) / denom;
    f->a2 = (c - b + a) / denom;
    
    // 清零历史状态，防止启动时产生阶跃突变
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

/**
 * @brief 陷波滤波核心处理函数
 * @param f 陷波滤波器结构体指针
 * @param input 当前周期的输入信号 (如编码器反馈位置或速度)
 * @return 滤波后的信号
 */
float NotchFilter_Update(NotchFilter* f, float input) {
    // 直接II型 (Direct Form II) 差分方程计算
    float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 
                                - f->a1 * f->y1 - f->a2 * f->y2;
    
    // 更新历史状态
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;
    
    return output-input;
}


float test_freq_list[] = {
    1.f,2.f,3.f,4.f,5.f,6.f,7.f,8.f,9.f,10.f,11.f,12.f,13.f,14.f,15.f
};

uint8_t test_freq_index = 0;

float test_time = 0.0f;

// MIT模式下，这个单位要看你发给电机的 tau_ff 单位
// 如果 tau_ff 是 Nm，这里就是 Nm
// 如果你自己叫 current_ff，这里就是你的电流指令单位
float test_amp = 0.1f;

float test_dt = 0.001f;                  // 控制周期，1000Hz就是0.001
float single_freq_hold_time = 3.0f;      // 每个频率保持3秒

uint8_t step_sweep_enable = 0;
uint8_t step_sweep_finish = 0;

// 方便 JScope 观察
float step_sweep_current_freq = 0.0f;
float step_sweep_output = 0.0f;

float StepSweep_Update(void)
{
    if (step_sweep_enable == 0)
    {
        step_sweep_output = 0.0f;
        return 0.0f;
    }

    if (step_sweep_finish)
    {
        step_sweep_output = 0.0f;
        return 0.0f;
    }

    float freq = test_freq_list[test_freq_index];

    step_sweep_current_freq = freq;

    float sin_current = test_amp * sinf(2.0f * PI * freq * test_time);

    step_sweep_output = sin_current;

    test_time += test_dt;

    if (test_time >= single_freq_hold_time)
    {
        test_time = 0.0f;
        test_freq_index++;

        if (test_freq_index >= sizeof(test_freq_list) / sizeof(test_freq_list[0]))
        {
            test_freq_index = 0;
            step_sweep_enable = 0;
            step_sweep_finish = 1;
            step_sweep_output = 0.0f;
            step_sweep_current_freq = 0.0f;

            return 0.0f;
        }
    }

    return sin_current;
}



void ZVD_Init(ZVD_Shaper *zvd, float wn, float zeta) {
    float K = expf(-zeta * PI / sqrtf(1.0f - zeta * zeta));
    float den = 1.0f + 2.0f * K + K * K;
    
    zvd->A1 = 1.0f / den;
    zvd->A2 = 2.0f * K / den;
    zvd->A3 = K * K / den;
    zvd->Td = PI / (wn * sqrtf(1.0f - zeta * zeta));
    
    // 清空历史缓冲区
    zvd->buffer[0] = 0.0f;
    zvd->buffer[1] = 0.0f;
    zvd->buffer[2] = 0.0f;
    zvd->index = 0;
}

// 运行 ZVD 卷积计算
float ZVD_Run(ZVD_Shaper *zvd, float input) {
    // 将最新输入存入环形缓冲区
    zvd->buffer[zvd->index] = input;
    
    // 计算卷积输出 (假设控制周期与 Td 匹配，或按比例映射)
    // 实际工程中，Td 通常对应多个控制周期，此处为简化演示的离散卷积模型
    float output = zvd->A1 * zvd->buffer[zvd->index] 
                 + zvd->A2 * zvd->buffer[(zvd->index + 2) % 3] 
                 + zvd->A3 * zvd->buffer[(zvd->index + 1) % 3];
                 
    // 更新环形索引
    zvd->index = (zvd->index + 1) % 3;
    return output;
}
void pack_2bit_data_simple(const uint8_t *src, uint8_t *dst)
{
    dst[0] = 0;
		dst[1] = 0;
		dst[2] = 0;
    for (u8 i = 0; i < 12; i++) {
        u8 byte_idx = i / 4;          // 每4个数据占1个字节
        u8 bit_offset = (i % 4) * 2;  // 在字节内的偏移: 0, 2, 4, 6
        dst[byte_idx] |= (src[i] & 0x03) << bit_offset;
    }
}

void controlPushPull(u8 mode){
	switch(mode){
		case 0:
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
			break;
		default:
			break;
	}
}
