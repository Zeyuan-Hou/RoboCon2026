/*
 * @Description: 
 * @Version: 2.0
 * @Autor: HITCRT
 * @Date: 2024-03-07 16:48:19
 * @LastEditors: Porcovsky
 * @LastEditTime: 2024-04-05 16:57:46
 */
#include "LK_motor.h" 
#include "CANbsp.h"
#include "Robot.h"
#include "pid_algorithm.h"

void LK_PID_Set(void){
	PID_Init(&stretch_LK.lk_pid.outer,350.0f,1.f,0,0,100.0f,100.0f,5000.0f,5000.0f,0);
	PID_Init(&stretch_LK.lk_pid.inner,0.42f,0,0,0,1000,100,2000,2000,500);
}

uint8_t LK_Pos_Ctrl(Motor_LK_8016 *pstMotor,fp32 tPos){
//	LK1_TD.aim=tPos;
//	CalTD(&LK1_TD);
	PID_Calc(&pstMotor->lk_pid.outer,tPos/*LK1_TD.x1*/,pstMotor->angle);
	PID_Calc(&pstMotor->lk_pid.inner,pstMotor->lk_pid.outer.fpU,pstMotor->anglev);
	if(fabsf(pstMotor->angle-tPos)<0.5f){
		return 1;
	}else{
		return 0;
	}
}
uint8_t LK_Pos_CtrlWithoutPID(Motor_LK_8016 *pstMotor,fp32 tPos){
	pstMotor->outerTarget=tPos;
	if(fabsf(pstMotor->angle-tPos)<1.f){
		return 1;
	}else{
		return 0;
	}
}
/**
 * @brief: 翎控电机失能函数
 * @param {CAN_HandleTypeDef} *hcan
 * @param {uint32_t} id
 * @note: 
 * @author: HITCRT
 */
void LK_8016_Close(CAN_HandleTypeDef *hcan, uint32_t id)
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=0x80;
	TxCurrent[1]=0x00;
	TxCurrent[2]=0x00;
	TxCurrent[3]=0x00;
	TxCurrent[4]=0x00;
	TxCurrent[5]=0x00;
	TxCurrent[6]=0x00;
	TxCurrent[7]=0x00;
	CANx_SendstdData(hcan,id,TxCurrent,8);
}
/**
 * @brief: 翎控电机使能函数
 * @param {CAN_HandleTypeDef} *hcan
 * @param {u32} id
 * @note: 
 * @author: HITCRT
 */
void CAN_8016_Open(CAN_HandleTypeDef *hcan,u32 id)
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=0x88;
	TxCurrent[1]=0x00;
	TxCurrent[2]=0x00;
	TxCurrent[3]=0x00;
	TxCurrent[4]=0x00;
	TxCurrent[5]=0x00;
	TxCurrent[6]=0x00;
	TxCurrent[7]=0x00;
	CANx_SendstdData(hcan,id,TxCurrent,8);
}
/**
 * @brief: 翎控电机单个电机电流控制函数
 * @param {CAN_HandleTypeDef} *hcan
 * @param {u32} id
 * @param {s16} current
 * @note: 数值范围-2048~ 2048 对应 ，MG 电机实际转矩电流范围-33A~33A，
 * @author: HITCRT
 */
void CAN_8016_SendCurrent_Single(CAN_HandleTypeDef *hcan,u32 id,s16 current)
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=0xA1;
	TxCurrent[1]=0x00;
	TxCurrent[2]=0x00;
	TxCurrent[3]=0x00;
	TxCurrent[4]=*(uint8_t *)(&current);
	TxCurrent[5]= *((uint8_t *)(&current)+1);
	TxCurrent[6]=0x00;
	TxCurrent[7]=0x00;
	CANx_SendstdData(hcan,id,TxCurrent,8);
}
/**
 * @brief: 翎控电机多电机电流模式
 * @param {CAN_HandleTypeDef} *hcan
 * @param {u32} id
 * @param {s16} current1
 * @param {s16} current2
 * @param {s16} current3
 * @param {s16} current4
 * @note: 
 * @author: HITCRT
 */
void CAN_8016_SendCurrent(CAN_HandleTypeDef *hcan,u32 id,s16 current1,s16 current2,s16 current3,s16 current4)
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=current1;
	TxCurrent[1]=current1>>8;
	TxCurrent[2]=current1;
	TxCurrent[3]=current1>>8;
	TxCurrent[4]=current1;
	TxCurrent[5]=current1>>8;
	TxCurrent[6]=current1;
	TxCurrent[7]=current1>>8;
	CANx_SendstdData(hcan,id,TxCurrent,8);
}
/**
 * @brief: 
 * @param {uint8_t *} CAN_RX_BUF
 * @param {Motor8016_Msg} *MG8016_Msg
 * @note: 
 * @author: HITCRT
 */
void LK_8016_DataProcess(uint8_t * CAN_RX_BUF, Motor_LK_8016 *LK_Motor)
{
	LK_Motor->temp=CAN_RX_BUF[1];
	LK_Motor->current=CAN_RX_BUF[2] | (CAN_RX_BUF[3]<<8);
	LK_Motor->speed=CAN_RX_BUF[4] | (CAN_RX_BUF[5]<<8);
	LK_Motor->encoder_num=CAN_RX_BUF[6] | (CAN_RX_BUF[7]<<8);
}


void LK_8016_zero_set(CAN_HandleTypeDef *hcan,u32 id) //该命令会将零点写入驱动的ROM，多次写入将会影响芯片寿命，不建议频繁使用
{
	uint8_t TxCurrent[8];
	TxCurrent[0]=0x19;
	TxCurrent[1]=0x00;
	TxCurrent[2]=0x00;
	TxCurrent[3]=0x00;
	TxCurrent[4]=0x00;
	TxCurrent[5]=0x00;
	TxCurrent[6]=0x00;
	TxCurrent[7]=0x00;
	CANx_SendstdData(hcan,id,TxCurrent,8);
}

