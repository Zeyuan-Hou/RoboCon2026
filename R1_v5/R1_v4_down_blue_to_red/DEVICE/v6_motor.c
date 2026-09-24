#include "v6_motor.h"

void V6_ReceiveData(V6_MOTOR *motor, uint8_t *data)
{
	float angle;
	float speed;
    memcpy(&angle, data, 4);
    memcpy(&speed, data + 4, 4);
	motor->angle = angle/motor->gearratio;
	motor->speed = speed/motor->gearratio;
//	LpFilter(&motor->lpf);
//	motor->speed = motor->lpf.out;
}

//id一般是0x200, 2006大疆电机和航模电机可以共用CAN发送包
HAL_StatusTypeDef V6_SendData(FDCAN_HandleTypeDef *hcan, int16_t id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t data[8] = {0};
	data[0]=(uint16_t)cur1>>8;
	data[1]=(uint16_t)cur1;
	data[2]=(uint16_t)cur2>>8;
	data[3]=(uint16_t)cur2;
	data[4]=(uint16_t)cur3>>8;
	data[5]=(uint16_t)cur3;
	data[6]=(uint16_t)cur4>>8;
	data[7]=(uint16_t)cur4; 
    return CAN_SendStdData(hcan, id ,data, 8);
}







