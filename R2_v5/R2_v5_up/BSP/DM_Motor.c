#include "DM_Motor.h"

void dm_motor_enable(hcan_t *hcan, Motor_DM *motor)
{
	switch (motor->ctrl.mode)
	{
	case mit_mode:
		enable_motor_mode(hcan, motor->id, MIT_MODE);
		break;
	case pos_mode:
		enable_motor_mode(hcan, motor->id, POS_MODE);
		break;
	case spd_mode:
		enable_motor_mode(hcan, motor->id, SPD_MODE);
		break;
	case psi_mode:
		enable_motor_mode(hcan, motor->id, PSI_MODE);
		break;
	}
}

void dm_motor_disable(hcan_t *hcan, Motor_DM *motor)
{
	switch (motor->ctrl.mode)
	{
	case mit_mode:
		disable_motor_mode(hcan, motor->id, MIT_MODE);
		break;
	case pos_mode:
		disable_motor_mode(hcan, motor->id, POS_MODE);
		break;
	case spd_mode:
		disable_motor_mode(hcan, motor->id, SPD_MODE);
		break;
	case psi_mode:
		disable_motor_mode(hcan, motor->id, PSI_MODE);
		break;
	}
	dm_motor_clear_para(motor);
}

void dm_motor_ctrl_send(hcan_t *hcan, Motor_DM *motor)
{
	switch (motor->ctrl.mode)
	{
	case mit_mode:
		mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
		break;
	case pos_mode:
		pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
		break;
	case spd_mode:
		spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
		break;
	case psi_mode:
		psi_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
		break;
	}
}


void dm_motor_clear_para(Motor_DM *motor)
{
	motor->ctrl.kd_set = 0;
	motor->ctrl.kp_set = 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

void dm_motor_clear_err(hcan_t *hcan, Motor_DM *motor)
{
	switch (motor->ctrl.mode)
	{
	case mit_mode:
		clear_err(hcan, motor->id, MIT_MODE);
		break;
	case pos_mode:
		clear_err(hcan, motor->id, POS_MODE);
		break;
	case spd_mode:
		clear_err(hcan, motor->id, SPD_MODE);
		break;
	case psi_mode:
		clear_err(hcan, motor->id, PSI_MODE);
		break;
	}
}

void dm_motor_fbdata(Motor_DM *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0]) & 0x0F;
	motor->para.state = (rx_data[0]) >> 4;
	motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
	motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -12.5f, 12.5f, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, -45.0f, 45.0f, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -12.0f, 12.0f, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

	CANx_SendstdData(hcan, id, data, 8);
}

void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

	CANx_SendstdData(hcan, id, data, 8);
}

void save_pos_zero(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;

	CANx_SendstdData(hcan, id, data, 8);
}

void clear_err(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;

	CANx_SendstdData(hcan, id, data, 8);
}

void mit_ctrl(hcan_t *hcan, Motor_DM *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -12.5f, 12.5f, 16);
	vel_tmp = float_to_uint(vel, -45.0f, 45.0f, 12);
	tor_tmp = float_to_uint(tor, -12.0f, 12.0f, 12);
	kp_tmp = float_to_uint(kp, 0.f, 500.f, 12);
	kd_tmp = float_to_uint(kd, 0.f, 5.f, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;

	CANx_SendstdData(hcan, id, data, 8);
}

void pos_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];

	id = motor_id + POS_MODE;
	pbuf = (uint8_t *)&pos;
	vbuf = (uint8_t *)&vel;

	data[0] = *pbuf;
	data[1] = *(pbuf + 1);
	data[2] = *(pbuf + 2);
	data[3] = *(pbuf + 3);

	data[4] = *vbuf;
	data[5] = *(vbuf + 1);
	data[6] = *(vbuf + 2);
	data[7] = *(vbuf + 3);

	CANx_SendstdData(hcan, id, data, 8);
}

void spd_ctrl(hcan_t *hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];

	id = motor_id + SPD_MODE;
	vbuf = (uint8_t *)&vel;

	data[0] = *vbuf;
	data[1] = *(vbuf + 1);
	data[2] = *(vbuf + 2);
	data[3] = *(vbuf + 3);

	CANx_SendstdData(hcan, id, data, 4);
}

void psi_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];

	uint16_t u16_vel = vel * 100;
	uint16_t u16_cur = cur * 10000;

	id = motor_id + PSI_MODE;
	pbuf = (uint8_t *)&pos;
	vbuf = (uint8_t *)&u16_vel;
	ibuf = (uint8_t *)&u16_cur;

	data[0] = *pbuf;
	data[1] = *(pbuf + 1);
	data[2] = *(pbuf + 2);
	data[3] = *(pbuf + 3);

	data[4] = *vbuf;
	data[5] = *(vbuf + 1);

	data[6] = *ibuf;
	data[7] = *(ibuf + 1);

	CANx_SendstdData(hcan, id, data, 8);
}

void read_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0xFF;		 // 低 8 位
	uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	CANx_SendstdData(&hfdcan1, 0x7FF, data, 4);
}

void read_motor_ctrl_fbdata(uint16_t id)
{
	uint8_t can_id_l = id & 0xFF;		 // 低 8 位
	uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	CANx_SendstdData(&hfdcan1, 0x7FF, data, 4);
}

void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;

	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	CANx_SendstdData(&hfdcan1, 0x7FF, data, 8);
}

void save_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0xFF;		 // 低 8 位
	uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	CANx_SendstdData(&hfdcan1, 0x7FF, data, 4);
}
