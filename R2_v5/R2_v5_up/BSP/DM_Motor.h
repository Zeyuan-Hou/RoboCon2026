#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "CAN_Bsp.h"
#include "Global_Variables.h"

#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPD_MODE 0x200
#define PSI_MODE 0x300

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(hcan_t *hcan, Motor_DM *motor);
void dm_motor_enable(hcan_t *hcan, Motor_DM *motor);
void dm_motor_disable(hcan_t *hcan, Motor_DM *motor);
void dm_motor_clear_para(Motor_DM *motor);
void dm_motor_clear_err(hcan_t *hcan, Motor_DM *motor);
void dm_motor_fbdata(Motor_DM *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t *hcan, Motor_DM *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor);
void pos_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t *hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float cur);

void save_pos_zero(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */
