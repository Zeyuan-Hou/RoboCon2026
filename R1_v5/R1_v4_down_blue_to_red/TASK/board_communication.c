#include "board_communication.h"
uint8_t fps_check(SYSTEM_MONITORS *sm)
{
	uint8_t ret = 1;
	ret &= (sm->fps.action_task > 900);
	ret &= (sm->fps.can1_v6_tx > 900);
	ret &= (sm->fps.can2_v6_tx > 900);
	ret &= (sm->fps.chassis_task > 900);
	ret &= (sm->fps.dt35_rx > 650);
	ret &= (sm->fps.leftdown_driver_rx > 900);
	ret &= (sm->fps.leftup_driver_rx > 900);
	ret &= (sm->fps.rightdown_driver_rx > 900);
	ret &= (sm->fps.rightup_driver_rx > 900);
	ret &= (sm->fps.location_task > 900);
	ret &= (sm->fps.navigation_task > 900);
	ret &= (sm->fps.radar_rx > 20);
	ret &= (sm->fps.rightdown_steer_rx > 900);
	ret &= (sm->fps.rightup_steer_rx > 900);
	ret &= (sm->fps.leftup_steer_rx > 900);
	ret &= (sm->fps.leftdown_steer_rx > 900);
	ret &= (sm->fps.send_task > 900);
	ret &= (sm->fps.vision_rx > 90);
	ret &= (sm->fps.with_upper_rx > 900);
	ret &= (sm->fps.with_upper_tx > 900);
	ret &= (sm->fps.travelswitch_rx > 650);
	return ret;
}

uint8_t motor_fps_check(SYSTEM_MONITORS *sm){
	uint8_t ret = 0;
	ret |= (sm->fps.leftdown_driver_rx > 900);
	ret |= (sm->fps.leftdown_steer_rx > 900) << 1;
	ret |= (sm->fps.leftup_driver_rx > 900) << 2;
	ret |= (sm->fps.leftup_steer_rx > 900) << 3;
	ret |= (sm->fps.rightup_driver_rx > 900) << 4;
	ret |= (sm->fps.rightup_steer_rx > 900) << 5;
	ret |= (sm->fps.rightdown_driver_rx > 900) << 6;
	ret |= (sm->fps.rightdown_steer_rx > 900) << 7;
	return ret;
}

void Send_to_Upper(void)
{
	//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	data_to_upper.header = 0x0b;
	data_to_upper.tail = 0x0a;
	data_to_upper.vis_x = (int16_t)location_to_upper.fpPosX;
	data_to_upper.vis_y = (int16_t)location_to_upper.fpPosY;
	data_to_upper.vis_yaw = (int16_t)(norm_angle(location_to_upper.fpPosQ) * 10.f);
	data_to_upper.progress = Nav.progress;
	data_to_upper.t_remain = Path_Points.t_remain;
	data_to_upper.fps_err = motor_fps_check(&sys_mnt);//!fps_check(&sys_mnt);
	if ((Nav.state == NAV_PATH || Nav.state == NAV_LOCK) && hypotf(location.fpPosX - Nav.final_pos.fpPosX, location.fpPosY - Nav.final_pos.fpPosY) < 600.f)
	{
		data_to_upper.mac_flag = 1;
	}
	else{
		data_to_upper.mac_flag = 0;
	}
	//    osDelay(2);
	if (HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&data_to_upper, sizeof(TO_UPPER)) == HAL_OK)
		sys_mnt.cnt.with_upper_tx++;
}

const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
/**
 * @brief 查表法计算 CRC-16 Modbus 校验值
 * @param data 指向待校验数据缓冲区的指针
 * @param length 数据长度
 * @return 16位 CRC 校验结果
 */

uint16_t crc16_check(uint8_t *data, uint16_t size){
    uint16_t crc = 0xFFFF; // 初始值为 0xFFFF
    
    for (size_t i = 0; i < size; i++) {
        uint8_t index = (uint8_t)(crc ^ data[i]);
        crc = (crc >> 8) ^ crc16_table[index];
    }
    return crc;
}

float V_k = 1.f;
float W_k = 1.f;

ST_VEL final_vel = {0, 500, 0};

void Receive_from_Upper(uint8_t *data)
{
	size_t size = sizeof(FROM_UPPER);
	for (uint8_t i = 0; i < size; i++)
	{
		if (data[i] == 0x0a && data[(i + size - 1) % size] == 0x0b)
		{
			sys_mnt.cnt.with_upper_rx++;
			memcpy(&data_from_upper, &data[i], size - i);
			memcpy((uint8_t *)(&data_from_upper) + size - i, data, i);
			break;

		}
	}
}

void Process_from_upper(void){
			if (Chassis.err == NO_ERROR || data_from_upper.chassis_action == 1)
			{
				if(data_from_upper.remote_Vx != 0) Remotevel.fpVx = data_from_upper.remote_Vx * V_k;
				if(data_from_upper.remote_Vy != 0) Remotevel.fpVy = -data_from_upper.remote_Vy * V_k;
				if(data_from_upper.remote_W != 0) Remotevel.fpW = -data_from_upper.remote_W * W_k;
				switch (data_from_upper.chassis_action)
				{
				case 1:
					Chassis_Change(CHASSIS_INIT);
					break;
				case 2:
					V_k = 0.3f;
					W_k = 0.3f;
					Chassis_Change(CHASSIS_LOCAL_REMOTE);
					break;
				case 3:
					Nav_Start((uint16_t)data_from_upper.nav_target);
					break;
				case 4:
					Chassis_Change(CHASSIS_LOCK);
					break;
				case 5:
					Chassis_Change(CHASSIS_FIXED);
					break;
				case 6:
					V_k = 2;
					W_k = 2;
					Chassis_Change(CHASSIS_LOCAL_REMOTE);
					break;
				case 7:
					V_k = 1.5f;
					W_k = 0.5f;
					Chassis_Change(CHASSIS_GLOBAL_REMOTE);
					break;
				case 12:
					noLock_mode = 0;
					break;
				case 13:
					Chassis_Change(CHASSIS_FIXED);
          Chassis.fixed_cnt = 1000;
          Chassis.fixed_dir = 90;
          Chassis.fixed_vel = 200;
					break;
				case 14:
					Nav_Start_withoutLock((uint16_t)data_from_upper.nav_target);
					break;
				default:
					break;
				}
				
			}
			else
			{
				memset(&Remotevel, 0, sizeof(ST_VEL));
			}
}

#pragma pack(push, 1)

typedef struct
{
	float data[31];
	uint8_t tail[4];
} TO_VOFA;

#pragma pack(pop)

TO_VOFA data_to_vofa = {
	.tail = {0x00, 0x00, 0x80, 0x7f}};

void Send_to_Vofa(void)
{
	data_to_vofa.data[0] = 1;
	data_to_vofa.data[1] = Nav.object.pos.fpPosX;
	data_to_vofa.data[2] = Nav.object.pos.fpPosY;
	data_to_vofa.data[3] = Nav.object.pos.fpPosQ;
	data_to_vofa.data[4] = location.fpPosY;
	data_to_vofa.data[5] = Vision.pos.fpPosY;
	data_to_vofa.data[6] = pid_leftdown_driver.fpFB;	
	data_to_vofa.data[7] = pid_leftup_driver.fpFB;;
	data_to_vofa.data[8] = location.fpPosQ;
	data_to_vofa.data[9] = location.fpPosX;
	data_to_vofa.data[10] = Vision.pos.fpPosX;
	data_to_vofa.data[11] =	pid_yaw.fpDes;
	data_to_vofa.data[12] = pid_yaw.fpFB;

	HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&data_to_vofa, sizeof(TO_VOFA));
}
