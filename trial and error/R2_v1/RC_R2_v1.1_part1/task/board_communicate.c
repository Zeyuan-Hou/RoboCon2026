#include "board_communicate.h"
// �Ƿ���������־λ
uint8_t flag_up_step;	// ��̨��
uint8_t flag_down_step; // ��̨��
uint8_t flag_up_R1;		// ��R1
// ����һ����ͨѶ
uint8_t flag_D_init;		   // D��ʼ��
uint8_t flag_D_ready_get_head; // D׼��ȡ����ͷ
uint8_t flag_D_get_left;	   // D���ȡ����ͷ
uint8_t flag_D_get_right;	   // D�ұ�ȡ����ͷ
uint8_t flag_D_get_block;	   // D׼����S���ķ���
uint8_t flag_D_3_init;

uint8_t flag_S_init;			  // S��ʼ��
uint8_t flag_S_get_block_down;	  // Sȡ�²㷽��
uint8_t flag_S_get_block_up;	  // Sȡ�ϲ㷽��
uint8_t flag_S_get_block_top;	  // Sȡ���ϲ㷽��
uint8_t flag_S_throw_block_back;  // S����ӷ���
uint8_t flag_S_throw_block_front; // S��ǰ�ӷ���
uint8_t flag_S_give_D;			  // S��D����
uint8_t flag_S_get_D_block;		  // SȡD���ķ���
uint8_t flag_S_put_block_middle;  // S�������в㷽��
uint8_t flag_S_put_block_top;	  // S�������ϲ㷽��

uint8_t flag_S_hold_block;
bool flag_update_tx = true;
/*******************************************************************************************
�������ƣ�Upper_Lower_Communication
�������ܣ��������°�ͨѶ �°彫���ݷ������ϰ�
���룺
		��txData���и�ֵ
�����

��ע��Dָ˫�ۣ�Sָ����
		[0] ֡ͷ 0x88
		[1] D��ʼ��
		[2] D׼��ȡ����ͷ
		[3] D���ȡ����
		[4] D�ұ�ȡ����
		[5] D׼����S���ķ���

		[6] S��ʼ��
		[7] Sȡ�²㷽��
		[8] Sȡ�ϲ㷽��
		[9] Sȡ���ϲ㷽��
		[10]S����ӷ���
		[11]S��ǰ�ӷ���
		[12]S��D����
		[13]SȡD���ķ���
		[14]S�������в㷽��
		[15]S�������ϲ㷽��
		[16]֡β 0x66

*******************************************************************************************/
void Upper_Lower_Communication(void)
{
	down_tx[0] = 0x88; // ֡ͷ
	down_tx[17] = 0x66;
	updateTxData();								 // 1-15λ
	HAL_UART_Transmit_DMA(&huart1, down_tx, 18); // ����
}
/**************************�ϰ����ݽ���**************************************/
int CHM;
void Upper_Data_Recieve_Deal(void)
{
	if (down_rx[0] != 0x33 && down_rx[17] != 0x55) // ֡ͷ֡β����
	{
		CHM++;
		return;
	}
	//		system_monitor_up[0] = rxData[2];
	//		system_monitor_up[1] = rxData[3];
	//		//2 3λ�ϰ�Ϊ����error��־λ
	// �ϰ��Ƿ��������

	flag_D_init = down_rx[1];			// D��ʼ��
	flag_D_ready_get_head = down_rx[2]; // D׼��ȡ����ͷ
	flag_D_get_left = down_rx[3];		// Dȡ������ͷ
	flag_D_get_right = down_rx[4];		// Dȡ������ͷ
	flag_D_get_block = down_rx[5];		// D׼����S���ķ���
	// flag_D_give_s_block= down_rx[6];//D��S����
  flag_D_3_init=down_rx[16];
	
	flag_S_init = down_rx[6];				// S��ʼ��
	flag_S_get_block_down = down_rx[7];		// Sȡ�²㷽��
	flag_S_get_block_up = down_rx[8];		// Sȡ�ϲ㷽��
	flag_S_get_block_top = down_rx[9];		// Sȡ���ϲ㷽��
	flag_S_throw_block_back = down_rx[10];	// S����ӷ���
	flag_S_throw_block_front = down_rx[11]; // S��ǰ�ӷ���
	flag_S_give_D = down_rx[12];			// S��D����
	flag_S_get_D_block = down_rx[13];		// SȡD���ķ���
	flag_S_put_block_middle = down_rx[14];	// S�������в㷽��
	flag_S_put_block_top = down_rx[15];		// S�������ϲ㷽��

	// flag_S_put_block = down_rx[10];
	// my_system_monitor.rate_cnt.UPPER_TO_LOWER++;
}
/****************************���·�������***************************/
void updateTxData(void)
{
	down_tx[1] = 0;
	down_tx[2] = 0;
	down_tx[3] = 0;
	down_tx[4] = 0;
	down_tx[5] = 0;

	down_tx[6] = 0;
	down_tx[7] = 0;
	down_tx[8] = 0;
	down_tx[9] = 0;
	down_tx[10] = 0;
	down_tx[11] = 0;
	down_tx[12] = 0;
	down_tx[13] = 0;
	down_tx[14] = 0;
	down_tx[15] = 0;
	down_tx[16] = 0;

	// ���ݵ�ǰ״̬�޸Ķ�Ӧ��down_txλΪ1
	switch (up_d_state)
	{
	case D_INIT: // 1
		down_tx[1] = 1;
		break;
	case D_READY_GET_HEAD: // 2
		down_tx[2] = 1;
		break;
	case D_GET_LEFT: // 3
		down_tx[3] = 1;
		break;
	case D_GET_RIGHT: // 4
		down_tx[4] = 1;
		break;
	case D_GET_BLOCK: // 5
		down_tx[5] = 1;
		break;
	case D_3_INIT:    //6
		down_tx[16] = 1;
	break;

	default:
		down_tx[1] = 0;
		down_tx[2] = 0;
		down_tx[3] = 0;
		down_tx[4] = 0;
		down_tx[5] = 0;
		down_tx[16] = 0;
		break;
	}
	switch (up_s_state)
	{
	case S_INIT: // 0
		down_tx[6] = 1;
		break;
	case S_GET_BLOCK_DOWN: // 1
		down_tx[7] = 1;
		break;
	case S_GET_BLOCK_UP: // 2
		down_tx[8] = 1;
		break;
	case S_GET_BLOCK_TOP: // 3
		down_tx[9] = 1;
		break;
	case S_THROW_BLOCK_BACK: // 4
		down_tx[10] = 1;
		break;
	case S_THROW_BLOCK_FRONT: // 5
		down_tx[11] = 1;
		break;
	case S_GIVE_D: // 6
		down_tx[12] = 1;
		break;
	case S_GET_D_BLOCK: // 7
		down_tx[13] = 1;
		break;
	case S_PUT_BLOCK_MIDDLE: // 8
		down_tx[14] = 1;
		break;
	case S_PUT_BLOCK_TOP: // 9
		down_tx[15] = 1;
		break;

	default:
		down_tx[7] = 0;
		down_tx[8] = 0;
		down_tx[9] = 0;
		down_tx[10] = 0;
		down_tx[11] = 0;
		down_tx[12] = 0;
		down_tx[13] = 0;
		down_tx[14] = 0;
		down_tx[15] = 0;
		
	
		break;
	}
}
/************************CRC-16-IBM����ʽ��x^16 + x^15 + x^2 + 1 (0xA001)**********************/
uint16_t crc16(uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF; // ��ʼֵ

	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
			{
				crc = (crc >> 1) ^ 0xA001; // ����ʽ��תֵ
			}
			else
			{
				crc = crc >> 1;
			}
		}
	}
	return crc;
}
