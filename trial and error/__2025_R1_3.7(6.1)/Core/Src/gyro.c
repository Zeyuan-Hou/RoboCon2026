#include "gyro.h"
/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：gyro.c
最近修改日期：2025.2.10
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：

----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
	 XSH       2025.2.19        	1.0      
**********************************************************************************************************************************************************/

      
#include <stddef.h>
#include <string.h>
#include <stdint.h>


GYRO_DATA gyro_data;//?????????????


uint8_t g_uart_rx_buf[512];	//???????????
uint16_t g_uart_rx_cnt = 0; //????????????

uint8_t g_decode_data[512];	//????????
uint16_t g_decode_data_pos = 0;	
/*------------------------------------------------Functions declare------------------------------------------------*/
int get_signed_int(unsigned char *data);
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);
extern void clear_data(int clr_len);

/*-------------------------------------------------------------------------------------------------------------*/
void gyro_deal(void)
{
	 if(g_uart_rx_cnt > 0)
		{
			memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
			g_decode_data_pos += g_uart_rx_cnt;
			g_uart_rx_cnt = 0;
		}		
		
		if(g_decode_data_pos > 0)
		{
			analysis_data(g_decode_data, g_decode_data_pos);
		}
}


unsigned char check_data_len_by_id(unsigned char id, unsigned char len, unsigned char *data)
{
	unsigned char ret = 0xff;

	switch(id)
	{

		case EULER_ID://?????
		{
			if(EULER_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				gyro_data.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				gyro_data.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				gyro_data.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				gyro_data.yaw *=10;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		default:
		break;
	}

	return ret;
}

/*--------------------------------------------------------------------------------------------------------------
* ???Э?????header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* crcУ???TID?????payload data???????????
*/
int analysis_data(unsigned char *data, short len)
{
  short payload_len = 0;
	unsigned short check_sum = 0;
	unsigned short pos = 0;
	unsigned char ret = 0xff;
	unsigned char *temp = NULL;
	short i = len;
	
	output_data_header_t *header = NULL;
	payload_data_t *payload = NULL;

	if(NULL == data || 0 >= len)
	{
		return para_err;
	}

	if(len < PROTOCOL_MIN_LEN)
	{
		return data_len_err;
	}

	temp = data;
	
	while(i >= PROTOCOL_HEADER_BYTE)
	{
		/*judge protocol header*/		
		if(PROTOCOL_FIRST_BYTE == *temp && PROTOCOL_SECOND_BYTE == temp[1])
		{
			break;
		}
		else
		{
			temp++;
			i--;				
		}
	}	
	
	if(i < PROTOCOL_MIN_LEN)
	{
		clear_data(len - i);
		return data_len_err;	/*pack len err*/
	}	
	
	/*further check*/
	header = (output_data_header_t *)temp;
	payload_len = header->len;

	if(payload_len + PROTOCOL_MIN_LEN > i)
	{
		return 	data_len_err;
	}

	/*checksum*/
	calc_checksum(temp + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
	if(check_sum != *((unsigned short *)(temp + PROTOCOL_CRC_DATA_POS(payload_len))))
	{
		clear_data(len - i + header->len + PROTOCOL_MIN_LEN);
		return crc_err;
	}

	/*analysis payload data*/
	pos = PAYLOAD_POS;

	while(payload_len > 0)
	{
		payload = (payload_data_t *)(temp + pos);
		ret = check_data_len_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);
		if((unsigned char)0x01 == ret)
		{
			pos += payload->data_len + sizeof(payload_data_t);
			payload_len -= payload->data_len + sizeof(payload_data_t);
		}
		else
		{
			pos++;
			payload_len--;
		}
	}
	
	payload_len = 0;
	clear_data(len - i + payload_len + PROTOCOL_MIN_LEN);
	
	return analysis_ok;
}

int get_signed_int(unsigned char *data)
{
	int temp = 0;

	temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}

int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
	unsigned char check_a = 0;
	unsigned char check_b = 0;
	unsigned short i;

	if(NULL == data || 0 == len || NULL == checksum)
	{
		return -1;
	}

	for(i = 0; i < len; i++)
	{
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short)(check_b << 8) | check_a);

	return 0;
}

void clear_data(int clr_len)
{
	memset(g_decode_data, 0, clr_len);	
	if(g_decode_data_pos > (unsigned int)clr_len)
	{
		g_decode_data_pos -= clr_len;
		memcpy(g_decode_data, g_decode_data + clr_len, g_decode_data_pos);
		memset(g_decode_data + g_decode_data_pos, 0, clr_len);
	}
	else
	{
		g_decode_data_pos = 0;
	}	
}

