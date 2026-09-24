#ifndef __GYRO_H
#define __GYRO_H

#include <stdint.h>

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53

#define PROTOCOL_HEADER_BYTE		(short)2

#define PROTOCOL_FIRST_BYTE_POS 		0
#define PROTOCOL_SECOND_BYTE_POS		1

#define PROTOCOL_TID_LEN				2
#define PROTOCOL_MIN_LEN				7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(payload_len)		((payload_len) + 3)	/*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						5

#define SINGLE_DATA_BYTES				4

/*data id define*/
#define EULER_ID				(unsigned char)0x40


/*length for specific data id*/
#define IMU_TEMP_DATA_LEN				(unsigned char)2
#define ACCEL_DATA_LEN					(unsigned char)12
#define ANGLE_DATA_LEN					(unsigned char)12
#define MAGNETIC_DATA_LEN				(unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			(unsigned char)12
#define EULER_DATA_LEN					(unsigned char)12
#define QUATERNION_DATA_LEN				(unsigned char)16
#define UTC_DATA_LEN					(unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		(unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	(unsigned char)4
#define LOCATION_DATA_LEN				(unsigned char)12
#define SPEED_DATA_LEN          		(unsigned char)12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f

#define IMU_TEMP_DATA_FACTOR		0.01f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR		0.0000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

typedef enum
{
	crc_err = -3,
	data_len_err = -2,
	para_err = -1,
	analysis_ok = 0,
	analysis_done = 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

typedef struct
{
    unsigned int itow;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char miniute;
    unsigned char second;
}utc_data_t;


typedef struct
{
	float pitch;			/*unit: ?? (deg)*/
	float roll;
	float yaw;
}GYRO_DATA;

#pragma pack()

/*------------------------------------------------------------------------------------------------------------*/

extern uint8_t g_uart_rx_buf[512];	/*rx DMA buffer of uart2*///????????
extern uint16_t g_uart_rx_cnt; /*reciede data length of uart2*///????????????

extern uint8_t g_decode_data[512];	/*buffer for decoding*///????????
extern uint16_t g_decode_data_pos;	/*bytes left in decode buffer*/
/*------------------------------------------------Functions declare--------------------------------------------*/
int analysis_data(unsigned char *data, short len);

void clear_data(int clr_len);
void gyro_deal(void);

extern GYRO_DATA gyro_data;

#endif
