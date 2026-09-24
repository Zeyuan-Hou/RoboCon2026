#include "gyro.h"

#include <stddef.h>
#include <string.h>
#include <stdint.h>
// 陀螺仪YS320

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE (unsigned char)0x59
#define PROTOCOL_SECOND_BYTE (unsigned char)0x53

#define PROTOCOL_HEADER_BYTE (short)2

#define PROTOCOL_FIRST_BYTE_POS 0
#define PROTOCOL_SECOND_BYTE_POS 1

#define PROTOCOL_TID_LEN 2
#define PROTOCOL_MIN_LEN 7 /*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS 2
#define CRC_CALC_LEN(payload_len) ((payload_len) + 3) /*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len) (CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS 5

#define SINGLE_DATA_BYTES 4

/*data id define*/
#define IMU_TEMP_ID (unsigned char)0x01
#define ACCEL_ID (unsigned char)0x10
#define ANGLE_ID (unsigned char)0x20
#define MAGNETIC_ID (unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID (unsigned char)0x31 /*原始值*/
#define EULER_ID (unsigned char)0x40
#define QUATERNION_ID (unsigned char)0x41
#define UTC_ID (unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID (unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID (unsigned char)0x52
#define LOCATION_ID (unsigned char)0x60
#define SPEED_ID (unsigned char)0x70

/*length for specific data id*/
#define IMU_TEMP_DATA_LEN (unsigned char)2
#define ACCEL_DATA_LEN (unsigned char)12
#define ANGLE_DATA_LEN (unsigned char)12
#define MAGNETIC_DATA_LEN (unsigned char)12
#define MAGNETIC_RAW_DATA_LEN (unsigned char)12
#define EULER_DATA_LEN (unsigned char)12
#define QUATERNION_DATA_LEN (unsigned char)16
#define UTC_DATA_LEN (unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN (unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN (unsigned char)4
#define LOCATION_DATA_LEN (unsigned char)12
#define SPEED_DATA_LEN (unsigned char)12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f

#define IMU_TEMP_DATA_FACTOR 0.01f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR 0.0000001
#define ALT_DATA_FACTOR 0.001f
#define SPEED_DATA_FACTOR 0.001f

typedef enum
{
    crc_err = -3,
    data_len_err = -2,
    para_err = -1,
    analysis_ok = 0,
    analysis_done = 1
} analysis_res_t;

#pragma pack(1)

typedef struct
{
    unsigned char header1; /*0x59*/
    unsigned char header2; /*0x53*/
    unsigned short tid;    /*1 -- 60000*/
    unsigned char len;     /*length of payload, 0 -- 255*/
} output_data_header_t;

typedef struct
{
    unsigned char data_id;
    unsigned char data_len;
} payload_data_t;

typedef struct
{
    unsigned int itow;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char miniute;
    unsigned char second;
} utc_data_t;

typedef struct
{
    float accel_x; /*unit: m/s2*/
    float accel_y;
    float accel_z;

    float angle_x; /*unit: ° (deg)/s*/
    float angle_y;
    float angle_z;

    float mag_x; /*unit: 归一化值*/
    float mag_y;
    float mag_z;

    float raw_mag_x; /*unit: mGauss*/
    float raw_mag_y;
    float raw_mag_z;

    float pitch; /*unit: ° (deg)*/
    float roll;
    float yaw;

    float quaternion_data0;
    float quaternion_data1;
    float quaternion_data2;
    float quaternion_data3;

    double latitude;  /*unit: deg*/
    double longtidue; /*unit: deg*/
    float altidue;    /*unit: m*/

    float vel_n; /*unit: m/s */
    float vel_e;
    float vel_d;

    utc_data_t utc_data; /*utc data*/

    unsigned int sample_timestamp;     /*unit: us*/
    unsigned int data_ready_timestamp; /*unit: us*/

    float imu_temp;
} protocol_info_t;
#pragma pack()
/*------------------------------------------------Variables define------------------------------------------------*/
protocol_info_t g_output_info;

/*------------------------------------------------Functions declare------------------------------------------------*/
int get_signed_int(unsigned char *data);
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);

/*-------------------------------------------------------------------------------------------------------------*/
unsigned char check_data_len_by_id(unsigned char id, unsigned char len, unsigned char *data)
{
    unsigned char ret = 0xff;

    switch (id)
    {
    case ACCEL_ID:
    {
        if (ACCEL_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.accel_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            g_output_info.accel_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            g_output_info.accel_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			sys_mnt.cnt.gyro_acc++;
            g_update_flag |= ACC_UPDATE;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case ANGLE_ID:
    {
        if (ANGLE_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.angle_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            g_output_info.angle_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            g_output_info.angle_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
						sys_mnt.cnt.gyro_W++;
                        g_update_flag |= OMEGA_UPDATE;
                        g_omega = g_output_info.angle_z;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case MAGNETIC_ID:
    {
        if (MAGNETIC_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.mag_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            g_output_info.mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            g_output_info.mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case RAW_MAGNETIC_ID:
    {
        if (MAGNETIC_RAW_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.raw_mag_x = get_signed_int(data) * MAG_RAW_DATA_FACTOR;
            g_output_info.raw_mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
            g_output_info.raw_mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case EULER_ID:
    {
        if (EULER_DATA_LEN == len)
        {
            g_pre_yaw = g_output_info.yaw;
            ret = (unsigned char)0x1;
            g_output_info.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            g_output_info.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            g_output_info.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
					sys_mnt.cnt.gyro_yaw++;
                    g_update_flag |= YAW_UPDATE;
                    g_yaw = g_output_info.yaw;
            if(g_yaw == g_pre_yaw){
                g_stuck_cnt++;
            }
            else if(g_stuck_cnt <= 10)
                g_stuck_cnt = 0;
            
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case QUATERNION_ID:
    {
        if (QUATERNION_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            g_output_info.quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            g_output_info.quaternion_data2 = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            g_output_info.quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case LOCATION_ID:
    {
        if (LOCATION_DATA_LEN == len)
        {
            int temp = 0;

            ret = (unsigned char)0x1;
            temp = *((int *)data); /*solve the 'UNALIGNED' fault of Usage Faults*/
            g_output_info.latitude = temp * LONG_LAT_DATA_FACTOR;

            temp = *((int *)data + 1);
            g_output_info.longtidue = temp * LONG_LAT_DATA_FACTOR;

            temp = *((int *)data + 2);
            g_output_info.altidue = temp * ALT_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case SPEED_ID:
    {
        if (SPEED_DATA_LEN == len)
        {
            int temp = 0;

            ret = (unsigned char)0x1;

            temp = *((int *)data); /*solve the 'UNALIGNED' fault of Usage Faults*/
            g_output_info.vel_n = temp * SPEED_DATA_FACTOR;

            temp = *((int *)data + 1);
            g_output_info.vel_e = temp * SPEED_DATA_FACTOR;

            temp = *((int *)data + 2);
            g_output_info.vel_d = temp * SPEED_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case IMU_TEMP_ID:
    {
        if (IMU_TEMP_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.imu_temp = *((short *)data) * IMU_TEMP_DATA_FACTOR;
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case UTC_ID:
    {
        if (UTC_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            memcpy((unsigned char *)&g_output_info.utc_data.itow, data, len);
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case SAMPLE_TIMESTAMP_ID:
    {
        if (SAMPLE_TIMESTAMP_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.sample_timestamp = *((unsigned int *)data);
        }
        else
        {
            ret = (unsigned char)0x00;
        }
    }
    break;

    case DATA_READY_TIMESTAMP_ID:
    {
        if (DATA_READY_TIMESTAMP_DATA_LEN == len)
        {
            ret = (unsigned char)0x1;
            g_output_info.data_ready_timestamp = *((unsigned int *)data);
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
 * 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
 * crc校验从TID开始到payload data的最后一个字节
 */
int analysis_data(unsigned char *data, short len)
{
    unsigned short payload_len = 0;
    unsigned short check_sum = 0;
    unsigned short pos = 0;
    unsigned char ret = 0xff;
    unsigned char *temp = NULL;
    short i = len;

    output_data_header_t *header = NULL;
    payload_data_t *payload = NULL;

    if (NULL == data || 0 >= len)
    {
        return para_err;
    }

    if (len < PROTOCOL_MIN_LEN)
    {
        return data_len_err;
    }

    temp = data;

    while (i >= PROTOCOL_HEADER_BYTE)
    {
        /*judge protocol header*/
        if (PROTOCOL_FIRST_BYTE == *temp && PROTOCOL_SECOND_BYTE == temp[1])
        {
            break;
        }
        else
        {
            temp++;
            i--;
        }
    }

    if (i < PROTOCOL_MIN_LEN)
    {
        clear_data(len - i);
        return data_len_err; /*pack len err*/
    }

    /*further check*/
    header = (output_data_header_t *)temp;
    payload_len = header->len;

    if (payload_len + PROTOCOL_MIN_LEN > i)
    {
        return data_len_err;
    }

    /*checksum*/
    calc_checksum(temp + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
    if (check_sum != *((unsigned short *)(temp + PROTOCOL_CRC_DATA_POS(payload_len))))
    {
        clear_data(len - i + header->len + PROTOCOL_MIN_LEN);
        return crc_err;
    }

    /*analysis payload data*/
    pos = PAYLOAD_POS;

    while (payload_len > 0)
    {
        payload = (payload_data_t *)(temp + pos);
        ret = check_data_len_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);
        if ((unsigned char)0x01 == ret)
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

    if (NULL == data || 0 == len || NULL == checksum)
    {
        return -1;
    }

    for (i = 0; i < len; i++)
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
    if (g_decode_data_pos > (unsigned int)clr_len)
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

//陀螺仪雷达融合获取角度（大多数时候相信陀螺仪，死掉会用雷达）

float gyro_or_radar = 0;

float GET_YAW(float gyro_yaw,float radar_yaw,float gyro_w)
{ 
	static uint8_t start_delay_200 = 0; //上电等200ms再开始
	static uint8_t inited = 0; //初始化一次
	static uint8_t stuck_cnt = 0;  //陀螺仪损坏计数
	static float prev_gyro = 0; //上一次陀螺仪角度
	static float pos_yaw = 0;  //真正用到导航里的YAW角
	static uint8_t gyro_dead_flag = 0; //陀螺仪死掉了
	static float gyro_offset = 0;  //陀螺仪纠偏量
	static float gyro_real = 0;  //陀螺仪加上纠偏之后的数据

  static uint16_t check_cnt = 0;        //  检测间隔计数
  static float last_diff = 0;           //  上次记录的差值
  static uint8_t need_fix = 0;          //  需要修正
	
	
    if(!inited) {
        start_delay_200++;
        if(radar_yaw != 0) {
            pos_yaw = radar_yaw;  // 延时期间先用雷达
        }
        if(start_delay_200 >= 40) {  // 200ms到了
            if(radar_yaw != 0) {
                gyro_offset = radar_yaw - gyro_yaw;
            }
            gyro_real = gyro_yaw + gyro_offset;
            prev_gyro = gyro_real;
            pos_yaw = gyro_real;
            inited = 1;
						start_delay_200 = 0;
        }
        return pos_yaw;
    }		
	
	
	gyro_real = gyro_yaw + gyro_offset;

   // 每隔一段时间检测差值
    check_cnt++;
    if(check_cnt >= 200 && radar_yaw != 0) {
        check_cnt = 0;
        last_diff = fabs(radar_yaw - gyro_real);
        if(last_diff > 0.03f) {  // 差值大于0.03rad需要修正
            need_fix = 1;
        }
    }

		
    if(need_fix && fabs(gyro_w) < 0.3f && radar_yaw != 0) {
        gyro_offset = radar_yaw - gyro_yaw;
        gyro_real = gyro_yaw + gyro_offset;
        need_fix = 0;
    }
		
		
		if(!gyro_dead_flag&&inited) //陀螺仪正常工作
	{
		
			// === 陀螺仪处理 ===
			if(gyro_real == prev_gyro) 
			{
        stuck_cnt++;
				if(stuck_cnt > 10) {gyro_dead_flag = 1;}// 判死刑
			} else {
								stuck_cnt = 0;
								pos_yaw = gyro_real;
								gyro_or_radar = 0;
							}		
		
	 prev_gyro = gyro_real;
	}else if(gyro_dead_flag&&inited)
		{
			pos_yaw = radar_yaw;
			gyro_or_radar = 1;
		  need_fix = 0;    
		}
	
	 return pos_yaw;
}



