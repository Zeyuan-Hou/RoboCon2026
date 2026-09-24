#include "gyro.h"

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

/*------------------------------------------------Variables define------------------------------------------------*/
GYRO_DATA gyro_data, gyro_data_filtered;

uint8_t g_uart_rx_buf[512];
uint16_t g_uart_rx_cnt = 0;

uint8_t g_decode_data[512];
uint16_t g_decode_data_pos = 0;

/*------------------------------------------------Functions declare------------------------------------------------*/
int get_signed_int(unsigned char *data);
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);
extern void clear_data(int clr_len);

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
            gyro_data.accel_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            gyro_data.accel_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            gyro_data.accel_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
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
            ret = (unsigned char)0x1;
            gyro_data.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
            gyro_data.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            gyro_data.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
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

void data_filter(){
    const float smooth = 80;                    // 平滑等级，一阶低通滤波器截止频率
    static ST_LPF FJ_ax = {0, 0, 0, smooth, 0.001f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJ_ay = {0, 0, 0, smooth, 0.001f};
    static ST_LPF FJ_az = {0, 0, 0, smooth, 0.001f};

    FJ_ax.in = gyro_data.accel_x;
    FJ_ay.in = gyro_data.accel_y;
    FJ_az.in = gyro_data.accel_z;

    LpFilter(&FJ_ax);
    LpFilter(&FJ_ay);
    LpFilter(&FJ_az);

    gyro_data_filtered.accel_x = FJ_ax.out;
    gyro_data_filtered.accel_y = FJ_ay.out;
    gyro_data_filtered.accel_z = FJ_az.out;
}
