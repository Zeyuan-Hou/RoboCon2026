#ifndef __GYRO_H__
#define __GYRO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "global_declare.h"
#include "algorithm.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

    /*------------------------------------------------MARCOS define------------------------------------------------*/

    /*------------------------------------------------Type define--------------------------------------------------*/
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
        float pitch; /*unit: ?? (deg)*/
        float roll;
        float yaw;

        float accel_x;
        float accel_y;
        float accel_z;
    } GYRO_DATA;

#pragma pack()

    /*------------------------------------------------------------------------------------------------------------*/
    extern GYRO_DATA gyro_data;

    extern uint8_t g_uart_rx_buf[512];
    extern uint16_t g_uart_rx_cnt;
    extern uint8_t g_decode_data[512];
    extern uint16_t g_decode_data_pos;

    /*------------------------------------------------Functions declare--------------------------------------------*/
    int analysis_data(unsigned char *data, short len);
    void data_filter();

#ifdef __cplusplus
}
#endif

#endif
