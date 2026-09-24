#ifndef __SYSTEM_MONITOR_H__
#define __SYSTEM_MONITOR_H__

#include "main.h"

#define NUM_PERIPHERAL 6

typedef enum {
    RS485_1 = 0,
    RS485_2,
    RS485_3,
    RS485_4,
    UDP_ENTER,
    UDP_REAL
} peripheral_list_e;

typedef enum {
    SYS_IDLE,
    SYS_NORMAL,
    SYS_ERROR,
} system_state_e;

typedef struct {
    uint32_t temp_rate[NUM_PERIPHERAL], real_rate[NUM_PERIPHERAL];
    system_state_e system_state;
		uint32_t uart_error_count[NUM_PERIPHERAL];
} system_monitor_t;

extern void error_detection(system_monitor_t *system_monitor);
extern system_monitor_t system_monitor;

#endif /* __SYSTEM_MONITOR_H__ */
