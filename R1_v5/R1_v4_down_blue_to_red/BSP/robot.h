#ifndef ___ROBOT_H___
#define ___ROBOT_H___

#include "stm32h7xx_hal.h"
#include "string.h"
#include <stddef.h>
#include <stdint.h>
#include "math.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "math_algos.h"
#include "path_algos.h"

#define M2006_GEARRATIO 36
#define V6_GEARRATIO 5.294f

#define M2006 MAX_CURRENT 12000
#define V6_MAX_CURRENT 9000

#define DELAYED_KF_SIZE 150

#pragma pack(push, 1)

typedef struct
{
    uint16_t location_task;
    uint16_t send_task;
    uint16_t navigation_task;
    uint16_t chassis_task;
    uint16_t action_task;
    uint16_t rightup_driver_rx;
    uint16_t rightdown_driver_rx;
    uint16_t leftup_driver_rx;
    uint16_t leftdown_driver_rx;
    uint16_t rightup_steer_rx;
    uint16_t rightdown_steer_rx;
    uint16_t leftup_steer_rx;
    uint16_t leftdown_steer_rx;
    uint16_t dt35_rx;
    uint16_t dt35_left_valid;
    uint16_t dt35_right_valid;
    uint16_t dt35_front_valid;
    uint16_t travelswitch_rx;
    uint16_t vision_rx;
		uint16_t vision_tx;
    uint16_t can1_v6_tx;
    uint16_t can2_v6_tx;
    uint16_t with_upper_rx;
    uint16_t with_upper_tx;
    uint16_t radar_rx;
    uint16_t posKF_update;
    uint16_t gyro_yaw;
		uint16_t gyro_acc;
		uint16_t gyro_W;
		uint16_t trt_tx;
} SYSTEM_MONITOR; // 系统监控,其中变量必须为uint16_t类型

typedef struct
{
    SYSTEM_MONITOR cnt;
    SYSTEM_MONITOR fps;
} SYSTEM_MONITORS;

#pragma pack(pop)

/* motor begin */
typedef struct
{
    int siRawValue;    /**< 当前编码器原始值 */
    int siPreRawValue; /**< 上次编码器原始值 */
    int siDiff;        /**< 编码器差值 */
    int siSumValue;    /**< 编码器累计值 */
    float siGearRatio; /**< 齿轮比 */
    int siNumber;      /**< 编码器线数 */
    float fpSpeed;     /**< 编码器测得的速度，单位：转/分钟 */
    uint8_t state;     // 判断初值是否为0，用于清零大疆电机初始编码器带来的角度
} ST_ENCODER;

typedef struct
{
    ST_ENCODER motor_encoder; /**< 电机编码器信息 */
    float EncoderNum;         /**< 编码器计数值 */
    float encoder_speed;      /**< 编码器测得的速度 */
    float angle;              // deg
    float speed;              // rad/s
    float current;
    uint8_t init; // 用于将增量式编码器初始化
    float uiGearRatio;
} DJI_MOTOR;

typedef struct
{
    float angle; // deg
    float speed; // rad/s
    ST_LPF lpf;
    float gearratio;
} V6_MOTOR;
/* motor end */

/* vision begin */
// 定义单应性矩阵结构体，用于保存初始化后的矩阵参数
typedef struct {
    float h11, h12, h13;
    float h21, h22, h23;
    float h31, h32, h33;
    int is_valid; // 标记矩阵是否有效
} HomographyMatrix;

#pragma pack(push, 1)
typedef struct
{
    float x;
    float y;
} VISION_XY;

typedef struct
{
    uint8_t header;
    float radar_x;
    float radar_y;
    float radar_yaw;
    float y2;
    uint8_t r1_target_1;
    uint8_t r1_target_2;
    uint8_t r1_pick_type_1;
    uint8_t r1_pick_type_2;
    uint8_t num;
    uint8_t radar_delay;
    uint8_t tail;
} VISION_DATA;

typedef struct
{
    uint8_t header;
	uint8_t start_signal;
    uint8_t num;
    uint8_t tail;
} TO_VISION;

#pragma pack(pop)

typedef struct
{
    ST_POS pos;
    VISION_XY mf[14]; // 0和1方块前方对齐，逆时针顺序；下面的角点也类似
    VISION_XY arcpoint[8];

    float front_region_y;
    float back_region_y;
    float left_region_x;
    float right_region_x;

    float front_region_mid;
    float back_region_mid;
    float left_region_mid;
    float right_region_mid;

    uint16_t mac_spot1;
    uint16_t mac_spot2;
} VISION;
/* vision end */

/* QD board begin */
typedef struct
{
    uint16_t dt35[4];
    uint8_t travel_swtich[8];
    float dt35_front;
    float dt35_back;
    float dt35_left;
    float dt35_right;
    uint8_t ts_leftup;
    uint8_t ts_rightup;
    uint8_t ts_leftdown;
    uint8_t ts_rightdown;
} QD_BOARD;
/* QD board end */

/* gyro begin */
typedef struct
{
	float pitch;//单位是deg
	float roll;
	float yaw;
	float yaw_pre;
	float ax;//单位是m/s2
	float ay;
	float az;
	float wx;//单位是deg/s
	float wy;
	float wz;
}GYRO;
/* gyro end */

/* chassis begin */
typedef struct
{
    float pos; // 期望正转方向，deg，已做过连续化处理，不一定是[-180,180]
    float vel; // 轮子转速，rad/s
    float ff; // 前馈
    float steer_init_flag;
    float steer_init_angle; // 初始角度，deg
    float steer_output;
    float driver_output;
    DJI_MOTOR steer;
    V6_MOTOR driver;
} STEER_WHEEL;

typedef struct
{ // 右手系，y轴正方向为前进方向，w轴正方向为逆时针方向
    STEER_WHEEL leftup;
    STEER_WHEEL rightup;
    STEER_WHEEL leftdown;
    STEER_WHEEL rightdown;
    ST_TD td_Vx;
    ST_TD td_Vy;
    ST_TD td_W;
} STEER_WHEELS;

typedef enum
{
    NO_ERROR = 0,
    CHASSIS_INIT_FAIL = 1,         // 初始化失败
    CHASSIS_OUTPUT_OVERSIZE = 2,   // 输出过大
    LOCATION_EXCEPTION = 3,        // 定位异常
    REMOTE_EXCEPTION = 4,          // 遥控异常
    RADAR_LOSS = 5,                // 雷达失联
    NAV_STUCK = 6,                 // 导航卡住
    CHASSIS_VELOCITY_OVERSIZE = 7, // 过速保护
    CHASSIS_INIT_DISCPLT = 8,      // 初始化未完成
		GYRO_STUCK = 9                 // 陀螺仪卡死
} Error_t;

typedef enum
{
    CHASSIS_STANDBY = 0,
    CHASSIS_INIT = 1,
    CHASSIS_LOCAL_REMOTE = 2,
    CHASSIS_GLOBAL_REMOTE = 3,
    CHASSIS_LOCK = 4,
    CHASSIS_NAV = 5,
    CHASSIS_FIXED = 6,
    CHASSIS_TEST = 7
} CHASSIS_STATE;

typedef struct
{
    CHASSIS_STATE state;
    ST_VEL local_vel;
    STEER_WHEELS wheels;
    uint8_t init_flag;
    float fixed_vel;
    float fixed_cnt;
    float fixed_dir;
    uint32_t cnt;
    Error_t err;
} CHASSIS;
/* chassis end */

/* nav begin */
typedef enum
{
    NAV_STANDBY = 0,
    NAV_PATH = 1,
    NAV_LOCK = 2,
		NAV_MAC_TEST = 3,
    NAV_TEST = 4,
} NAV_STATE;

typedef struct
{
    NAV_STATE state;
    uint8_t init_flag;
    uint8_t mac_flag;//边走边吸开启的标志位
    PATH_OBJECT object;
    ST_POS final_pos;
    uint16_t cur_region; // 当前所在区域
    uint16_t cur_spot;   // 当前目标点
    float progress;
    ST_VEL global_vel;
    uint8_t radar_flag; // 雷达新的一帧
} NAV;

typedef struct
{
    float pos[3];
    float vel[3];
    float radar[3];
    float P[3];
    float R_obs[3];  // 观测噪声
    float Q_base[3]; // 过程噪声
    float Q_adapt;
} PosKF_t; // 位置卡尔曼滤波器

typedef struct{
    ST_POS pos;
    float P[3];
    ST_VEL vel;
    float residualSSE;
}KF_history_t;

typedef enum{
    ACC_UPDATE = 0x01,
    OMEGA_UPDATE = 0x02,
    YAW_UPDATE = 0x04
}imu_update_t;

typedef struct{
    ST_POS pos;
    ST_VEL vel;
    float ax;
    float ay;
    float P[8];//噪声矩阵，分别为x,y,yaw,vx,vy,omega,ax,ay
    uint8_t update_flag;
}full_status_t;

typedef struct{
    full_status_t status[DELAYED_KF_SIZE];
    float Q[8];//过程噪声
    float R_imu[4];//观测噪声,分别为yaw,omega,ax,ay
    float R_odom[3];//车身速度噪声系数
    float R_radar[3];//雷达噪声,分别为x,y,yaw
}DelayedFullKF_t;
/* nav end */

/* board communication begin */
#pragma pack(push, 1)
typedef struct
{
    uint8_t header;
    float remote_Vx;
    float remote_Vy;
    float remote_W;
    uint32_t nav_target;
    uint8_t chassis_action;
    uint8_t upper_cplt;
    uint8_t QR_byte;//0是无效，全是0且chassis_action == 11 属于错误情况
    uint8_t KFS_2;
    uint8_t KFS_3;
    uint8_t tail;
} FROM_UPPER;

typedef struct
{
    uint8_t header;
    float progress;
    uint8_t fps_err;
    float t_remain;
    uint8_t mac_flag;
    int16_t vis_x;
    int16_t vis_y;
    int16_t vis_yaw;
    uint8_t tail;
} TO_UPPER;

typedef struct{
	uint8_t header[2];
	uint8_t addr;
	uint8_t data[4];
	uint8_t tail[2];
}TRT_DATA;
#pragma pack(pop)
/* board communication end*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern osThreadId_t action_taskHandle;
extern osMessageQueueId_t Gyro_QueueHandle;

extern uint8_t uart4_rxbuf[200];
extern uint8_t uart1_rxbuf[200];

extern uint8_t g_uart_rx_buf[512]; /*rx DMA buffer of uart2*/
extern uint16_t g_uart_rx_cnt; /*reciede data length of uart2*/
extern uint8_t g_decode_data[512];     /*buffer for decoding*/
extern uint16_t g_decode_data_pos; /*bytes left in decode buffer*/
extern uint8_t g_update_flag;
extern float g_pre_yaw;
extern float g_yaw;
extern float g_ax;
extern float g_ay;
extern float g_omega;
extern uint32_t g_stuck_cnt;

extern SYSTEM_MONITORS sys_mnt;
extern VISION_DATA Vision_Data;
extern VISION Vision;
extern TO_VISION To_Vision;
extern QD_BOARD QD;
extern GYRO Gyro;
extern CHASSIS Chassis;
extern ST_VEL Remotevel;
extern ST_PID pid_remote_yaw;
extern ST_PID pid_leftup_driver;
extern ST_PID pid_rightup_driver;
extern ST_PID pid_leftdown_driver;
extern ST_PID pid_rightdown_driver;
extern ST_CASCADE_PID pid_leftup_steer;
extern ST_CASCADE_PID pid_rightup_steer;
extern ST_CASCADE_PID pid_leftdown_steer;
extern ST_CASCADE_PID pid_rightdown_steer;
extern ST_TD td_radar_x;
extern ST_TD td_radar_y;
extern ST_TD td_radar_yaw;
extern ST_FF ff_leftup_driver;
extern ST_FF ff_rightup_driver;
extern ST_FF ff_leftdown_driver;
extern ST_FF ff_rightdown_driver;
extern ST_POS location;
extern ST_POS location_to_upper;
extern ST_VEL wheeltobody_vel;
extern float wheeltobody_vel_residualSSE;
extern NAV Nav; // 定义导航
extern ST_PID pid_x;
extern ST_PID pid_y;
extern ST_PID pid_yaw; // 定义导航PID
extern float nav_ff_k;
extern float nav_ff_k_W;
extern PosKF_t PosKF;
extern DelayedFullKF_t DFKF;
extern TO_UPPER data_to_upper;
extern FROM_UPPER data_from_upper;
extern ST_VEL blank_vel;
extern float _mac_vel[3];
extern uint8_t mac_des_point[3];
extern uint8_t last_spot;
extern uint8_t dt35_semaphore;
extern HomographyMatrix H_matrix;
extern TRT_DATA TRT_data;

void System_Monitor(SYSTEM_MONITORS *sm);
void BSP_Init(void);
#endif // ___ROBOT_H___
