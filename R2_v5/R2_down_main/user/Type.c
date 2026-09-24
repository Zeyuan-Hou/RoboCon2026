#include "Type.h"

//帧率检测
ST_SYSTEM_MONITOR monitor = {0};


uint8_t ramp_test_flag = 0;
uint8_t flag_y=0;
uint8_t flag_x=0;

uint8_t ramp_state = 0;

//气动板
uint8_t airCtrl[1] = {0};
uint8_t AirOperaterCtrlBuf[6] = {0};
uint8_t travel_switch;
uint8_t travel_switch_mode[8];
DT35_distanceT DT35_temp = {0};
DT35_distanceT DT35_fact = {0};
float DT35_X_fact,DT35_Y_fact= 0;


//GO1 + TD
RIS_MotorData_t uart_rx_data;

MotorCmd_t cmd_1 = {.id = 1,.flag_init = 0,};
MotorData_t data_1 = {0};

MotorCmd_t cmd_2 = {.id = 2,.flag_init = 0,};
MotorData_t data_2 = {0};

MotorCmd_t cmd_3 = {.id = 3,.flag_init = 0,};
MotorData_t data_3 = {0};

MotorCmd_t cmd_4 = {.id = 4,.flag_init = 0,};
MotorData_t data_4 = {0};

ST_TD GO1_td_1 = {.r = 7500,.T = 0.001,.h = 0.002,.aim = 0,.x = 0,.x1 = 0,.x2 = 0};
ST_TD GO1_td_2 = {.r = 7500,.T = 0.001,.h = 0.002,.aim = 0,.x = 0,.x1 = 0,.x2 = 0};
ST_TD GO1_td_3 = {.r = 7500,.T = 0.001,.h = 0.002,.aim = 0,.x = 0,.x1 = 0,.x2 = 0};
ST_TD GO1_td_4 = {.r = 7500,.T = 0.001,.h = 0.002,.aim = 0,.x = 0,.x1 = 0,.x2 = 0};



//视觉
uint8_t vision_rec[166] = {0}; // 接收
vision_Data_t vision_data_recieve ={0};
uint8_t vision_rec_test[166] = {0}; //视觉接收校验，防止数据错位


//大疆电机+PID
ST_DJI_MOTOR DJI_MOTOR= 
{
	.DJI_1 = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0 , .state = 0}},
	.DJI_2 = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0 , .state = 0}},
};

ST_DJI_RUN dji_run =
{
	.DJI_1=	
	{
	.fpKp=100.f,
	.fpKi=3.5f,
	.fpKd=0.0f,
	
	.fpUMax=9000.f,
	.fpUpMax=8000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=600.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f
	},
	
	.DJI_2=	
	{
	.fpKp=100.f,
	.fpKi=3.5f,
	.fpKd=0.0f,
	
	.fpUMax=9000.f,
	.fpUpMax=8000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=600.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f	
	},
	.flag_2006_V = 0
};


//底盘轮子电机 + PID
MOTOR_WHEEL motor_wheel ={0};
uint8_t flag_wheel_slope = 0;  //轮子上斜坡时的前馈标志位
//ST_Chassis_Run chassis_run=   //有前馈
//{
//	.wheel_1 =
//{
//	.fpKp=180.f,
//	.fpKi=1.7f,
//	.fpKd=0.f,
//	
//	.fpUMax=22000.f,
//	.fpUpMax=20000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=800.f,
//	
//	.fpEMax=400.f,
//	.fpEMin=0.0f,
//	.fpKffv = 10.f,
//	.fpforward_0 = 0.f,
//	.fpKffa = 10000.0f,
//	.slope_forward = 5000.0f,
//},
//	.wheel_2 =
//{
//	.fpKp=180.f,
//	.fpKi=1.7f,
//	.fpKd=0.f,
//	
//	.fpUMax=22000.f,
//	.fpUpMax=20000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=800.f,
//	
//	.fpEMax=400.f,
//	.fpEMin=0.0f,
//	.fpKffv = 10.f,
//	.fpforward_0 = 0.f,
//	.fpKffa = 10000.0f,
//	.slope_forward = 5000.0f,
//},
//	.wheel_3 =
//{
//	.fpKp=180.f,
//	.fpKi=1.7f,
//	.fpKd=0.f,
//	
//	.fpUMax=22000.f,
//	.fpUpMax=20000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=800.f,
//	
//	.fpEMax=400.f,
//	.fpEMin=0.0f,
//	.fpKffv = 10.f,
//	.fpforward_0 = 0.f,
//	.fpKffa = 10000.0f,
//	.slope_forward = -5000.0f,

//},
//	.wheel_4 =
//{
//	.fpKp=180.f,
//	.fpKi=1.7f,
//	.fpKd=0.f,
//	
//	.fpUMax=22000.f,
//	.fpUpMax=20000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=800.f,
//	
//	.fpEMax=400.f,
//	.fpEMin=0.0f,
//	.fpKffv = 10.f,
//	.fpforward_0 = 0.f,
//	.fpKffa = 10000.0f,
//	.slope_forward = -5000.0f,
//}
//};

ST_Chassis_Run chassis_run=   //无前馈
{
	.wheel_1 =
{
	.fpKp=180.f,
	.fpKi=5.f,
	.fpKd=0.f,
	
	.fpUMax=22000.f,
	.fpUpMax=20000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=500.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f,
	.fpforward_0 = 1200.f,   //静摩擦
	.ramp_foward=5500.f,
	
	.fpKffa = 0.0f,
	
},
	.wheel_2 =
{
	.fpKp=180.f,
	.fpKi=5.f,
	.fpKd=0.f,
	
	.fpUMax=22000.f,
	.fpUpMax=20000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=500.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f,
	.fpforward_0 = 1200.f,   //静摩擦
	.ramp_foward=5500.f,

	.fpKffa = 0.0f,	
	
},
	.wheel_3 =
{
	.fpKp=180.f,
	.fpKi=5.f,
	.fpKd=0.f,
	
	.fpUMax=22000.f,
	.fpUpMax=20000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=500.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f,
	.fpforward_0 = 1200.f,   //静摩擦
	.ramp_foward=5500.f,
	
	.fpKffa = 0.0f,	
	
},
	.wheel_4 =
{
	.fpKp=180.f,
	.fpKi=5.f,
	.fpKd=0.f,
	
	.fpUMax=22000.f,
	.fpUpMax=20000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=500.f,
	
	.fpEMax=400.f,
	.fpEMin=0.0f,
	.fpforward_0 = 1200.f,   //静摩擦
	.ramp_foward=5500.f,
	
	.fpKffa = 0.0f,	
	
}
};
//导航
ST_Nav nav = {
	.nav_state = NAV_INIT,
	.auto_path = {
	.pos_pid = {
	.pid_x = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 1500.f, .fpEMin = 3.0f},
	.pid_y = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 1500.f, .fpEMin = 3.0f},
	.pid_w = {.fpKp = 2.5f, .fpKi = 0.000f, .fpKd = 0.0f, .fpUMax = 20.f, .fpUpMax = 8.0f, .fpUdMax = 1000.f, .fpSumEMax = 10.f, .fpEMax = 4.f, .fpEMin = 0.0f}		},
	},
};
POS_TRACKER point_only ={
	.pid_x = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 1500.f, .fpEMin = 3.0f},
	.pid_y = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 1500.f, .fpEMin = 3.0f},
	.pid_w = {.fpKp = 2.5f, .fpKi = 0.000f, .fpKd = 0.0f, .fpUMax = 20.f, .fpUpMax = 8.0f, .fpUdMax = 1000.f, .fpSumEMax = 10.f, .fpEMax = 4.f, .fpEMin = 0.0f}
};


POS_TRACKER point_2_area ={
	.pid_x = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 300.f, .fpEMax = 500.f, .fpEMin = 3.0f},
	.pid_y = {.fpKp = 1.74f, .fpKi = 0.0f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 300.f, .fpEMax = 500.f, .fpEMin = 3.0f},
	.pid_w = {.fpKp = 2.5f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 20.f, .fpUpMax = 8.0f, .fpUdMax = 1000.f, .fpSumEMax = 10.f, .fpEMax = 4.f, .fpEMin = 0.0f}
};

POS_TRACKER NAV_LOCK_POS_PID ={
	.pid_x = {.fpKp = 1.6f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 1200.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 500.f, .fpEMin = 0.0f},
	.pid_y = {.fpKp = 1.6f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 2500.f, .fpUpMax = 1200.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 500.f, .fpEMin = 0.0f},
	.pid_w = {.fpKp = 2.0f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 20.f, .fpUpMax = 8.0f, .fpUdMax = 1000.f, .fpSumEMax = 10.f, .fpEMax = 4.f, .fpEMin = 0.0f}
};

POS_TRACKER NAV_DT35_PID ={
	.pid_x = {.fpKp = 2.85f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 3200.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 500.f, .fpEMin = 0.0f},
	.pid_y = {.fpKp = 2.85f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 3200.f, .fpUpMax = 2000.f, .fpUdMax = 3000.f, .fpSumEMax = 400.f, .fpEMax = 500.f, .fpEMin = 0.0f},
	.pid_w = {.fpKp = 1.5f, .fpKi = 0.00f, .fpKd = 0.0f, .fpUMax = 20.f, .fpUpMax = 8.0f, .fpUdMax = 1000.f, .fpSumEMax = 10.f, .fpEMax = 4.f, .fpEMin = 0.0f}
};
ST_POS robot_pos = {0}; 	/*机器人反馈坐标（实时更新）*/
header_pos_t header_pos = {0};		//夹头坐标（按照视觉发来的信息排序）
target_t area_one_target = {0};		//一区点到点目标值
target_t area_two_target = {0};		//二区点到点目标值
target_t area_three_target = {0};	//三区点到点目标值
float K_VEL_X,K_VEL_Y,K_VEL_W = 1; //导航速度规划前馈的系数
chassis_run_des straight_des ,rotation_des = {0};	//轮子速度分配
allow_t allow_nav = {0};  //导航容许范围
allow_t allow_point = {0};  //点到点容许范围
uint8_t reach_kfs_num = 0; //经过了规划的第几个KFS
Cube_Line cube_x, cube_y, cube_w; //三次曲线样条系数
PATH_POINT Path_Point = {.point_inx = 0, .flag_point_to_point = 1, .flag_cube_set =0, .point_tim = 0}; //路线类型
uint8_t point_only_state = 0; //纯点到点坐标选取状态
uint8_t only_point_range_state = 0; //点到点范围标志位
float offset_2006 = 0; //三区站起来用2006小轮纠位置
uint8_t gyro_or_radar = 0; //当前导航yaw反馈基于雷达还是陀螺仪标志位  0为陀螺仪 1为雷达
 


//上下台阶标志位
uint8_t lift_state = 0;//四条腿站起状态
uint8_t lift_state_two = 0;//前两条腿垫高状态
uint8_t up_down_state = 0;//总标志位
uint8_t up_state_200 = 0; 
uint8_t up_state_400 = 0;
uint8_t down_state_200 = 0;
uint8_t down_state_400 = 0;
uint8_t combine_state = 0; //合体
//GO1参数切换标志位
uint8_t GO1_state_change = 0;
//导航精度切换标志位
uint8_t nav_reach_state = 0;
//底盘和遥控标志位
ST_FLAG ctrl_flag = {0};
//导航路径切换标志位
uint8_t flag_lock = 0;  //到达目标锁住
uint8_t all_path_state = 0; //总路径状态
uint8_t path_state_1	= 0;  //一区
uint8_t path_state_2	= 0;	//二区
uint8_t path_state_3	= 0;	//三区 
uint8_t path_state_4	= 0;  //三区回一区重试
uint8_t first_path_state = 0;  //路径切换
//一区可以执行准备动作标志位
uint8_t flag_one_area_ready = 0;
//导航区域切到点到点的标志位
uint8_t flag_area_to_point = 0;


//串口通信
uint8_t uart4_receive[21] = {0};	//板间串口
uint8_t inner_send[8] = {0};    //板间发送
uint8_t inner_receive[21] = {0}; //板间接收
uint8_t uart_ctrl_receive[4] = {0}; //硬件键盘串口
uint8_t key_receive[4] = {0};		//硬件键盘接收
uint8_t key_send[5] = {0};		//硬件键盘发送
float yaw_gyro = 0;  //上层传过来角度
float yaw_gyro_rad = 0;  
float gyro_w = 0; //上层传回角速度
uint8_t vision_send[3] = {0}; //向视觉发送标志位

//测试用
uint8_t remote_up_down = 0;
float ucGateX = 2500;
float ucGateY = 2500;
float ucGateW = 1000;
float ssXSpedLimit = 3000;
float ssYSpedLimit = 3000;
float ssWSpedLimit = 300;
float test_rad = 0;
float test_forward = 0;
uint8_t flag_up = 0;
target_t target_test = {0};
uint16_t cnt_test = 0;
uint16_t fps_test = 0;
float cmd_1_T,cmd_2_T,cmd_3_T,cmd_4_T = 0;
uint8_t test_flag = 0;
float test_f = 800;



//适应性训练遥控发送
uint8_t remote_send[25] = {0};
MONITOR_ERROR monitor_error = {0};
uint8_t remote_state = 0;
float test_remote_x = 1;
float test_remote_y = 1;
float test_wheel_1 = 0;



