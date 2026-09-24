#include "robot.h"
uint8_t uart4_rxbuf[200];
uint8_t uart1_rxbuf[200];

uint8_t g_uart_rx_buf[512]; /*rx DMA buffer of uart2*/
uint16_t g_uart_rx_cnt = 0; /*reciede data length of uart2*/
uint8_t g_decode_data[512];     /*buffer for decoding*/
uint16_t g_decode_data_pos = 0; /*bytes left in decode buffer*/
uint8_t g_update_flag = 0;
float g_pre_yaw = 0;
float g_yaw = 0;
float g_omega = 0;
uint32_t g_stuck_cnt = 0;


SYSTEM_MONITORS sys_mnt; //定义系统监视器
VISION_DATA Vision_Data;
VISION Vision;
TO_VISION To_Vision = {
	.header = 0x66,
    .start_signal = 1,
	.num = 1,
	.tail = 0x99
};
ST_TD td_radar_x = {
    .h = 0.3,//1,//0.033, 
    .T = 0.001,
    .r = 20//4//10
};
ST_TD td_radar_y = {
    .h = 0.3,//1,//0.033, 
    .T = 0.001,
    .r = 20//4//10
};
ST_TD td_radar_yaw = {
    .h = 0.03, 
    .T = 0.001,
    .r = 3000,
};

QD_BOARD QD;
GYRO Gyro;
ST_VEL Remotevel;


ST_PID pid_remote_yaw= {
    .fpKp = 2.5f,
		.fpKi = 0.01f,
		.fpKd = 0.f,
    .fpUMax = 150.f,
    .fpUpMax = 150.f,
    .fpUiMax = 50.f,
    .fpUdMax = 25.f,
    .fpSumEMax = 50.f,
    .fpElimit = 1000.f,
    .fpEMin = 0.f
};


CHASSIS Chassis={
    .wheels={
        .leftdown={
			.pos = 90,
            .driver={
                .gearratio = V6_GEARRATIO,
                .lpf = {
                    .off_freq = 50,
                    .samp_tim = 0.001
                }
            },
            .steer={
                .motor_encoder={
                    .siNumber = 8192
                },
                .uiGearRatio = M2006_GEARRATIO*3
            }
        },
        .leftup={
			.pos = 90,
            .driver={
                .gearratio = V6_GEARRATIO,
                .lpf = {
                    .off_freq = 50,
                    .samp_tim = 0.001
                }
            },
            .steer={
                .motor_encoder={
                    .siNumber = 8192
                },
                .uiGearRatio = M2006_GEARRATIO*3
            }
        },
        .rightdown={
			.pos = -90,
            .driver={
                .gearratio = V6_GEARRATIO,
                .lpf = {
                    .off_freq = 50,
                    .samp_tim = 0.001
                }
            },
            .steer={
                .motor_encoder={
                    .siNumber = 8192
                },
                .uiGearRatio = M2006_GEARRATIO*3
            }
        },
        .rightup={
			.pos = -90,
            .driver={
                .gearratio = V6_GEARRATIO,
                .lpf = {
                    .off_freq = 50,
                    .samp_tim = 0.001
                }
            },
            .steer={
                .motor_encoder={
                    .siNumber = 8192
                },
                .uiGearRatio = M2006_GEARRATIO*3
            }
        },
        .td_Vx = {
            .r = 10000,
            .h = 0.005f,
            .T = 0.001f
        },
        .td_Vy = {
            .r = 10000,
            .h = 0.005f,
            .T = 0.001f
        },
        .td_W = {
            .r = 1300,
            .h = 0.005f,
            .T = 0.001f
        }
    }
}; //定义底盘
ST_CASCADE_PID pid_leftdown_steer={
    .outer={
        .fpKp = 0.28f, 
        .fpKd = 400.f,
			  .fpKi = 0.f,
				.fpElimit = 10.f,
				.fpSumEMax = 2000.f,
        .fpUMax = 10000.f,
        .fpUpMax = 10000.f,
        .fpUdMax = 0.f,
				.fpUiMax = 0.f
    },
    .inner={
        .fpKp = 1300.f, 
				.fpKi = 0.75f,
        .fpElimit = 30.f,
        .fpSumEMax = 800.f,
        .fpUMax = 9000.f,
        .fpUpMax = 9000.f,
        .fpUiMax = 2000.f,
			  .fpUdMax = 0.f
    }
};
ST_CASCADE_PID pid_leftup_steer={
    .outer={
        .fpKp = 0.28f, 
        .fpKd = 400.f,
			  .fpKi = 0.f,
				.fpElimit = 10.f,
				.fpSumEMax = 2000.f,
        .fpUMax = 10000.f,
        .fpUpMax = 10000.f,
        .fpUdMax = 0.f,
				.fpUiMax = 0.f
    },
    .inner={
        .fpKp = 1300.f, 
				.fpKi = 0.75f,
        .fpElimit = 30.f,
        .fpSumEMax = 800.f,
        .fpUMax = 9000.f,
        .fpUpMax = 9000.f,
        .fpUiMax = 2000.f,
			  .fpUdMax = 0.f
    }
};
ST_CASCADE_PID pid_rightup_steer={
    .outer={
        .fpKp = 0.28f, 
        .fpKd = 400.f,
			  .fpKi = 0.f,
				.fpElimit = 10.f,
				.fpSumEMax = 2000.f,
        .fpUMax = 10000.f,
        .fpUpMax = 10000.f,
        .fpUdMax = 0.f,
				.fpUiMax = 0.f
    },
    .inner={
        .fpKp = 1300.f, 
				.fpKi = 0.75f,
        .fpElimit = 30.f,
        .fpSumEMax = 800.f,
        .fpUMax = 9000.f,
        .fpUpMax = 9000.f,
        .fpUiMax = 2000.f,
			  .fpUdMax = 0.f
    }
};
ST_CASCADE_PID pid_rightdown_steer={
    .outer={
        .fpKp = 0.28f, 
        .fpKd = 400.f,
			  .fpKi = 0.f,
				.fpElimit = 10.f,
				.fpSumEMax = 2000.f,
        .fpUMax = 10000.f,
        .fpUpMax = 10000.f,
        .fpUdMax = 0.f,
				.fpUiMax = 0.f
    },
    .inner={
        .fpKp = 1300.f, 
				.fpKi = 0.75f,
        .fpElimit = 30.f,
        .fpSumEMax = 800.f,
        .fpUMax = 9000.f,
        .fpUpMax = 9000.f,
        .fpUiMax = 2000.f,
			  .fpUdMax = 0.f
    }
};

ST_PID pid_leftup_driver={
    .fpKp = 400.f,//500.f,//300.f,//700.f, 
	  .fpKi = 5.f,//0.1f,//0.45f,
	  .fpKd = 0.f,
    .fpElimit = 30.f,
    .fpSumEMax = 100.f,
   	.fpUMax = 20000.f,
  	.fpUpMax = 20000.f,
   	.fpUiMax = 4000.f,
		.fpUdMax = 0.f
};
ST_PID pid_rightup_driver={
    .fpKp = 350.f,//500.f,//300.f,//700.f, 
	  .fpKi = 3.f,//0.1f,//0.45f,
	  .fpKd = 0.f,
    .fpElimit = 30.f,
    .fpSumEMax = 150.f,
   	.fpUMax = 20000.f,
  	.fpUpMax = 20000.f,
   	.fpUiMax = 4000.f,
		.fpUdMax = 0.f
};
ST_PID pid_leftdown_driver={
    .fpKp = 400.f,//500.f,//300.f,//700.f, 
	  .fpKi = 3.5f,//0.1f,//0.45f,
	  .fpKd = 0.f,
    .fpElimit = 30.f,
    .fpSumEMax = 150.f,
   	.fpUMax = 20000.f,
  	.fpUpMax = 20000.f,
   	.fpUiMax = 4000.f,
		.fpUdMax = 0.f
};
ST_PID pid_rightdown_driver={
    .fpKp = 400.f,//500.f,//300.f,//700.f, 
	  .fpKi = 3.5f,//0.1f,//0.45f,
	  .fpKd = 0.f,
    .fpElimit = 30.f,
    .fpSumEMax = 150.f,
   	.fpUMax = 20000.f,
  	.fpUpMax = 20000.f,
   	.fpUiMax = 4000.f,
		.fpUdMax = 0.f
};

ST_FF ff_leftup_driver={
    .td = {
        .h = 0.005f,
        .r = 2500.f,
        .T = 0.001f},
	.k0 = 0,
	.k1 = 25,
	.k2=45//10000//16000
};
ST_FF ff_rightup_driver={
    .td = {
        .h = 0.005f,
        .r = 2500.f,
        .T = 0.001f},
	.k0 = 0,
	.k1 = 25,
	.k2=45//10000//16000
};
ST_FF ff_leftdown_driver={
    .td = {
        .h = 0.005f,
        .r = 2500.f,
        .T = 0.001f},
	.k0 = 0,
	.k1 = 25,
	.k2=45//10000//16000
};
ST_FF ff_rightdown_driver={
    .td = {
        .h = 0.005f,
        .r = 2500.f,
        .T = 0.001f},
	.k0 = 0,
	.k1 = 25,
	.k2=45//10000//16000
};


ST_POS location; //定义位置
ST_POS location_to_upper;
ST_VEL wheeltobody_vel;//轮速反解算车身速度，全局坐标系
float wheeltobody_vel_residualSSE;
HomographyMatrix H_matrix = {0};

NAV Nav; //定义导航
ST_PID pid_x = {
    .fpKp = 1.6f,
//    .fpKi = 0.013f,
//    .fpKd = 3.f,

    .fpUMax = 3000.f,
    .fpUpMax = 3000.f,
    .fpUiMax = 2000.f,
    .fpUdMax = 500.f,
    .fpSumEMax = 3000.f,
    .fpElimit = 100.f,
    .fpEMin = 0.f

};
ST_PID pid_y = {
    .fpKp = 1.6f,
//    .fpKi = 0.013f,
//    .fpKd = 3.f,

    .fpUMax = 3000.f,
    .fpUpMax = 3000.f,
    .fpUiMax = 2000.f,
    .fpUdMax = 500.f,
    .fpSumEMax = 3000.f,
    .fpElimit = 100.f,
    .fpEMin = 0.f
};
ST_PID pid_yaw = {
    .fpKp = 3.f,
//    .fpKi = 0.005f,
//    .fpKd = 5.6f,

    .fpUMax = 150.f,
    .fpUpMax = 150.f,
    .fpUiMax = 50.f,
    .fpUdMax = 25.f,
    .fpSumEMax = 2000.f,
    .fpElimit = 5.f,
    .fpEMin = 0.f

}; // 定义导航PID

PosKF_t PosKF = {
    .Q_base = {0.05f, 0.05f, 1.f},
    .R_obs = {25,25, 0.01f},//{90, 90, 0.125f},//现在角度测量噪声为0，表示完全相信雷达给出的角度值
    .P = {1,1,1},
    .Q_adapt = 0.1f // 定义位置卡尔曼滤波器
};

DelayedFullKF_t DFKF = {
    .Q = {0.05f, 0.05f, 0.01f, 0.1f, 0.1f, 0.1f, 0.5f, 0.5f},
    .R_imu = {0.01f, 0.1f, 10, 10},
    .R_odom = {5, 5, 0.5f},
    .R_radar = {25, 25, 0.01f}
};

TRT_DATA TRT_data={
	.header = {0x55, 0xAA},
	.addr = 0x51,
	.tail = {0x01, 0x55}
};


float nav_ff_k = 0.85f;//导航前馈系数
float nav_ff_k_W = 0.8f;

TO_UPPER data_to_upper = {0};
FROM_UPPER data_from_upper = {0};

ST_VEL blank_vel = {0,0,0};
float _mac_vel[3] = {0}; 
uint8_t mac_des_point[3] ={0};
uint8_t last_spot = 0;
uint8_t dt35_semaphore = 0;

void System_Monitor(SYSTEM_MONITORS *sm){
    uint16_t *p=(uint16_t *)sm;
    uint16_t size = sizeof(SYSTEM_MONITOR)/sizeof(uint16_t);
    for(uint8_t i=0; i<size; i++){
        p[i+size]=p[i];
        p[i]=0;
    }
}

HAL_StatusTypeDef status;

void BSP_Init(void){
    CAN_INIT();
    HAL_UART_Receive_DMA(&huart4, uart4_rxbuf, 20);
		HAL_UART_Receive_DMA(&huart1, uart1_rxbuf, sizeof(VISION_DATA));
	
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)g_uart_rx_buf, 512); // 串口帧固定为14字节
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);


// 启动接收
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)g_uart_rx_buf, 512);

//    __HAL_UART_DISABLE(&huart3);
//    __HAL_UART_FLUSH_DRREGISTER(&huart3);
//    __HAL_UART_CLEAR_OREFLAG(&huart3);
//		HAL_UART_Receive_DMA(&huart3, (uint8_t *)g_uart_rx_buf, 512);
//    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
//    __HAL_UART_ENABLE(&huart3);
//	// 1. 清除上电初期可能积累的错误标志
//    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
//    
//    // 2. 使用官方“空闲中断+DMA”API 启动接收
//    HAL_StatusTypeDef status;
//    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)g_uart_rx_buf, 512);
//    if(status != HAL_OK) {
//        // 可以在这里打断点，如果返回非 OK，说明初始化参数有错
//    }
	
	
	
	
}








