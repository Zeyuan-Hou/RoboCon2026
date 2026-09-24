#include "HIPNUC_gyro.h"

// static hipnuc_raw_t hipnuc_raw = {0};陀螺仪解算出来的数据都写在这里了

/* Enable/Disable fixed data array decoding example */
//#define ENABLE_FIXED_DATA_EXAMPLE 0 //这个没啥用

/* Enable/Disable DMA for USART reception */
//#define ENABLE_USART_DMA 1 //不用DMA就是0，开启DMA就是1,没屁用

//自己配置
// #define USART1_BAUD 115200
// #define USART2_BAUD 115200


/* IMU stream read/control struct */
 hipnuc_raw_t hipnuc_raw = {0};


/* 0: no new data arrived, 1: new data arrived */
 uint8_t new_data_flag = 0;

/* The char buffer used to show result */
 char log_buf[LOG_STRING_SIZE];

 uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
 uint8_t dma_rx_buf[UART_RX_BUF_SIZE];
 uint16_t uart_rx_index = 0; //接收数据计数器

/* Function prototypes */
 void app_init(void);
 void process_data(void);
void handle_usart_rx_idle(UART_HandleTypeDef *huart, uint16_t rx_size);

//其实就是开启接收的一个带延迟的启动
/**
 * @brief System initialization
 *
 * Initializes delay, NVIC priority, USART1 for debug output, USART2 for IMU data reception,
 * and optionally DMA for USART2 reception. Also initializes the IMU instance.
 */
 void app_init(void)
{

    /* Initialize delay */
    // delay_init();
    // delay_ms(1);

    HAL_Delay(1); //手写一个启动延时

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级配置

    /* Initialize USART1 for debug output */
    /* Initialize USART2 for IMU data reception */
    // USART_Configuration(USART1_BAUD, USART2_BAUD);//实际上是配置USART1和USART2的函数，没用

    // #if ENABLE_USART_DMA//不用看这一步
    //     /* Initialize DMA for USART2 reception */
    //     DMA_Configuration();
    // #endif

    /* Initialize instance */
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
    new_data_flag = 0;
		
		 // 开启 USART2 空闲中断 DMA 接收（HAL 最新写法）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, UART_RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

//处理数据//这个是一直要运行的函数
/**
 * @brief Process IMU data
 *
 * If new data is available, processes the received IMU data by decoding it and printing the results.
 */
 void process_data(void)
{
    if (new_data_flag) //新数据来了
    {
        new_data_flag = 0;

        for (uint16_t i = 0; i < uart_rx_index; i++)
        {
            if (hipnuc_input(&hipnuc_raw, uart_rx_buf[i]))
            {
                /* Convert result to strings */
							
//                hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
							
                //gyro_data的单位是0.1°							
                gyro_data.pitch=hipnuc_raw.hi91.pitch*10.f;
							  gyro_data.yaw=hipnuc_raw.hi91.yaw*10.f;
							  gyro_data.roll=hipnuc_raw.hi91.roll*10.f;
							
							  system_monitor.gyro_cnt++;
							
                /* Display result */
                //没用，注释掉了
                // printf("parse ok, frame len:%d\r\n", hipnuc_raw.len);
                // printf("%s\r\n", log_buf);

                /*
                You can use data as for example:

                printf("acc:%.3f, %.3f, %.3f\r\n", hipnuc_raw.hi91.acc[0], hipnuc_raw.hi91.acc[1], hipnuc_raw.hi91.acc[2]);
                */
            }
        }

        uart_rx_index = 0; // Reset buffer index after processing
    }
}

//随自己使用修改串口，这个是标准库的，没法用
/**
 * @brief USART2 interrupt handler
 *
 * Handles USART2 interrupts for IDLE line detection and RXNE (Receive Not Empty).
 */
//void USART2_IRQHandler(void)
//{
//    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
//    {
//        /* Clear IDLE line detected bit */
//        USART_ReceiveData(USART2);

//        handle_usart_rx_idle();
//    }

//#if !ENABLE_USART_DMA
//    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//        uint8_t ch = USART_ReceiveData(USART2);
//        uart_rx_buf[uart_rx_index++] = ch;
//        if (uart_rx_index >= UART_RX_BUF_SIZE)
//        {
//            uart_rx_index = 0; // Prevent buffer overflow
//        }
//    }
//#endif
//}

// HAL 库 ReceiveToIdle 专用（处理DMA接收完成）
void handle_usart_rx_idle(UART_HandleTypeDef *huart, uint16_t rx_size)
{
    // 把收到的数据 → 复制到解析缓冲区
    for (uint16_t i = 0; i < rx_size; i++)
    {
        uart_rx_buf[uart_rx_index++] = dma_rx_buf[i];
        if (uart_rx_index >= UART_RX_BUF_SIZE)
            uart_rx_index = 0;
    }

    // 标记：新数据到了，可以解析
    new_data_flag = 1;
		


    // 重新开启下一次接收（关键！）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, UART_RX_BUF_SIZE);
}

//随自己使用修改dma通道，这个是标准库的，没法用
/**
 * @brief Handle USART RX data when IDLE line is detected
 *
 * Processes received data and resets DMA if DMA is used.
 */
//void handle_usart_rx_idle(void)
//{
//#if ENABLE_USART_DMA
//    /* Disable DMA1 Channel6 */
//    DMA_Cmd(DMA1_Channe5, DISABLE);

//    /* Get number of bytes received */
//    uint16_t rx_size = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channe5);

//    /* Process received data */
//    for (uint16_t i = 0; i < rx_size; i++)
//    {
//        uart_rx_buf[uart_rx_index++] = dma_rx_buf[i];

//        if (uart_rx_index >= UART_RX_BUF_SIZE)
//        {
//            uart_rx_index = 0; // Prevent buffer overflow
//        }
//    }

//    /* Reset DMA1 Channel6 */
//    DMA_SetCurrDataCounter(DMA1_Channe5, UART_RX_BUF_SIZE);

//    /* Re-enable DMA1 Channel6 */
//    DMA_Cmd(DMA1_Channe5, ENABLE);
//#endif

//    /* Set flag to indicate new data available */
//    new_data_flag = 1;
//}
