#include "main.h"
#include "ADS1256.h"
#include "spi.h"

//复位ADS1256
void ADS1256_Init(void)
{
	//复位
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);    // RESET为高电平
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS引脚拉低，进行SPI通信
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);  // RESET为低电平 - 复位			

	HAL_Delay(100);
//	vTaskDelay(pdMS_TO_TICKS(100));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);    // RESET为高电平
	
	HAL_Delay(10);
//	vTaskDelay(pdMS_TO_TICKS(10));
	// 等待 DRDY 引脚为低
	while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) == GPIO_PIN_SET);  // 如果这个引脚为高则卡在这里，直至变低，数据处理好

	// 同步和唤醒 ADS1256
	HAL_SPI_Transmit(&hspi1, (uint8_t[]){ADS1256_CMD_SYNC}, 1, HAL_MAX_DELAY);        // 同步命令
	HAL_SPI_Transmit(&hspi1, (uint8_t[]){ADS1256_CMD_WAKEUP}, 1, HAL_MAX_DELAY);      // 唤醒命令

	// 等待 DRDY 引脚为低
	while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) == GPIO_PIN_SET);

	// 配置 ADS1256 的状态寄存器STATUS，连续写入4个寄存器
	uint8_t status_config[] = {ADS1256_CMD_WREG | ADS1256_STATUS, 0x00, 0x06};
	HAL_SPI_Transmit(&hspi1, status_config, sizeof(status_config), HAL_MAX_DELAY); // 写入状态寄存器

	// 配置 MUX 寄存器
	uint8_t mux_config[] = {ADS1256_CMD_WREG | ADS1256_MUX, 0x00, ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM};
	HAL_SPI_Transmit(&hspi1, mux_config, sizeof(mux_config), HAL_MAX_DELAY); // 配置通道

	// 配置 ADCON 寄存器
	uint8_t adcon_config[] = {ADS1256_CMD_WREG | ADS1256_ADCON, 0x00, ADS1256_GAIN_1};
	HAL_SPI_Transmit(&hspi1, adcon_config, sizeof(adcon_config), HAL_MAX_DELAY); // 设置增益

	// 配置 DRATE 寄存器
	uint8_t drate_config[] = {ADS1256_CMD_WREG | ADS1256_DRATE, 0x00, ADS1256_DRATE_15000SPS};
	HAL_SPI_Transmit(&hspi1, drate_config, sizeof(drate_config), HAL_MAX_DELAY); // 设置采样速度

	// 配置 IO 寄存器 (如果有必要)
	uint8_t io_config[] = {ADS1256_CMD_WREG | ADS1256_IO, 0x00};
	HAL_SPI_Transmit(&hspi1, io_config, sizeof(io_config), HAL_MAX_DELAY); // 配置IO
	
	HAL_Delay(100);
//	vTaskDelay(pdMS_TO_TICKS(100));
	// 等待 DRDY 为低，表示采样数据已准备好
	while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) == GPIO_PIN_SET);

	// 执行自校准
	HAL_SPI_Transmit(&hspi1, (uint8_t[]){ADS1256_CMD_SELFCAL}, 1, HAL_MAX_DELAY);  // 自校准命令

	// 停止SPI通信
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // CS引脚拉高，停止SPI通信
	//标志位
	ADS1256_INIT_FLAG = 1;
}


//从ADS1256读取数据
unsigned int ADS1256ReadData(void)
{
    unsigned int sum = 0;           // 存储 32 位数据
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // 将GPIOB Pin12拉低，准备开始通信

    // 等待 ADS1256 数据准备好 (DRDY为低)
    while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) == GPIO_PIN_SET); 

    // 向ADS1256发送SYNC和WAKEUP命令
    uint8_t sync_cmd = ADS1256_CMD_SYNC;
    uint8_t wakeup_cmd = ADS1256_CMD_WAKEUP;
    HAL_SPI_Transmit(&hspi1, &sync_cmd, 1, HAL_MAX_DELAY);  // 发送SYNC命令
    HAL_SPI_Transmit(&hspi1, &wakeup_cmd, 1, HAL_MAX_DELAY);  // 发送WAKEUP命令
    
    // 等待 DRDY 变低，表示数据准备好
    while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) == GPIO_PIN_SET); 

    // 向 ADS1256 发送 RDATA 命令，开始读取数据
    uint8_t rdata_cmd = ADS1256_CMD_RDATA;
    HAL_SPI_Transmit(&hspi1, &rdata_cmd, 1, HAL_MAX_DELAY);  // 发送RDATA命令

    // 延时，确保数据稳定
		// 可以根据实际情况调整延时	
//  	vTaskDelay(pdMS_TO_TICKS(1));
		delay_us(1);
//		for(int i=0;i<10;i++);
//		vTaskDelay(pdMS_TO_TICKS(0));
		
		// SPI 全双工，读取 24 位数据
		uint8_t rx_data[3] = {0};  // 用于存储接收到的 3 个字节数据
		HAL_SPI_Receive(&hspi1, rx_data, 3, HAL_MAX_DELAY);  // 接收 24 位数据

		// 将接收到的数据拼接成 32 位整数
		sum = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

		// 设置 GPIOB Pin12 为高电平，结束通信
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// 设置 GPIOB 的 Pin 12 引脚为高电平，通常用于结束通信
		
		return sum;//读取到的32位数据放在这里
		
}



//向指定寄存器地址发送数据
void ADS1256WREG(unsigned char regaddr,unsigned char databyte)
{
    // 拉低 CS 片选引脚
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // 等待 ADS1256 的 DRDY 引脚为低，表示可以进行写操作
    while (HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port, ADS1256_DRDY_Pin) != GPIO_PIN_RESET);

    // 发送写寄存器命令，确保 regaddr 的低 4 位
    uint8_t cmd = ADS1256_CMD_WREG | (regaddr & 0x0F);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

    // 发送数据字节数-1（0x00 表示只有一个字节需要写入）
    uint8_t byte_count = 0x00;
    HAL_SPI_Transmit(&hspi1, &byte_count, 1, HAL_MAX_DELAY);

    // 发送实际的数据
    HAL_SPI_Transmit(&hspi1, &databyte, 1, HAL_MAX_DELAY);

    // 拉高 CS 片选引脚，停止 SPI 通信
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	
}



unsigned int ADS_sum(unsigned char channel)
{
	ADS1256WREG(ADS1256_MUX,channel);		//设置通道
	return ADS1256ReadData();//读取AD值，返回24位数据为读到的电压值（0-2.5V）
}



long double k=4.0;                 //电阻分压的比例系数0-10V -> 0-2.5V
long ulResult;                     //每一个通道的读取电压结果
long double ldVolutage[11];        //准备取相邻3次取平均值的寄存结果数组
uint8_t average_count = 1;         //打算求平均次数(这里的取平均次数可以到11次）原来的给的2
long double TureVolutage = 0;      //最终这个通道反馈的真实结果
// 初始化滤波器的输出（通常在第一次处理前需要设定）
float filteredVoltage = 0.0;
float alpha = 0.1;  // 调整alpha值，通常0 < alpha < 1，值越小平滑效果越强



long double GetDistance(int iq)    //iq为第几组通道
{
	
	int j = 0;
	int i = 0;
	uint8_t ADS1256_MUXP_AINx;
	if(iq == 0)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN0;
	}
	else if(iq == 1)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN1;
	}
	else if(iq == 2)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN2;
	}
	else if(iq == 3)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN3;
	}
	else if(iq == 4)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN4;
	}
	else if(iq == 5)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN5;
	}
	else if(iq == 6)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN6;
	}
	else if(iq == 7)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN7;
	}
	for(j = 0;j < average_count;j++ )
	{

		//ulResult = ADS_sum((i << 4) | ADS1256_MUXN_AINCOM);                           //00000001->00010000;00010000|00001000 = 00011000 = 0x18	，第二种形式，不建议用因为算起来很麻烦	
		ulResult = ADS_sum( ADS1256_MUXP_AINx | ADS1256_MUXN_AINCOM);//0-2.5V
    //如果ulResult最高位是1，下面的操作是取补码
		
		if( ulResult & 0x800000 )
		{
			ulResult = ~(unsigned long)ulResult;
			ulResult &= 0x7fffff;
			ulResult += 1;
			ulResult = -ulResult;
		}
		ldVolutage[j] = (long double)ulResult*0.59604644775390625*k;                 //用来拟合DT35监测到的距离，初步感觉是0-10m（即这里是将0-10V拟合到0-10m）
	}
  //FIR滤波器(这里有待修正，没有使用FIR滤波，不会搞延时处理。但是我观测去除FIR数据没啥问题）
	for(i = 0;i < average_count;i++)
	{
		TureVolutage = TureVolutage + ldVolutage[i];
	}
	TureVolutage = TureVolutage*0.000001/average_count;


//// 遍历输入数据，应用一阶低通滤波
//for(i = 0; i < average_count; i++)
//{
//  filteredVoltage = alpha * ldVolutage[i] + (1 - alpha) * filteredVoltage;
//}

//// 结果就是filteredVoltage，它已经是经过一阶低通滤波后的值
//TureVolutage = filteredVoltage;


	return TureVolutage;

}

/*************************************************************************
函 数 名：delay_us
函数功能：延时t us
*************************************************************************/
void delay_us(uint32_t time)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt = 0;
    uint32_t reload = SysTick -> LOAD;

    ticks = time*170;       //延时指定时间需要的节拍数
    told = SysTick -> VAL;  //刚进入时的计数器值

    while(1){
        tnow = SysTick -> VAL;
        if(tnow != told){
            //SYSTICK是一个递减的计数器
            if(tnow < told) tcnt += told - tnow;    //说明计数器没有递减到重装载
            else    tcnt += reload - tnow + told;   //说明计数器重装载了
            told = tnow;
            if(tcnt >= ticks)   break;              //计数值到达了指定的节拍数
        }
    }
}
