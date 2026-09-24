#include "NRF24L01.h"
#include "spi.h"

/**************************************************************
	参数定义
***************************************************************/
ACK_PAYLOAD nRF24L01_ack_pay;

uint8_t nRF24L01_TxBuf[32]={0};
uint8_t nRF24L01_RxBuf[32]={0};
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x76,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x76,0x43,0x10,0x10,0x01}; //接收地址
  
/***********************************************************
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
************************************************************/
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFF)!=HAL_OK)
  {
    d_read=0xFF;
  }
  return d_read; 
}
 
/************************************************************
  * 函数功能: 检测24L01是否存在
  * 输入参数: 无
  * 返 回 值: 0：成功;1:失败
  * 说    明：无          
*************************************************************/ 
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
   
	NRF24L01_Write_Buf(NRF_WRITE_REG|TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;   //检测24L01错误	
	return 0;		 	//检测到24L01
}	
 
/************************************************************
  * 函数功能: SPI写寄存器
  * 输入参数: 寄存器reg，写入的值value
  * 返 回 值: 写入成功/不成功状态值
  * 说    明：reg:指定寄存器地址           
*************************************************************/ 
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
  uint8_t status;	
  NRF24L01_SPI_CS_ENABLE();                 //使能SPI传输
  status =SPIx_ReadWriteByte(&hspi2,reg);   //发送寄存器号 
  SPIx_ReadWriteByte(&hspi2,value);         //写入寄存器的值
  NRF24L01_SPI_CS_DISABLE();                //禁止SPI传输	   
  return(status);       			//返回状态值
}
 

/**********************************************************
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 寄存器reg
  * 返 回 值: 读取成功/不成功的状态值
  * 说    明：reg:要读的寄存器
**********************************************************/ 
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_SPI_CS_ENABLE();          //使能SPI传输		
  SPIx_ReadWriteByte(&hspi2,reg);   //发送寄存器号
  reg_val=SPIx_ReadWriteByte(&hspi2,0XFF);//读取寄存器内容
  NRF24L01_SPI_CS_DISABLE();          //禁止SPI传输		    
  return(reg_val);           //返回状态值
}		
 
/***********************************************************
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: reg,*pbuf,len
  * 返 回 值: 此次读到的状态寄存器值 
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
***********************************************************/ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  NRF24L01_SPI_CS_ENABLE();           //使能SPI传输
  status=SPIx_ReadWriteByte(&hspi2,reg);//发送寄存器值(位置),并读取状态值   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspi2,0XFF);//读出数据
  }
  NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
  return status;        //返回读到的状态值
}
 
/*************************************************************
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: reg,*pbuf,len
  * 返 回 值: 状态值
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *           
 **************************************************************/ 
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_SPI_CS_ENABLE();          //使能SPI传输
  status = SPIx_ReadWriteByte(&hspi2,reg);//发送寄存器值(位置),并读取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspi2,*pBuf++); //写入数据	 
  }
  NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
  return status;          //返回读到的状态值
}		
 
/****************************************************************
  * 函数功能: 启动NRF24L01发送一次数据
  * 输入参数: 发送的数据寄存器的位置
  * 返 回 值: 发送完成状况
  * 说    明：txbuf:待发送数据首地址
  * 单工发送         
*****************************************************************/ 
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta; 
	NRF24L01_CE_LOW();
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE_HIGH();//启动发送	 
  
	while(NRF24L01_IRQ_PIN_READ()!=0);//等待发送完成
  
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG|STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}

/****************************************************************
  * 函数功能: 处理NRF24L01成功发送并且接收到的应答ACK信号数据包
  * 输入参数: ack_pay数据包的地址
  * 返 回 值: 发送完成状况
  * 说    明：ACK_PAYLOAD *ack_pay 应答信号数据包格式
  *           
*****************************************************************/ 
uint8_t NRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay)
{
	uint8_t status;
	while(1)
	{
		status=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	 
		NRF24L01_Write_Reg(NRF_WRITE_REG|STATUS,status); //清除TX_DS或MAX_RT中断标志
	
		//如果启动ACK模式
#if(DYNPD_ACK_DATA) 
		if(status & RX_OK)		// 判断是否接收到数据
		{
			ack_pay->Ack_Len = NRF24L01_Read_Reg(R_RX_PL_WID);//接收到数据长度
			if(ack_pay->Ack_Len < 33)
			{
				NRF24L01_Read_Buf(RD_RX_PLOAD, ack_pay->Ack_Buf, ack_pay->Ack_Len);	//从RX_FIFO里面读取动态长度数据包
				ack_pay->Ack_Status = 1;
			}
			else 
			{
				ack_pay->Ack_Len = 0;
			}
			NRF24L01_Write_Reg(NRF_WRITE_REG | FLUSH_RX,0xff);	//清空缓冲区
			
			ack_pay->Ack_Channel = (status & 0x0e) >> 1;//从status中提取出channel的信息
		}
#endif
		if(status&MAX_TX)//达到最大重发次数
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
			return MAX_TX; 
		}
		if(status&TX_OK)//发送完成
		{
			return TX_OK;
		}
		return 0xff;//其他原因发送失败
		}
}

/****************************************************************
  * 函数功能: 启动NRF24L01发送一次动态长度数据
  * 输入参数: 发送的数据寄存器的位置，数据包的长度
  * 返 回 值: 发送完成状况
  * 说    明：txbuf:待发送数据首地址 len：动态数据包长度
  * 伪全双工发送   
*****************************************************************/ 
uint8_t NRF_TX_Packet(uint8_t *txbuf ,uint16_t len)
{
	NRF24L01_CE_LOW();
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,len);//写数据到TX BUF  32个字节
	NRF24L01_CE_HIGH();//启动发送
	HAL_Delay(20);
	while(NRF24L01_IRQ_PIN_READ()!=0);//等待发送完成
	
	return (NRF24L01_Tx_Ack(&nRF24L01_ack_pay));
	
}

/********************************************************************
  * 函数功能:启动NRF24L01接收一次数据
  * 输入参数: 接受的数据位置
  * 返 回 值: 接受完成情况
  * 说    明：rxbuf:接受到的数据首地址
  *           
*********************************************************************/ 
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t len=0;
	uint8_t sta;	
	//NRF24L01_CE_LOW();// 禁用接收模式（低电平），准备读取接收数据
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG|STATUS,sta); //清除TX_DS或MAX_RT中断标志
		
	if(sta&RX_OK)//接收到数据
	{
		len = NRF24L01_Read_Reg(R_RX_PL_WID);// 读取接收数据包的长度
		if(len<33)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,len);//读取数据
		}
		else
		{
			len = 0;// 数据包过长，设定长度为 0
		}
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器  
	}	   
	NRF24L01_CE_HIGH();//置高 CE 引脚，激活接收模式，准备接收新的数据包
	return len;//收到数据长度
}			


/********************************************************************
  * 函数功能:	向 nRF24L01 写入带数据的 ACK 负载 (ACK Payload)，
							并且发送带数据的 ACK 响应。
  * 输入参数:	ack_pay - 包含 ACK 数据结构体
  * 返 回 值: 无返回值
  * 说    明：该函数用于动态数据长度模式下，发送一个带数据的 ACK 响应。
  *********************************************************************/
#if(DYNPD_ACK_DATA) //带数据的ACK功能
void NRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay)
{
	NRF24L01_CE_LOW();// 禁用接收模式（低电平），准备读取接收数据
  // 清除TX FIFO寄存器，准备发送新的 ACK 数据
  NRF24L01_Write_Reg(NRF_WRITE_REG | FLUSH_TX, 0xff); 

  // 向 nRF24L01 写入带数据的 ACK 负载
  NRF24L01_Write_Buf(NRF_WRITE_REG | W_ACK_PAYLOAD | ack_pay.Ack_Channel, ack_pay.Ack_Buf, ack_pay.Ack_Len);
	NRF24L01_CE_HIGH();//置高 CE 引脚，激活接收模式，准备接收新的数据包
}
#endif

/**********************************************************************
  * 函数功能: 该函数初始化NRF24L01到RX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
***********************************************************************/ 
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();	  
  NRF24L01_Write_Reg(NRF_WRITE_REG|CONFIG, 			0x0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(NRF_WRITE_REG|EN_AA,				0x01);//使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF_WRITE_REG|EN_RXADDR,		0x01);//使能通道0的接收地址  	 
  NRF24L01_Write_Reg(NRF_WRITE_REG|RF_CH,				85);	//设置RF通信频率		  
  NRF24L01_Write_Reg(NRF_WRITE_REG|RF_SETUP,		0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
//	NRF24L01_Write_Reg(NRF_WRITE_REG|FLUSH_TX, 		0xff);	//清除TX FIFO寄存器.发射模式下用
//	NRF24L01_Write_Reg(NRF_WRITE_REG|FLUSH_RX, 		0xff);	//清除RX FIFO寄存器.接收模式下用,在传输应答信号过程中不应执行此指令
	NRF24L01_Write_Reg(NRF_WRITE_REG|DYNPD, 			0x01);	//使能通道0的动态载荷长度
	NRF24L01_Write_Reg(NRF_WRITE_REG|FEATURE,  		0x06);	//使能ACK带载,使能ACK的动态载荷长度	
  NRF24L01_Write_Reg(NRF_WRITE_REG|RX_PW_P0,		RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	       
  NRF24L01_Write_Buf(NRF_WRITE_REG|RX_ADDR_P0,	(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	
  NRF24L01_CE_HIGH(); //CE为高,进入接收模式 
  HAL_Delay(1);
}	
 
/*********************************************************************
  * 函数功能: 该函数初始化NRF24L01到TX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
**********************************************************************/ 
void NRF24L01_TX_Mode(void)
{														 
  NRF24L01_CE_LOW();	    
  NRF24L01_Write_Buf(NRF_WRITE_REG|TX_ADDR,		(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(NRF_WRITE_REG|RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//设置TX节点地址,主要为了使能ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG|EN_AA,			0x01);  //使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF_WRITE_REG|EN_RXADDR,	0x01); 	//使能通道0的接收地址  
  NRF24L01_Write_Reg(NRF_WRITE_REG|SETUP_RETR,0xff);	//设置自动重发间隔时间:4000us + 86us;最大自动重发次数:15次
  NRF24L01_Write_Reg(NRF_WRITE_REG|RF_CH,			40);    //设置RF通道为40
  NRF24L01_Write_Reg(NRF_WRITE_REG|RF_SETUP,	0x0f); 	//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(NRF_WRITE_REG|CONFIG,		0x0e);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
//  NRF24L01_Write_Reg(NRF_WRITE_REG|FLUSH_TX, 	0xff);	//清除TX FIFO寄存器.发射模式下用
//	NRF24L01_Write_Reg(NRF_WRITE_REG|FLUSH_RX, 	0xff);	//清除RX FIFO寄存器.接收模式下用,在传输应答信号过程中不应执行此指令
	NRF24L01_Write_Reg(NRF_WRITE_REG|DYNPD, 		0x01);	//使能通道0的动态载荷长度
	NRF24L01_Write_Reg(NRF_WRITE_REG|FEATURE,  	0x06);	//使能ACK带载,使能ACK的动态载荷长度
	NRF24L01_CE_HIGH();//CE为高,10us后启动发送
  HAL_Delay(1);
}
 
///**********************************************************************
//  * 函数功能: 切换NRF24L01的工作模式
//  * 输入参数: re_mode
//  * 返 回 值: 无
//	* 说    明：re_mode=1:接收模式;re_mode=0:接收模式
//  *           
//***********************************************************************/
//void NRF24L01_Set_Mode(uint8_t re_mode)
//{
//	uint8_t mode = NRF24L01_Read_Reg(CONFIG);
//	if(re_mode)//接受模式
//	{
//		NRF24L01_Write_Reg(NRF_WRITE_REG|CONFIG,mode | 0x01);
//	}
//	else//发送模式
//	{
//		NRF24L01_Write_Reg(NRF_WRITE_REG|CONFIG,mode & 0xfe);
//	}
//}



/**********************************************************************
  * 函数功能: 该函数NRF24L01进入低功耗模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
***********************************************************************/
void NRF_LowPower_Mode(void)
{
	NRF24L01_CE_LOW();	 
	NRF24L01_Write_Reg(NRF_WRITE_REG|CONFIG, 0x00);		//配置工作模式:掉电模式
}


