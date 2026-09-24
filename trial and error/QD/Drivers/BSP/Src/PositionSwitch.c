#include "main.h"
#include "fdcan.h"
#include "PositionSwitch.h"

//行程开关部分
void Scan_key(void)
{
	if( aucDisShakeFlag[ucKeyNum] == 0 )				                  //正常扫描状态下
		{
			if( aucKeyScanCnt[ucKeyNum] >= KEYSCAN_TIME )	          	//到了一次扫描的时间1ms
			{  //下面判断语句是将 usKeyStat 右移 ucKeyNum 位，这样可以把指定按键的状态移动到最低位。再与1进行按位与，得到此开关状态
				if( ((usKeyStat>>ucKeyNum)&1)!= ScanKey((ucKeyNum)) )	 	//如果原来状态和新状态不一样
				{
					aucKeyStatTemp[ucKeyNum] = ScanKey(ucKeyNum) ;			  //临时存对应按键值
					aucDisShakeFlag[ucKeyNum] = 1;				   			        //消抖标志位设为1
				}
				else								 					                        	//如果原来状态和新状态一致
				{															                        //什么都不做
				}
				aucKeyScanCnt[ucKeyNum] = 0;	   					              //计数值归零，重新开始计数，等待下一次扫描
			}
			aucKeyScanCnt[ucKeyNum] ++;			   				                //按键检测扫描时间计数								
		}
	else												                                  //消抖状态下
		{
	   	if(aucDisShakeCnt[ucKeyNum] >= DISSHAKE_TIME)	   	        //到了消抖时间 ?10ms
			{
				if( aucKeyStatTemp[ucKeyNum] == ScanKey(ucKeyNum) )		  //上一状态和新状态一致
				{
					usKeyStat ^= (1<<ucKeyNum) ;				   		            //对应位的状态翻转（按位异或，这里产生了按位反转的效果）
					
					KEY_TxData[0] = usKeyStat;
//					CAN_SendStdData(&hfdcan1,0x20,KEY_TxData,1);			        //经消抖处理后检测到不同值，通过CAN发送
//					vTaskDelay(pdMS_TO_TICKS(1));
				}
				else	   												                        //上一状态和新状态不一致，表示为抖动
				{		   												                          //什么都不做
				}
				aucDisShakeFlag[ucKeyNum] = 0;						              //标志位置0，重新开始正常扫描
				aucDisShakeCnt[ucKeyNum] = 0;						                //计数值归零，重新开始计数，等待下一次消抖						
			}
			aucDisShakeCnt[ucKeyNum] ++;			   			                //消抖时间计数			
		}
	


    // 分解每个bit位到数组
    for (uint8_t i = 0; i < 8; i++) 
	  {
        key_states[i] = (usKeyStat >> i) & 0x01;
    }		
}


//读取此时刻的行程开关的状态，高电平返回1，低电平返回0
uint8_t ScanKey(uint8_t KeyNum)
{
	uint8_t Stat;
	switch(KeyNum)
	{
	 	case 0:
			Stat = GET_KEY1;
		break;
	 	case 1:
			Stat = GET_KEY2;
		break;
	 	case 2:
			Stat = GET_KEY3;
		break;
	 	case 3:
			Stat = GET_KEY4;
		break;
	 	case 4:
			Stat = GET_KEY5;
		break;
	 	case 5:
			Stat = GET_KEY6;
		break;
	 	case 6:
			Stat = GET_KEY7;
		break;
	 	case 7:
			Stat = GET_KEY8;
		break;

		default:
		break;	
	}
	return Stat;
}

void TravelSwitchDataDeal(const uint16_t travel_switch, uint8_t* key_states) 
{
    // 分解每个bit位到数组
    for (uint8_t i = 0; i < 16; i++) 
	{
        key_states[i] = (travel_switch >> i) & 0x01;
    }
}
