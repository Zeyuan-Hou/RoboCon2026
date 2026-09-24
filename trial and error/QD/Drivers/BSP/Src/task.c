#include "main.h"
#include "PositionSwitch.h"
#include "ADS1256.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Valve.h"
#include "task.h"


void CAN_DATA(void)
{   
			DT35_TxData[0] = ket0 & 0xFF;          
			DT35_TxData[1] = (ket0 >> 8) & 0xFF;   

			DT35_TxData[2] = ket1 & 0xFF;          
			DT35_TxData[3] = (ket1 >> 8) & 0xFF;   

			DT35_TxData[4] = ket2 & 0xFF;          
			DT35_TxData[5] = (ket2 >> 8) & 0xFF;   

			DT35_TxData[6] = ket3 & 0xFF;
			DT35_TxData[7] = (ket3 >> 8) & 0xFF;

//			DT35_TxData[8] = ket4 & 0xFF;          
//			DT35_TxData[9] = (ket4 >> 8) & 0xFF;   

//			DT35_TxData[10] = ket5 & 0xFF;         
//			DT35_TxData[11] = (ket5 >> 8) & 0xFF;  

//			DT35_TxData[12] = ket6 & 0xFF;         
//			DT35_TxData[13] = (ket6 >> 8) & 0xFF;  

//			DT35_TxData[14] = ket7 & 0xFF;         
//			DT35_TxData[15] = (ket7 >> 8) & 0xFF;  
      CAN_SendStdData(&hfdcan1,0x220,KEY_TxData,1);//KEY

			CAN_SendStdData(&hfdcan1,0x210,DT35_TxData,8);//DT35	
			
//			CAN_SendStdData(&hfdcan2,0x200,KEY_TxData,1);//KEY

//			CAN_SendStdData(&hfdcan2,0x210,DT35_TxData,8);//DT35	

		
			if(ucKeyNum != 8)
			{
				Scan_key();
				ucKeyNum++;
			}
			else
			{
				ucKeyNum = 0;
			}

            
            //LED-500ms
			if (ucLedCnt >= 400)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				ucLedCnt = 0;
			}
			ucLedCnt++;

}

void GET_DISTANCE(void)
{
  {
			ket0 = CESHI0 = 1000 * GetDistance(0);

			ket1 = CESHI1 = 1000 * GetDistance(1);

			ket2 = CESHI2 = 1000 * GetDistance(2);

			ket3 = CESHI3 = 1000 * GetDistance(3);
		
//			ket4 = CESHI4 = 1000 * GetDistance(4);

//			ket5 = CESHI5 = 1000 * GetDistance(5);

//			ket6 = CESHI6 = 1000 * GetDistance(6);

//			ket7 = CESHI7 = 1000 * GetDistance(7);

  }
}

void VALVE_CONTROL(void)
{

		ValveCtrl();

}
