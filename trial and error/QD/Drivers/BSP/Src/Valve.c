#include "main.h"
#include "Valve.h"

void ValveCtrl(void)
{
	if((TempValveStat&VALVE1) == VALVE1)
	{
		VALVE_1_OPEN;
	}
	else
	{
		VALVE_1_CLOSE;
	}
	
	if((TempValveStat&VALVE2) == VALVE2)
	{
		VALVE_2_OPEN;
	}
	else
	{
		VALVE_2_CLOSE;
	}	
	
	if((TempValveStat&VALVE3) == VALVE3)
	{
		VALVE_3_OPEN;
	}
	else
	{
		VALVE_3_CLOSE;
	}	
	
	if((TempValveStat&VALVE4) == VALVE4)
	{
		VALVE_4_OPEN;
	}
	else
	{
		VALVE_4_CLOSE;
	}	
	
	if((TempValveStat&VALVE5) == VALVE5)
	{
		VALVE_5_OPEN;
	}
	else
	{
		VALVE_5_CLOSE;
	}	
	
	if((TempValveStat&VALVE6) == VALVE6)
	{
		VALVE_6_OPEN;
	}
	else
	{
		VALVE_6_CLOSE;
	}	
	

		
}

void ValveAllStart(void)
{
	VALVE_1_OPEN;
	VALVE_2_OPEN;
	VALVE_3_OPEN;
	VALVE_4_OPEN;
	VALVE_5_OPEN;
	VALVE_6_OPEN;


}
void ValveAllClose(void)
{
	VALVE_1_CLOSE;
	VALVE_2_CLOSE;
	VALVE_3_CLOSE;
	VALVE_4_CLOSE;
	VALVE_5_CLOSE;
	VALVE_6_CLOSE;

}
