#include "vision_task.h"

void Vision_task(void)
{
	if(Vision_Task_State != LOCKED)
	{
		nav.nav_state = NAV_VISION_PATH;
	
	
	}
}
