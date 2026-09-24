#include "fun_task.h"

void Fun_task(void)
{
	switch(Task_Choice)
	{
		case CHOOSE_EMPTY_MODE:
			break;
		case CHOOSE_RESET_MODE:
			Manual_Task_State = LOCKED;
			Vision_Task_State = LOCKED;
			ShootChal_Task_State = LOCKED;
			DribChal_Task_State = LOCKED;
			break;
		case CHOOSE_MANUAL_MODE:
			Reset_Task_State = LOCKED;
			Vision_Task_State = LOCKED;
			ShootChal_Task_State = LOCKED;
			DribChal_Task_State = LOCKED;
			break;
		case CHOOSE_VISION_MODE:
			Reset_Task_State = LOCKED;
			Manual_Task_State = LOCKED;
			ShootChal_Task_State = LOCKED;
			DribChal_Task_State = LOCKED;
			break;
		case CHOOSE_SHOOTCHAL_MODE:
			Reset_Task_State = LOCKED;
			Manual_Task_State = LOCKED;
			Vision_Task_State = LOCKED;
			DribChal_Task_State = LOCKED;
			break;
		case CHOOSE_DRIBCHAL_MODE:
			Reset_Task_State = LOCKED;
			Manual_Task_State = LOCKED;
			Vision_Task_State = LOCKED;
			ShootChal_Task_State = LOCKED;
			break;
	}
	Reset_task();
	Manual_task();
	Vision_task();
	Dribble_challenge();
	Shoot_challenge();
}


