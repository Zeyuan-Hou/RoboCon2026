#include "rec_ball.h"

void Receive_ball(void)
{
	switch(Rec_Ball_State)
	{
		case J60_LIFT://J60升起到接球范围最大
			J60_target_pos = 80;
			if(fabs(J60_target_pos - J60_angle) < 5)//J60成功升起后，进入等待接球模式
			{
				Rec_Ball_State = REC_WAIT;
			}
			break;
		case REC_WAIT:
			if(switch_on_2 == 1)//球滚进运球框里了
				Rec_Ball_State = DRIB_READY;
			break;
		case DRIB_READY:
			J60_target_pos = 80;//58;//把J60放低，等待之后的运球
			if(fabs(J60_target_pos - J60_angle) < 5)
			{
				Rec_Ball_State = REC_END;
			}
			break;
		case REC_END:
			break;
		default:
			break;
	}
}

