#include "chassis.h"

CHASSIS_STATUS *CS_curstatus=NULL;


void CS_Change(CHASSIS_STATUS *newstatus)
{
    if(CS_curstatus==newstatus) return;
    if(CS_curstatus!=NULL && CS_curstatus->Exit != NULL) CS_curstatus->Exit();
    CS_curstatus = newstatus;
    if(newstatus!=NULL && newstatus->Enter != NULL) CS_curstatus->Enter();
}

void CS_Run(void)
{
    if(CS_curstatus!=NULL && CS_curstatus->Execute != NULL) CS_curstatus->Execute();
}

chassis_run_des steer_prevel;

void CS_highspeed_Enter(void){//更快
//    chassis_run.leftdown.fpKp = 100.0f;
//    chassis_run.leftdown.fpKd = 0.f;
//    chassis_run.leftdown.fpKi = 0.f;
//    chassis_run.leftup.fpKp = 100.0f;
//    chassis_run.leftup.fpKd = 0.f;
//    chassis_run.leftup.fpKi = 0.f;
//    chassis_run.rightdown.fpKp = 100.f;
//    chassis_run.rightdown.fpKd = 0.f;
//    chassis_run.rightdown.fpKi = 0.f;
//    chassis_run.rightup.fpKp = 100.0f;
//    chassis_run.rightup.fpKd = 0.f;
//    chassis_run.rightup.fpKi = 0.f;
//	
//	steer_prevel.leftdown=leftdown_motor.anglev;
//	steer_prevel.rightdown=rightdown_motor.anglev;
//	steer_prevel.rightup=rightup_motor.anglev;
//	steer_prevel.leftup=leftup_motor.anglev;
//	
//	fric_k_1.leftdown=25;
//	fric_k_1.leftup=25;
//	fric_k_1.rightdown=25;
//	fric_k_1.rightup=25;
//	
//	fric_k_2.leftdown=35000;
//	fric_k_2.leftup=35000;

//	fric_k_2.rightup=35000;
//	fric_k_2.rightdown =35000;
    chassis_run.leftdown.fpKp = 60.0f;
    chassis_run.leftdown.fpKd = 0.f;
    chassis_run.leftdown.fpKi = 0.f;
    chassis_run.leftup.fpKp = 60.0f;
    chassis_run.leftup.fpKd = 0.f;
    chassis_run.leftup.fpKi = 0.f;
    chassis_run.rightdown.fpKp = 60.f;
    chassis_run.rightdown.fpKd = 0.f;
    chassis_run.rightdown.fpKi = 0.f;
    chassis_run.rightup.fpKp = 60.0f;
    chassis_run.rightup.fpKd = 0.f;
    chassis_run.rightup.fpKi = 0.f;
	
	steer_prevel.leftdown=leftdown_motor.anglev;
	steer_prevel.rightdown=rightdown_motor.anglev;
	steer_prevel.rightup=rightup_motor.anglev;
	steer_prevel.leftup=leftup_motor.anglev;
	
	//wheel_encoder_velt_filter.leftdown_velt.preout=leftdown_motor.anglev;
	//wheel_encoder_velt_filter.leftup_velt.preout=leftup_motor.anglev;
	//wheel_encoder_velt_filter.rightdown_velt.preout=rightdown_motor.anglev;
	//wheel_encoder_velt_filter.rightup_velt.preout=rightup_motor.anglev;
	
	fric_k_1.leftdown=15;
	fric_k_1.leftup=15;
	fric_k_1.rightdown=15;
	fric_k_1.rightup=15;
	
	fric_k_2.leftdown=20000;
	fric_k_2.leftup=20000;
	fric_k_2.rightup=20000;
	fric_k_2.rightdown =20000;


    
}


float CS_pos_velmax=2500;
float CS_pos_wmax=300;

void CS_highspeed_Execute(void){
        float fpQ;
        if (nav.nav_state == RC_LOCAL ||nav.nav_state==SEMIAUTO_UP_DOWN_STAIRS)
        {   // 以车身坐标系操控
            fpQ = PI/2;
        }
	    	else{
            // 以全场坐标系操控
            fpQ =stRobot.stPos.fpPosQ * RADIAN_10;
					while(fpQ>=PI) fpQ-=PI2;
					while(fpQ<-PI) fpQ+=PI2;
        }
        Convert_velt(&nav.expect_robot_global_velt, &expect_robot_local_Velt, fpQ); // 把local->fpW = global->fpW注释了
    
        // 角速度死区
        if (fabs(expect_robot_local_Velt.fpW) < 0.1f) // rad/s
            expect_robot_local_Velt.fpW = 0.f;

					float ratio=1.f;
					float vel=sqrtf(powf(expect_robot_local_Velt.fpX,2)+powf(expect_robot_local_Velt.fpY,2));
					if(vel >=3000.f) ratio= 3000.f/vel;
					else ratio =fmax(powf(vel/CS_pos_velmax, 2.f),0.7f);
					expect_robot_local_Velt.fpX *=ratio;
					expect_robot_local_Velt.fpY *=ratio;
				  
	    	
        // 左右侧电机转向正方向相反！！
        steer_velt.leftup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.rightdown = powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.leftdown = -powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    
        steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
        steer_pos.leftdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
        steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
        steer_pos.rightdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    
        // 速度过小时，arctan分式计算误差大，无法准确反映方向，保持上次位置
        if (fabsf(steer_velt.leftup) < 5.f) steer_pos.leftup = steer_pos_pre.leftup;
        if (fabsf(steer_velt.leftdown) < 5.f) steer_pos.leftdown = steer_pos_pre.leftdown;
        if (fabsf(steer_velt.rightup) < 5.f) steer_pos.rightup = steer_pos_pre.rightup;
        if (fabsf(steer_velt.rightdown) < 5.f) steer_pos.rightdown = steer_pos_pre.rightdown;
    
        swerve_optimize(steer_pos_pre.rightdown, &steer_pos.rightdown, &steer_velt.rightdown);
        swerve_optimize(steer_pos_pre.rightup, &steer_pos.rightup, &steer_velt.rightup);
        swerve_optimize(steer_pos_pre.leftdown, &steer_pos.leftdown, &steer_velt.leftdown);
        swerve_optimize(steer_pos_pre.leftup, &steer_pos.leftup, &steer_velt.leftup);
    
        steer_pos_pre.leftup = steer_pos.leftup;
        steer_pos_pre.leftdown = steer_pos.leftdown;
        steer_pos_pre.rightup = steer_pos.rightup;
        steer_pos_pre.rightdown = steer_pos.rightdown;
    
        // +机械零点
        leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO + leftup_init_angle;
        leftdown_turn_motor.Input = steer_pos.leftdown * TURN_GEAR_RATIO + leftdown_init_angle;
        rightdown_turn_motor.Input = steer_pos.rightdown * TURN_GEAR_RATIO + rightdown_init_angle;
        rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO + rightup_init_angle ;
}



void CS_lowspeed_Enter(void){//更精确
    chassis_run.leftdown.fpKp = 60.0f;
    chassis_run.leftdown.fpKd = 0.f;
    chassis_run.leftdown.fpKi = 0.f;
    chassis_run.leftup.fpKp = 60.0f;
    chassis_run.leftup.fpKd = 0.f;
    chassis_run.leftup.fpKi = 0.f;
    chassis_run.rightdown.fpKp = 60.f;
    chassis_run.rightdown.fpKd = 0.f;
    chassis_run.rightdown.fpKi = 0.f;
    chassis_run.rightup.fpKp = 60.0f;
    chassis_run.rightup.fpKd = 0.f;
    chassis_run.rightup.fpKi = 0.f;
	
	steer_prevel.leftdown=leftdown_motor.anglev;
	steer_prevel.rightdown=rightdown_motor.anglev;
	steer_prevel.rightup=rightup_motor.anglev;
	steer_prevel.leftup=leftup_motor.anglev;
	
	//wheel_encoder_velt_filter.leftdown_velt.preout=leftdown_motor.anglev;
	//wheel_encoder_velt_filter.leftup_velt.preout=leftup_motor.anglev;
	//wheel_encoder_velt_filter.rightdown_velt.preout=rightdown_motor.anglev;
	//wheel_encoder_velt_filter.rightup_velt.preout=rightup_motor.anglev;
	
	fric_k_1.leftdown=15;
	fric_k_1.leftup=15;
	fric_k_1.rightdown=15;
	fric_k_1.rightup=15;
	
	fric_k_2.leftdown=20000;
	fric_k_2.leftup=20000;
	fric_k_2.rightup=20000;
	fric_k_2.rightdown =20000;
}


void CS_lowspeed_Execute(void){
        float fpQ;
        if (nav.nav_state == RC_LOCAL ||nav.nav_state==SEMIAUTO_UP_DOWN_STAIRS)
        {   // 以车身坐标系操控
            fpQ = PI/2;
        }
	    	else{
            // 以全场坐标系操控
            fpQ =stRobot.stPos.fpPosQ * RADIAN_10;
					while(fpQ>=PI) fpQ-=PI2;
					while(fpQ<-PI) fpQ+=PI2;
        }
        Convert_velt(&nav.expect_robot_global_velt, &expect_robot_local_Velt, fpQ); // 把local->fpW = global->fpW注释了
    
        // 角速度死区
        if (fabs(expect_robot_local_Velt.fpW) < 0.1f) // rad/s
            expect_robot_local_Velt.fpW = 0.f;
					float ratio=1.f;
					float vel=sqrtf(powf(expect_robot_local_Velt.fpX,2)+powf(expect_robot_local_Velt.fpY,2));
					if(vel >=CS_pos_velmax) ratio= CS_pos_velmax/vel;
					else ratio = fmax(powf(vel/CS_pos_velmax, 2.f),0.5f);
					expect_robot_local_Velt.fpX *=ratio;
					expect_robot_local_Velt.fpY *=ratio;
				  
	    	
        // 左右侧电机转向正方向相反！！
        steer_velt.leftup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.rightdown = powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
        steer_velt.leftdown = -powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    
        steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
        steer_pos.leftdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
        steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
        steer_pos.rightdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    
        // 速度过小时，arctan分式计算误差大，无法准确反映方向，保持上次位置
        if (fabsf(steer_velt.leftup) < 5.f) steer_pos.leftup = steer_pos_pre.leftup;
        if (fabsf(steer_velt.leftdown) < 5.f) steer_pos.leftdown = steer_pos_pre.leftdown;
        if (fabsf(steer_velt.rightup) < 5.f) steer_pos.rightup = steer_pos_pre.rightup;
        if (fabsf(steer_velt.rightdown) < 5.f) steer_pos.rightdown = steer_pos_pre.rightdown;
    
        swerve_optimize(steer_pos_pre.rightdown, &steer_pos.rightdown, &steer_velt.rightdown);
        swerve_optimize(steer_pos_pre.rightup, &steer_pos.rightup, &steer_velt.rightup);
        swerve_optimize(steer_pos_pre.leftdown, &steer_pos.leftdown, &steer_velt.leftdown);
        swerve_optimize(steer_pos_pre.leftup, &steer_pos.leftup, &steer_velt.leftup);
    
        steer_pos_pre.leftup = steer_pos.leftup;
        steer_pos_pre.leftdown = steer_pos.leftdown;
        steer_pos_pre.rightup = steer_pos.rightup;
        steer_pos_pre.rightdown = steer_pos.rightdown;
    
        // +机械零点
        leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO + leftup_init_angle;
        leftdown_turn_motor.Input = steer_pos.leftdown * TURN_GEAR_RATIO + leftdown_init_angle;
        rightdown_turn_motor.Input = steer_pos.rightdown * TURN_GEAR_RATIO + rightdown_init_angle;
        rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO + rightup_init_angle ;
}

uint32_t CS_position_pretime=0;


void CS_position_Enter(void){
    chassis_run.leftdown.fpKp = 60.0f;
    chassis_run.leftdown.fpKd = 0.f;
    chassis_run.leftdown.fpKi = 0.f;
    chassis_run.leftup.fpKp = 60.0f;
    chassis_run.leftup.fpKd = 0.f;
    chassis_run.leftup.fpKi = 0.f;
    chassis_run.rightdown.fpKp = 60.f;
    chassis_run.rightdown.fpKd = 0.f;
    chassis_run.rightdown.fpKi = 0.f;
    chassis_run.rightup.fpKp = 60.0f;
    chassis_run.rightup.fpKd = 0.f;
    chassis_run.rightup.fpKi = 0.f;
	
	steer_prevel.leftdown=leftdown_motor.anglev;
	steer_prevel.rightdown=rightdown_motor.anglev;
	steer_prevel.rightup=rightup_motor.anglev;
	steer_prevel.leftup=leftup_motor.anglev;
	
	//wheel_encoder_velt_filter.leftdown_velt.preout=leftdown_motor.anglev;
	//wheel_encoder_velt_filter.leftup_velt.preout=leftup_motor.anglev;
	//wheel_encoder_velt_filter.rightdown_velt.preout=rightdown_motor.anglev;
	//wheel_encoder_velt_filter.rightup_velt.preout=rightup_motor.anglev;
	
	fric_k_1.leftdown=15;
	fric_k_1.leftup=15;
	fric_k_1.rightdown=15;
	fric_k_1.rightup=15;
	
	fric_k_2.leftdown=15000;
	fric_k_2.leftup=15000;
	fric_k_2.rightup=15000;
	fric_k_2.rightdown =15000;
	
}

void CS_position_Execute(void){
	nav.expect_robot_global_velt.fpY=1000;
	SpeedDistribute_Four_SteeringWheel(&nav);
	steer_velt.leftup = 0;
	steer_velt.rightup = 0;
	steer_velt.rightdown = 0;
	steer_velt.leftdown =0;

}

CHASSIS_STATUS CS_lowspeed={CS_lowspeed_Enter,CS_lowspeed_Execute,NULL};
CHASSIS_STATUS CS_highspeed={CS_highspeed_Enter,CS_highspeed_Execute,NULL};
CHASSIS_STATUS CS_pos={CS_position_Enter,CS_position_Execute,NULL};

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav)
{
    float fpQ;
    if (p_nav->nav_state == RC_LOCAL ||p_nav->nav_state==SEMIAUTO_UP_DOWN_STAIRS)
    {   // 以车身坐标系操控
        fpQ = PI/2;
    }
		else{
        // 以全场坐标系操控
            fpQ =stRobot.stPos.fpPosQ * RADIAN_10;
					while(fpQ>=PI) fpQ-=PI2;
					while(fpQ<-PI) fpQ+=PI2;
    }
    Convert_velt(&p_nav->expect_robot_global_velt, &expect_robot_local_Velt, fpQ); // 把local->fpW = global->fpW注释了

    // 角速度死区
    if (fabs(expect_robot_local_Velt.fpW) < 0.1f) // rad/s
        expect_robot_local_Velt.fpW = 0.f;
		
		

    // 左右侧电机转向正方向相反！！
    steer_velt.leftup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightdown = powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.leftdown = -powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;

    steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.leftdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.rightdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    // steer_pos.leftup = normalize_angle(steer_pos.leftup);
    // steer_pos.leftdown = normalize_angle(steer_pos.leftdown);
    // steer_pos.rightup = normalize_angle(steer_pos.rightup);
    // steer_pos.rightdown = normalize_angle(steer_pos.rightdown);

    // 速度过小时，arctan分式计算误差大，无法准确反映方向，保持上次位置
    if (fabsf(steer_velt.leftup) < 5.f) steer_pos.leftup = steer_pos_pre.leftup;
    if (fabsf(steer_velt.leftdown) < 5.f) steer_pos.leftdown = steer_pos_pre.leftdown;
    if (fabsf(steer_velt.rightup) < 5.f) steer_pos.rightup = steer_pos_pre.rightup;
    if (fabsf(steer_velt.rightdown) < 5.f) steer_pos.rightdown = steer_pos_pre.rightdown;

    // 老代码的舵轮转向处理有问题，会让转角转到180°，实际转角至多为90°
    // // 处理转向跳变问题
    // if (fabsf(steer_pos.rightdown - steer_pos_pre.rightdown) > 90.f){
    //     steer_pos.rightdown = normalize_angle(steer_pos.rightdown + 180.f);
    //     steer_velt.rightdown = -steer_velt.rightdown;
    // }
    // if (fabsf(steer_pos.rightup - steer_pos_pre.rightup) > 90.f){
    //     steer_pos.rightup = normalize_angle(steer_pos.rightup + 180.f);
    //     steer_velt.rightup = -steer_velt.rightup;
    // }
    // if (fabsf(steer_pos.leftdown - steer_pos_pre.leftdown) > 90.f){
    //     steer_pos.leftdown = normalize_angle(steer_pos.leftdown + 180.f);
    //     steer_velt.leftdown = -steer_velt.leftdown;
    // }
    // if (fabsf(steer_pos.leftup - steer_pos_pre.leftup) > 90.f){
    //     steer_pos.leftup = normalize_angle(steer_pos.leftup + 180.f);
    //     steer_velt.leftup = -steer_velt.leftup;
    // }

    swerve_optimize(steer_pos_pre.rightdown, &steer_pos.rightdown, &steer_velt.rightdown);
    swerve_optimize(steer_pos_pre.rightup, &steer_pos.rightup, &steer_velt.rightup);
    swerve_optimize(steer_pos_pre.leftdown, &steer_pos.leftdown, &steer_velt.leftdown);
    swerve_optimize(steer_pos_pre.leftup, &steer_pos.leftup, &steer_velt.leftup);

    steer_pos_pre.leftup = steer_pos.leftup;
    steer_pos_pre.leftdown = steer_pos.leftdown;
    steer_pos_pre.rightup = steer_pos.rightup;
    steer_pos_pre.rightdown = steer_pos.rightdown;

    // +机械零点
    leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO + leftup_init_angle;
    leftdown_turn_motor.Input = steer_pos.leftdown * TURN_GEAR_RATIO + leftdown_init_angle;
    rightdown_turn_motor.Input = steer_pos.rightdown * TURN_GEAR_RATIO + rightdown_init_angle;
    rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO + rightup_init_angle ;
}

void Drive_Chassis(){
	  chassis_run.leftup.fpDes = steer_velt.leftup;
    chassis_run.leftdown.fpDes = steer_velt.leftdown;
    chassis_run.rightdown.fpDes = steer_velt.rightdown;
    chassis_run.rightup.fpDes = steer_velt.rightup;
    if (nav.nav_state != CHASSIS_OFF)
    {
        PID_Calc(&chassis_run.rightdown, chassis_run.rightdown.fpDes, chassis_run.rightdown.fpFB);
        PID_Calc(&chassis_run.rightup, chassis_run.rightup.fpDes, chassis_run.rightup.fpFB);
        PID_Calc(&chassis_run.leftdown, chassis_run.leftdown.fpDes, chassis_run.leftdown.fpFB);
        PID_Calc(&chassis_run.leftup, chassis_run.leftup.fpDes, chassis_run.leftup.fpFB);

        DJI_ControlLoop(&rightdown_turn_motor);
        DJI_ControlLoop(&rightup_turn_motor);
        DJI_ControlLoop(&leftdown_turn_motor);
        DJI_ControlLoop(&leftup_turn_motor);
    }

    // 提高启动时的加速度，缩短小脚上台阶的加速距离
    if (chassis_run.feed_forward_state == WITH_FORWARD){
        friction_compensation();
        cur_steer_velt.leftdown = ClipFloat(chassis_run.leftdown.fpU + friction_feedforward.leftdown, -15000.f, 15000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run.leftup.fpU + friction_feedforward.leftup, -15000.f, 15000.f);
        cur_steer_velt.rightdown = ClipFloat(chassis_run.rightdown.fpU + friction_feedforward.rightdown, -15000.f, 15000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run.rightup.fpU + friction_feedforward.rightup, -15000.f, 15000.f);
    }else{
        cur_steer_velt.leftdown = ClipFloat(chassis_run.leftdown.fpU, -15000.f, 15000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run.leftup.fpU, -15000.f, 15000.f);
        cur_steer_velt.rightdown = ClipFloat(chassis_run.rightdown.fpU, -15000.f, 15000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run.rightup.fpU, -15000.f, 15000.f);
    }
		
   CAN_SendCurrent(&hcan1, 0X200, leftdown_turn_motor.motor_current, cur_steer_velt.leftdown, rightdown_turn_motor.motor_current, cur_steer_velt.rightdown);
   CAN_SendCurrent(&hcan2, 0x200, leftup_turn_motor.motor_current, cur_steer_velt.leftup, rightup_turn_motor.motor_current, cur_steer_velt.rightup);
}



void friction_compensation(void){
	float a_leftup=steer_velt.leftup-steer_prevel.leftup;
	float a_leftdown=steer_velt.leftdown-steer_prevel.leftdown;
	float a_rightup=steer_velt.rightup-steer_prevel.rightup;
	float a_rightdown=steer_velt.rightdown-steer_prevel.rightdown;
    friction_feedforward.leftup = ClipFloat(fric_k_1.leftup * steer_velt.leftup+fric_k_2.leftup*a_leftup, -6000.f, 6000.f); 
    friction_feedforward.leftdown = ClipFloat(fric_k_1.leftdown * steer_velt.leftdown+fric_k_2.leftdown*a_leftdown, -6000.f, 6000.f); 
    friction_feedforward.rightup = ClipFloat(fric_k_1.rightup * steer_velt.rightup+fric_k_2.rightup*a_rightup, -6000.f, 6000.f); 
    friction_feedforward.rightdown = ClipFloat(fric_k_1.rightdown * steer_velt.rightdown+fric_k_2.rightdown*a_rightdown, -6000.f, 6000.f);
	steer_prevel.leftup=steer_velt.leftup;
	steer_prevel.leftdown=steer_velt.leftdown;
	steer_prevel.rightup=steer_velt.rightup;
	steer_prevel.rightdown=steer_velt.rightdown;
}

// 返回相对ref最近的target等效角（可能超出[-180,180]，用于连续命令）
static float angle_nearest_to(float target_deg, float ref_deg)
{
    float delta = fmodf(target_deg - ref_deg, 360.0f);
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return ref_deg + delta;
}

// 舵轮转向优化，保证每次转向电机的转角至多为90°
static void swerve_optimize(float prev_deg, float *target_deg, float *wheel_vel)
{
    float cand = angle_nearest_to(*target_deg, prev_deg);
    float delta = cand - prev_deg;
    if (fabsf(delta) > 90.0f){
        cand = angle_nearest_to(*target_deg + 180.0f, prev_deg);
        *wheel_vel = -*wheel_vel;
    }
    *target_deg = cand; // 可能超出[-180,180]，但保证与prev的数值差≤90°
}
