#include "Chassis_Motor_Ctrl.h"


void Chassis_Motor_Ctrl(void){
//进行航模电机自身的pid运算    
//		if(nav.nav_state!=NAV_OFF)
//		{
//			
//            if(uphilling==1)
//			{
////            // 左上电机
////            chassis_run.leftup.fpKp = 210.f;
////            chassis_run.leftup.fpKi = 0.7f;
////            chassis_run.leftup.fpKd = 0.1f;
////            chassis_run.leftup.fpSumEMax = 6000.f;
////            chassis_run.leftup.fpEMin = 0.f;

////            // 右上电机
////            chassis_run.rightup.fpKp = 180.f;
////            chassis_run.rightup.fpKi = 0.45f;
////            chassis_run.rightup.fpKd = 0.15f;
////            chassis_run.rightup.fpSumEMax = 4000.f;
////            chassis_run.rightup.fpEMin = 0.f;

////            // 左下电机
////            chassis_run.leftdown.fpKp = 210.f;
////            chassis_run.leftdown.fpKi = 0.7f;
////            chassis_run.leftdown.fpKd = 0.15f;
////            chassis_run.leftdown.fpSumEMax = 6000.f;
////            chassis_run.leftdown.fpEMin = 2.0f;

////            // 右下电机（爬坡专用）
////            chassis_run.rightdown.fpKp = 200.f;
////            chassis_run.rightdown.fpKi = 0.45f;
////            chassis_run.rightdown.fpKd = 0.45f;
////            chassis_run.rightdown.fpSumEMax = 2000.f;
////            chassis_run.rightdown.fpEMin = 2.0f;



//            chassis_run.leftup.fpKp = 210.f;
//            chassis_run.leftup.fpKi = 0.45f;
//            chassis_run.leftup.fpKd = 0.1f;
//            chassis_run.leftup.fpSumEMax = 6000.f;
//            chassis_run.leftup.fpEMin = 0.f;

//            // 右上电机
//            chassis_run.rightup.fpKp = 200.f;
//            chassis_run.rightup.fpKi = 0.45f;
//            chassis_run.rightup.fpKd = 0.15f;
//            chassis_run.rightup.fpSumEMax = 4000.f;
//            chassis_run.rightup.fpEMin = 0.f;

//            // 左下电机
//            chassis_run.leftdown.fpKp = 170.f;
//            chassis_run.leftdown.fpKi = 0.4f;
//            chassis_run.leftdown.fpKd = 0.15f;
//            chassis_run.leftdown.fpSumEMax = 4000.f;
//            chassis_run.leftdown.fpEMin = 2.0f;

//            // 右下电机（爬坡专用）
//            chassis_run.rightdown.fpKp = 200.f;
//            chassis_run.rightdown.fpKi = 0.45f;
//            chassis_run.rightdown.fpKd = 0.45f;
//            chassis_run.rightdown.fpSumEMax = 2000.f;
//            chassis_run.rightdown.fpEMin = 2.0f;
//        }
//        // ======================
//        // 平地模式：恢复另一套参数
//        // ======================
//        else
//        {   


//				
//			
//			if(fabs(chassis_run.leftup.fpDes)>0.5)      
//            {// 左上电机
//            chassis_run.leftup.fpKp = 170.f;//170
//            chassis_run.leftup.fpKi = 0.7f;
//            chassis_run.leftup.fpKd = 0.1f;
//            chassis_run.leftup.fpSumEMax = 6000.f;
//            chassis_run.leftup.fpEMin = 0.f;}
//			
//			else {
//					// 左上电机
//chassis_run.leftup.fpKp = 0.f;
//chassis_run.leftup.fpKi = 0.f;
//chassis_run.leftup.fpKd = 0.f;
//chassis_run.leftup.fpSumEMax = 0.f;
//chassis_run.leftup.fpEMin = 2.0f;}

//			

//if (fabs(chassis_run.rightup.fpDes)>0.5)
//{



//            // 右上电机
//            chassis_run.rightup.fpKp = 150.f;
//            chassis_run.rightup.fpKi = 0.45f;
//            chassis_run.rightup.fpKd = 0.15f;
//            chassis_run.rightup.fpSumEMax = 8000.f;
//            chassis_run.rightup.fpEMin = 0.f;}
//						
//			else {
//						// 右上电机
//chassis_run.rightup.fpKp = 0.f;
//chassis_run.rightup.fpKi = 0.f;
//chassis_run.rightup.fpKd = 0.f;
//chassis_run.rightup.fpSumEMax = 0.f;
//chassis_run.rightup.fpEMin = 2.0f;}


//	

//if (fabs(chassis_run.leftdown.fpDes)>0.5)
//{
//	


//            // 左下电机
//            chassis_run.leftdown.fpKp = 160.f;
//            chassis_run.leftdown.fpKi = 0.7f;
//            chassis_run.leftdown.fpKd = 0.15f;
//            chassis_run.leftdown.fpSumEMax = 8000.f;
//            chassis_run.leftdown.fpEMin = 2.0f;}

////            // 左下电机
////            chassis_run.leftdown.fpKp = 170.f;
////            chassis_run.leftdown.fpKi = 0.4f;
////            chassis_run.leftdown.fpKd = 0.15f;
////            chassis_run.leftdown.fpSumEMax = 4000.f;
////            chassis_run.leftdown.fpEMin = 2.0f;}
//			else {
//						// 左下电机
//chassis_run.leftdown.fpKp = 0.f;
//chassis_run.leftdown.fpKi = 0.f;
//chassis_run.leftdown.fpKd = 0.f;
//chassis_run.leftdown.fpSumEMax = 0.f;
//chassis_run.leftdown.fpEMin = 2.0f;}


//if (fabs(chassis_run.rightdown.fpDes)>0.5)
//{
//	

//            // 右下电机（平地专用）
//            chassis_run.rightdown.fpKp = 120.f;//100
//            chassis_run.rightdown.fpKi = 0.9f;
//            chassis_run.rightdown.fpKd = 0.45f;
//            chassis_run.rightdown.fpSumEMax = 8000.f;
//            chassis_run.rightdown.fpEMin = 2.0f;}
//						
//			else {
//						// 右下电机
//chassis_run.rightdown.fpKp = 0.f;
//chassis_run.rightdown.fpKi = 0.f;
//chassis_run.rightdown.fpKd = 0.f;
//chassis_run.rightdown.fpSumEMax = 0.f;
//chassis_run.rightdown.fpEMin = 0.f;}
//						
//						
//		}

//}
//







			// AdaptiveLPF_Update(&chassis_run.rightdown.LPF,chassis_run.rightdown.fpFB,chassis_run.rightdown.fpDes);
			// AdaptiveLPF_Update(&chassis_run.rightup.LPF,chassis_run.rightup.fpFB,chassis_run.rightup.fpDes);
			// AdaptiveLPF_Update(&chassis_run.leftdown.LPF,chassis_run.leftdown.fpFB,chassis_run.leftdown.fpDes);
			// AdaptiveLPF_Update(&chassis_run.leftup.LPF,chassis_run.leftup.fpFB,chassis_run.leftup.fpDes);
			
//			PID_Calc_New(&chassis_run.rightdown);
//			PID_Calc_New(&chassis_run.rightup);
//			PID_Calc_New(&chassis_run.leftdown);
//			PID_Calc_New(&chassis_run.leftup);
		
//PI_Feedforward_Calc(&cha);
		PI_Feedforward_Calc(&chassis_run.leftdown, chassis_run.leftdown.fpDes, chassis_run.leftdown.fpFB);
		PI_Feedforward_Calc(&chassis_run.leftup, chassis_run.leftup.fpDes, chassis_run.leftup.fpFB);
		PI_Feedforward_Calc(&chassis_run.rightup, chassis_run.rightup.fpDes, chassis_run.rightup.fpFB);
		PI_Feedforward_Calc(&chassis_run.rightdown, chassis_run.rightdown.fpDes, chassis_run.rightdown.fpFB);


 //下面是防止输出为0让自研电调重新初始化
      if((fabs((float)chassis_run.leftdown.fpU)<=1)||(fabs((float)chassis_run.leftdown.fpDes)<3)) {chassis_run.leftdown.fpU=1;chassis_run.leftdown.fpDes=0;}
			if((fabs((float)chassis_run.leftup.fpU)<=1)||(fabs((float)chassis_run.leftup.fpDes)<3)) {chassis_run.leftup.fpU=1;chassis_run.leftup.fpDes=0;}
			if((fabs((float)chassis_run.rightup.fpU)<=1)||(fabs((float)chassis_run.rightup.fpDes)<3)) {chassis_run.rightup.fpU=1;chassis_run.rightup.fpDes=0;}
			if((fabs((float)chassis_run.rightdown.fpU)<=1)||(fabs((float)chassis_run.rightdown.fpDes)<3)) {chassis_run.rightdown.fpU=1;chassis_run.rightdown.fpDes=0;}
			
			
//			if(fabs((float)chassis_run.leftup.fpU)<=1)   {chassis_run.leftup.fpU=1;};
//			if(fabs((float)chassis_run.rightup.fpU)<=1)  {chassis_run.rightup.fpU=1;};
//			if(fabs((float)chassis_run.rightdown.fpU)<=1) {chassis_run.rightdown.fpU=1;};
			
			
			
			motor_SendCurent(0x200,chassis_run.leftdown.fpU,chassis_run.leftup.fpU,chassis_run.rightup.fpU,chassis_run.rightdown.fpU);//输出在这里
//			motor_SendCurent(0x200,chassis_run.leftdown.fpU,chassis_run.leftup.fpU,chassis_run.rightup.fpU,0);
//						motor_SendCurent(0x200,1,1,1,1);
			
			
		
	}

