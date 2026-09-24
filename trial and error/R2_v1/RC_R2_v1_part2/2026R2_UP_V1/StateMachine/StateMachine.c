#include "StateMachine.h"
#include "Robot.h"
#include "DoubleArm.h"
#include "LK_motor.h"
#include "DJI_motor.h"
#include "J60_motor.h"
uint8_t flags[4];
uint32_t SA_cnt=0;
uint8_t lilState=0;//伸缩臂完成动作过程中的状态标志位
TOTAL_STATE machineState ={.SA={.SA_State = STANDBY_SA,.Cplt_State = CPLT},.DA={.DA_State=STANDBY_DA,.Cplt_State =CPLT}};
void stateMachine(void){
	
	static uint32_t cnt;
	switch(machineState.DA.DA_State){
		case STANDBY_DA:
			if(machineState.DA.Cplt_State==CPLT){
				switch(action_DA){
					case 1://准备取武器头
						machineState.DA.Cplt_State=UNCPLT;
						feedback_DA =1;
						break;
					case 4://准备存方块
						machineState.DA.Cplt_State=UNCPLT;
						feedback_DA = 4;
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						feedback_DA=9;
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						wristCtrl_L(SBDA_L_ROLL,SBDA_L_PITCH,&wrist_L);
						wristCtrl_L(SBDA_R_ROLL,SBDA_R_PITCH,&wrist_R);
						leftShoulderCMD.position_=SBDA_L_J60;
						rightShoulderCMD.position_=SBDA_R_J60;
						break;
				}
			}else{
				switch(action_DA){
					case 1://准备取武器头
						//一堆位控函数；
							
								cnt++;
								AirOperaterCtrlBuf[0]=PUSH;
								AirOperaterCtrlBuf[1]=PUSH;
								AirOperaterCtrlBuf[2]=LIL_SUCK;
								pushAndPull(PUMP_STOP,1);
								AirOperaterCtrlBuf[3]=OPEN;
								AirOperaterCtrlBuf[4]=OPEN;
								flags[0]=wristCtrl_L(rampSignalFP(SBDA_L_ROLL,RTGW_L_ROLL,cnt-300,400),rampSignalFP(SBDA_L_PITCH,RTGW_L_PITCH,cnt-300,400),&wrist_L);
								flags[1]=wristCtrl_L(rampSignalFP(SBDA_R_ROLL,RTGW_R_ROLL,cnt-300,400),rampSignalFP(SBDA_R_PITCH,RTGW_R_PITCH,cnt-300,400),&wrist_R);
								leftShoulderCMD.position_=rampSignalFP(SBDA_L_J60,RTGW_L_J60,cnt,1000) ;
								rightShoulderCMD.position_ = rampSignalFP(SBDA_R_J60,RTGW_R_J60,cnt,1000) ;
							//如果机构到位，则令machineState.DA.DA_State=READY_TO_GRAB_WEAPON;并令machineState.DA.Cplt_State=CPLT;
							if(cnt>=1000){
									machineState.DA.DA_State=READY_TO_GRAB_WEAPON;
									machineState.DA.Cplt_State=CPLT;
									feedback_DA=10;
									cnt=0;
							}					
						break;
					case 4://准备存方块
						//一堆位控函数；
						cnt++;
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						wristCtrl_L(rampSignalFP(SBDA_L_ROLL,RTSK_L_ROLL,cnt,500) ,rampSignalFP(SBDA_L_PITCH,RTSK_L_PITCH,cnt,500),&wrist_L);
						wristCtrl_L(rampSignalFP(SBDA_R_ROLL,RTSK_R_ROLL,cnt,500) ,rampSignalFP(SBDA_R_PITCH,RTSK_R_PITCH,cnt,500),&wrist_R);
						leftShoulderCMD.position_=rampSignalFP(SBDA_L_J60,RTSK_L_J60,cnt,500) ;
						rightShoulderCMD.position_ =rampSignalFP(SBDA_R_J60,RTSK_R_J60,cnt,500);
						//如果机构到位，则令machineState.DA.DA_State=READY_TO_STORAGE_KFS;并令machineState.DA.Cplt_State=CPLT;
						if(cnt>550){
							if(fabs(leftShoulder.position_-RTSK_L_J60)<0.06f&&fabs(rightShoulder.position_-RTSK_R_J60)<0.06f){
								machineState.DA.DA_State=READY_TO_STORAGE_KFS;
								machineState.DA.Cplt_State=CPLT;
								feedback_DA=40;
								cnt=0;
							}
						}
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case READY_TO_GRAB_WEAPON:
			if(machineState.DA.Cplt_State==CPLT){
				switch(action_DA){
					case 2://左臂开始取武器头
						feedback_DA=2;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					case 3://右臂开始取武器头
						feedback_DA=3;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					default:
						AirOperaterCtrlBuf[0]=PUSH;
						AirOperaterCtrlBuf[1]=PUSH;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTGW_L_ROLL,RTGW_L_PITCH,&wrist_L);
						flags[1]=wristCtrl_L(RTGW_R_ROLL ,RTGW_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_=RTGW_L_J60 ;
						rightShoulderCMD.position_ = RTGW_R_J60 ;
						break;
				}
			}else{
				switch(action_DA){
					case 2://左臂开始取武器头
						if(cnt<1500){
							AirOperaterCtrlBuf[0]=PUSH;
							AirOperaterCtrlBuf[1]=PUSH;
							AirOperaterCtrlBuf[2]=LIL_SUCK;
							pushAndPull(PUMP_STOP,1);
							if(cnt<500){
								AirOperaterCtrlBuf[3]=OPEN;
							}else{

								AirOperaterCtrlBuf[3]=CLOSE; 
							}
							AirOperaterCtrlBuf[4]=OPEN;
							flags[0]=wristCtrl_L(RTGW_L_ROLL,RTGW_L_PITCH,&wrist_L);
							flags[1]=wristCtrl_L(RTGW_R_ROLL,RTGW_R_PITCH,&wrist_R);
							leftShoulderCMD.position_=RTGW_L_J60 ;
							rightShoulderCMD.position_ = RTGW_R_J60 ;
							LK_Pos_CtrlWithoutPID(&stretch_LK,RTGK_LK);
							cnt++;
						}else if(cnt>=1500&&cnt<2000){
							AirOperaterCtrlBuf[3]=CLOSE;
							flags[0]=wristCtrl_L(RTGW_L_ROLL,-33,&wrist_L);
							cnt++;
						}else if(cnt>=2000){
							AirOperaterCtrlBuf[3]=CLOSE;
							if(cnt>2200){
								AirOperaterCtrlBuf[0]=PULL;
							}
							if(cnt>2300){
								flags[1]=wristCtrl_L(WC_L_ROLL ,WC_L_PITCH,&wrist_L);
							}else{
								wristCtrl_L(RTGW_L_ROLL ,-33 ,&wrist_L);
							}
							leftShoulderCMD.position_=rampSignalFP(RTGW_L_J60,WC_L_J60,cnt-2000,750);
							LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,SBSA_LK,cnt-2000,500));
							cnt++;
							if(cnt>=3000){
								if(fabs(leftShoulder.position_-WC_L_J60)<0.06f){
									cnt=0;
									machineState.DA.DA_State=WAIT_COMBINE;
								}
							}
						}
						break;
					case 3://右臂开始取武器头
						if(cnt<1500){
							AirOperaterCtrlBuf[0]=PULL;
							AirOperaterCtrlBuf[1]=PUSH;
							AirOperaterCtrlBuf[2]=LIL_SUCK;
							pushAndPull(PUMP_STOP,1);
							AirOperaterCtrlBuf[3]=OPEN;
							if(cnt<500){
								AirOperaterCtrlBuf[4]=OPEN;
							}else{
								AirOperaterCtrlBuf[4]=CLOSE; 
							}
							flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
							flags[1]=wristCtrl_L(RTGW_R_ROLL ,RTGW_R_PITCH ,&wrist_R);
							leftShoulderCMD.position_=WC_L_J60 ;
							rightShoulderCMD.position_ = RTGW_R_J60 ;
							LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
							cnt++;
						}else if(cnt>=1500&&cnt<2000){
							AirOperaterCtrlBuf[4]=CLOSE;
							flags[1]=wristCtrl_L(RTGW_R_ROLL ,27 ,&wrist_R);
							cnt++;
						}else if(cnt>=2000){
							AirOperaterCtrlBuf[4]=CLOSE;
							AirOperaterCtrlBuf[0]=PULL;
							if(cnt>2200){
								AirOperaterCtrlBuf[1]=PULL;
							}
							if(cnt>2500){
								flags[1]=wristCtrl_L(WC_R_ROLL ,WC_R_PITCH,&wrist_R);
							}else{
								wristCtrl_L(RTGW_R_ROLL ,27 ,&wrist_R);
							}
							rightShoulderCMD.position_ =rampSignalFP(RTGW_R_J60,WC_R_J60,cnt-2000,600) ;
							cnt++;
							if(cnt>=3000){
								if(fabs(leftShoulder.position_-WC_L_J60)<0.06f&&fabs(rightShoulder.position_-WC_R_J60)<0.06f){
									cnt=0;
									machineState.DA.DA_State=WAIT_COMBINE;
								}
							}
						}
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case WAIT_COMBINE:
			if(machineState.DA.Cplt_State==CPLT){
				switch(action_DA){
					case 0://	回复初始状态
						feedback_DA=0;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					case 2://左臂等待组装武器
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PUSH;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
						flags[1]=wristCtrl_L(RTGW_R_ROLL ,RTGW_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_=WC_L_J60 ;
						rightShoulderCMD.position_ = RTGW_R_J60 ;
						LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
						break;
					case 3://右臂等待组装武器
						feedback_DA=3;
						machineState.DA.DA_State=READY_TO_GRAB_WEAPON;
						LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
						break;
					case 4://准备存方块
						feedback_DA=4;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=CLOSE;
						AirOperaterCtrlBuf[4]=CLOSE;
						wristCtrl_L(SBDA_L_ROLL,SBDA_L_PITCH,&wrist_L);
						wristCtrl_L(SBDA_R_ROLL,SBDA_R_PITCH,&wrist_R);
						leftShoulderCMD.position_=SBDA_L_J60;
						rightShoulderCMD.position_=SBDA_R_J60;
						break;
				}
			}else{
				switch(action_DA){
					case 0:
						//一堆位控函数；
						cnt++;
						AirOperaterCtrlBuf[0]=PUSH;
						AirOperaterCtrlBuf[1]=PUSH;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(SBDA_L_ROLL,SBDA_L_PITCH,&wrist_L);
						flags[1]=wristCtrl_L(SBDA_R_ROLL,SBDA_R_PITCH,&wrist_R);
						leftShoulderCMD.position_=rampSignalFP(WC_L_J60,SBDA_L_J60,cnt,100);
						rightShoulderCMD.position_=rampSignalFP(RTGW_R_J60,SBDA_R_J60,cnt,500);
						if(flags[0]&&flags[1]){
							if(fabs(leftShoulder.position_-SBDA_L_J60)<0.06f&&fabs(rightShoulder.position_-SBDA_R_J60)<0.06f){
								machineState.DA.DA_State=STANDBY_DA;
								machineState.DA.Cplt_State=CPLT;
								cnt=0;
								feedback_DA=9;
							}
						}
						//如果到达位置，则令machineState.DA.DA_State=STANDBY_DA;并令machineState.DA.Cplt_State=CPLT;
						break;
					case 2:
						if(cnt<5000){
							AirOperaterCtrlBuf[0]=PULL;
							AirOperaterCtrlBuf[1]=PUSH;
							AirOperaterCtrlBuf[2]=LIL_SUCK;
							pushAndPull(PUMP_STOP,1);
							AirOperaterCtrlBuf[3]=CLOSE;
							AirOperaterCtrlBuf[4]=OPEN;
							flags[0]=wristCtrl_L(WC_L_ROLL ,WC_L_PITCH ,&wrist_L);
							flags[1]=wristCtrl_L(RTGW_R_ROLL ,RTGW_R_PITCH ,&wrist_R);
							leftShoulderCMD.position_=WC_L_J60 ;
							rightShoulderCMD.position_ = RTGW_R_J60 ;
							LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
							cnt++;
						}else if(cnt>=5000&&cnt<5500){
							AirOperaterCtrlBuf[3]=OPEN;
							cnt++;
						}else if(cnt>=5500&&cnt<6300){
							if(cnt>5700){
								wristCtrl_L(RTSK_L_ROLL,RTSK_L_PITCH ,&wrist_L);
							}else{
								wristCtrl_L(WC_L_ROLL ,WC_L_PITCH ,&wrist_L);
							}
							cnt++;
						}else if(cnt>=6300){
							
							cnt=0;
							machineState.DA.Cplt_State=CPLT;
							feedback_DA=20;
						}
						break;
					case 3:
						if(cnt<1000){
							AirOperaterCtrlBuf[0]=PULL;
							AirOperaterCtrlBuf[1]=PULL;
							AirOperaterCtrlBuf[2]=LIL_SUCK;
							pushAndPull(PUMP_STOP,1);
							AirOperaterCtrlBuf[3]=OPEN;
							AirOperaterCtrlBuf[4]=CLOSE;
							flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
							flags[1]=wristCtrl_L(WC_R_ROLL ,WC_R_PITCH ,&wrist_R);
							leftShoulderCMD.position_=WC_L_J60 ;
							rightShoulderCMD.position_ = WC_R_J60 ;
							LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
							cnt++;
						}else if(cnt>=1000&&cnt<1500){
							AirOperaterCtrlBuf[4]=OPEN;
							LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
							cnt++;
						}else if(cnt>=1500&&cnt<4500){
							flags[1]=wristCtrl_L(RTSK_R_ROLL ,RTSK_R_PITCH ,&wrist_R);
							if(cnt>2500){
								LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(SBSA_LK,RTGK_LK,cnt-2500,1000));
							}else{
								LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
							}
							cnt++;
						}else if(cnt>=4500){
							LK_Pos_CtrlWithoutPID(&stretch_LK,RTGK_LK);
							leftShoulderCMD.position_=rampSignalFP(WC_L_J60,RTSK_L_J60,cnt-4500,680);
							rightShoulderCMD.position_ = rampSignalFP(WC_R_J60,RTSK_R_J60,cnt-4500,680);
							cnt++;
							if(cnt>5200){
								if(fabs(leftShoulder.position_-RTSK_L_J60)<0.06f&&fabs(rightShoulder.position_-RTSK_R_J60)<0.06f){
									cnt=0;
									machineState.DA.DA_State=READY_TO_STORAGE_KFS;
									machineState.DA.Cplt_State=CPLT;
									feedback_DA=30;
								}
							}
						}
						break;
					case 4:
						//一堆位控函数；
						cnt++;
						AirOperaterCtrlBuf[0]=PUSH;
						AirOperaterCtrlBuf[1]=PUSH;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_RUN,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
						flags[1]=wristCtrl_L(RTSK_R_ROLL ,RTSK_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_ =rampSignalFP(WC_L_J60,RTSK_L_J60,cnt,600) ;
						rightShoulderCMD.position_=rampSignalFP(WC_R_J60,RTSK_R_J60,cnt,600) ;
						if(flags[0]&&flags[1]){
							if(fabs(leftShoulder.position_-RTSK_L_J60)<0.06f&&fabs(rightShoulder.position_-RTSK_R_J60)<0.06f){
								cnt=0;
								machineState.DA.DA_State=READY_TO_STORAGE_KFS;
								machineState.DA.Cplt_State=CPLT;
								feedback_DA=40;
							}
						}
						//如果到达位置，则令machineState.DA.DA_State=READY_TO_STORAGE_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case READY_TO_STORAGE_KFS:
			if(machineState.DA.Cplt_State==CPLT){
				switch(action_DA){
					case 5://存方块
						feedback_DA=5;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
						flags[1]=wristCtrl_L(RTSK_R_ROLL ,RTSK_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_ = RTSK_L_J60 ;
						rightShoulderCMD.position_= RTSK_R_J60 ;
						break;
				}
			}else{
				switch(action_DA){
					case 5:
						//一堆位控函数；
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_RUN,1);
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
						flags[1]=wristCtrl_L(RTSK_R_ROLL ,RTSK_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_ = RTSK_L_J60 ;
						rightShoulderCMD.position_= RTSK_R_J60 ;
						if(feedback_SA==40){
							machineState.DA.DA_State=STORAGE_KFS;
							machineState.DA.Cplt_State=CPLT;
							feedback_DA=50;
						}
						//如果到达位置，则令machineState.DA.DA_State=READY_TO_USE_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case STORAGE_KFS:
			if(machineState.DA.Cplt_State==CPLT){
				switch(action_DA){
					case 0://回复初始动作
						feedback_DA=0;
						machineState.DA.Cplt_State=UNCPLT;
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
//						AirOperaterCtrlBuf[2]=LIL_SUCK;
						AirOperaterCtrlBuf[3]=OPEN;
						AirOperaterCtrlBuf[4]=OPEN;
						flags[0]=wristCtrl_L(RTSK_L_ROLL ,RTSK_L_PITCH ,&wrist_L);
						flags[1]=wristCtrl_L(RTSK_R_ROLL ,RTSK_R_PITCH ,&wrist_R);
						leftShoulderCMD.position_ = RTSK_L_J60 ;
						rightShoulderCMD.position_= RTSK_R_J60 ;
						break;
				}
			}else{
				switch(action_DA){
					case 0:
						//一堆位控函数；
						AirOperaterCtrlBuf[0]=PULL;
						AirOperaterCtrlBuf[1]=PULL;
						AirOperaterCtrlBuf[2]=LIL_SUCK;
						pushAndPull(PUMP_STOP,1);
						AirOperaterCtrlBuf[3]=CLOSE;
						AirOperaterCtrlBuf[4]=CLOSE;
						flags[0]=wristCtrl_L(SBDA_L_ROLL,SBDA_L_PITCH,&wrist_L);
						flags[1]=wristCtrl_L(SBDA_R_ROLL,SBDA_R_PITCH,&wrist_R);
						leftShoulderCMD.position_=SBDA_L_J60;
						rightShoulderCMD.position_=SBDA_R_J60;
						if(flags[0]&&flags[1]){
							if(fabs(leftShoulder.position_-SBDA_L_J60)<0.06f&&fabs(rightShoulder.position_-SBDA_R_J60)<0.06f){
								machineState.DA.DA_State=STANDBY_DA;
								machineState.DA.Cplt_State=CPLT;
								feedback_DA=9;
							}
						}
						//如果到达位置，则令machineState.DA.DA_State=STANDBY_DA;并令machineState.DA.Cplt_State=CPLT;
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
	}
	
	
	
	if((machineState.DA.DA_State==READY_TO_GRAB_WEAPON||machineState.DA.DA_State==WAIT_COMBINE)){
		return;
	}else{

	switch(machineState.SA.SA_State){
		case STANDBY_SA:
			if(machineState.SA.Cplt_State==CPLT){
				switch(action_SA){
					case 1://准备取方块
						feedback_SA=1;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						pushAndPull(PUMP_STOP,0);
						AirOperaterCtrlBuf[5]=BIG_SUCK;
						LK_Pos_CtrlWithoutPID(&stretch_LK,SBSA_LK);
						DJI_Pos_CtrlWithoutPID(&stretch_2006,SBSA_DJI);
						stretch_DM.ctrl.pos_set=SBSA_DM;
						break;
				}
			}else{
				switch(action_SA){
					case 1:
						//一堆位控函数；
						SA_cnt++;
						pushAndPull(PUMP_STOP,0);
						AirOperaterCtrlBuf[5]=BIG_SUCK;
						flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(SBSA_LK,RTGK_LK,SA_cnt,400));
						flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,RTGK_DJI);
						stretch_DM.ctrl.pos_set=rampSignalFP(SBSA_DM,RTGK_DM,SA_cnt,400);
						//如果到达位置，则令machineState.SA.SA_State=READY_TO_GET_KFS;并令machineState.SA.Cplt_State=CPLT;
						if(flags[2]&&flags[3]){
							if(SA_cnt>410){
								machineState.SA.SA_State=READY_TO_GET_KFS;
								machineState.SA.Cplt_State=CPLT;
								feedback_SA=10;
								SA_cnt=0;
								gravityCompensation_SA_state=0;
							}
						}
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case READY_TO_GET_KFS:
			if(machineState.SA.Cplt_State==CPLT){
				switch(action_SA){
					case 0://恢复初始动作
						feedback_SA=0;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					case 2://取方块
						feedback_SA=2;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					case 6:// 取存着的方块
						feedback_SA=6;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					case 8://小脚站起来取方块
						feedback_SA=8;
						machineState.SA.Cplt_State=UNCPLT;
						break;						
					default:
						//一堆位控函数，保持进此状态时的位置；
						pushAndPull(PUMP_STOP,0);
						AirOperaterCtrlBuf[5]=BIG_SUCK;
						flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,RTGK_LK);
						flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,RTGK_DJI);
						stretch_DM.ctrl.pos_set=RTGK_DM;
						break;
				}
			}else{
				switch(action_SA){
					case 0:
						SA_cnt++;
						pushAndPull(PUMP_STOP,0);
						AirOperaterCtrlBuf[5]=BIG_SUCK;
						flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,SBSA_LK,SA_cnt,400));
						flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,SBSA_DJI);
						stretch_DM.ctrl.pos_set=rampSignalFP(RTGK_DM,SBSA_DM,SA_cnt,400);
						if(flags[2]&&flags[3]){
							if(SA_cnt>450){
								machineState.SA.SA_State=STANDBY_SA;
								machineState.SA.Cplt_State=CPLT;
								feedback_SA=9;
								SA_cnt=0;
							}
						}
						break;
					case 2:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								if(height_flag==DOWN_KFS){
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,GKFD_LK,SA_cnt,500));
									stretch_DM.ctrl.pos_set=rampSignalFP(RTGK_DM,GKFD_DM,SA_cnt,500);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(RTGK_DJI,GKFD_DJI,SA_cnt,500));
								}else{
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,GKFU_LK,cnt,500));
									stretch_DM.ctrl.pos_set=rampSignalFP(RTGK_DM,GKFU_DM,SA_cnt,500);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,RTGK_DJI);
								}
								if(flags[3]&&SA_cnt>=500){
										lilState++;
										SA_cnt=0;
									
								}
								break;
							case 1:
								SA_cnt++;
								if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFD_LK);
										stretch_DM.ctrl.pos_set=GKFD_DM;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,GKFD_DJI);
										if(SA_cnt>500){
											lilState++;									
											SA_cnt=0;
										}
								}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFU_LK);
										stretch_DM.ctrl.pos_set=GKFU_DM;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(RTGK_DJI,GKFU_DJI,SA_cnt,1200));
										if(fabs(stretch_2006.angle-GKFU_DJI)<2){
											lilState++;									
											SA_cnt=0;
										}
									}
								
								
								break;
							case 2:
								if(SA_cnt<500){
									gravityCompensation_SA_state=1;
									pushAndPull(PUMP_RUN,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFD_LK);
										stretch_DM.ctrl.pos_set=GKFD_DM;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,GKFD_DJI);
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFU_LK);
										stretch_DM.ctrl.pos_set=GKFU_DM;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,GKFU_DJI);
									}
									
									SA_cnt++;
								}else if(SA_cnt>=500){
									SA_cnt++;
									pushAndPull(PUMP_RUN,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(GKFD_LK,HK_LK,SA_cnt-500,800));
										stretch_DM.ctrl.pos_set=rampSignalFP(GKFD_DM,HK_DM,SA_cnt-500,800);
										DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(GKFD_DJI,HK_DJI,SA_cnt-500,800));
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(GKFU_LK,HK_LK,SA_cnt-500,800));
										stretch_DM.ctrl.pos_set=rampSignalFP(GKFU_DM,HK_DM,SA_cnt-500,800);
										DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(GKFU_DJI,HK_DJI,SA_cnt-500,800));
									}
									if(SA_cnt>=1300&&fabs(stretch_2006.angle-HK_DJI)<2){
											lilState=0;
											SA_cnt=0;
											machineState.SA.SA_State = HOLD_KFS;
											machineState.SA.Cplt_State = CPLT;
											feedback_SA=20;
									}
								}
								break;
							default:
								break;
						}	
						break;
					case 6:
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,HKWD_LK-2.5f,SA_cnt,500));
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(RTGK_DJI,HKWD_DJI+100,SA_cnt,500));
								stretch_DM.ctrl.pos_set=rampSignalFP(RTGK_DM,HKWD_DM+0.12f,SA_cnt,500);
								if(SA_cnt>=500){
									
										lilState++;
										SA_cnt=0;
									
								}
								break;
							case 1:
								if(SA_cnt<600){
									AirOperaterCtrlBuf[2]=LIL_SUCK;
									pushAndPull(PUMP_RUN,1);
									pushAndPull(PUMP_RUN,0);SA_cnt++;
								}else if(SA_cnt>=600&&SA_cnt<1200){
									if(SA_cnt<800){
										AirOperaterCtrlBuf[2]=LIL_SPIT;
									}else{
										AirOperaterCtrlBuf[2]=LIL_SUCK;
									}
									pushAndPull(PUMP_STOP,1);
									pushAndPull(PUMP_RUN,0);
									SA_cnt++;
								}else if(SA_cnt>=1200){
									SA_cnt++;
									gravityCompensation_SA_state=1;
									pushAndPull(PUMP_RUN,0);
									pushAndPull(PUMP_STOP,1);
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HKWD_LK-2.5f,HK_LK,SA_cnt-1200,800));
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HKWD_DJI+100,HK_DJI,SA_cnt-1200,800));
									stretch_DM.ctrl.pos_set=rampSignalFP(HKWD_DM+0.12f,HK_DM,SA_cnt-1200,800);
									if(SA_cnt>=2000){
											lilState=0;
											SA_cnt=0;
											machineState.SA.SA_State = HOLD_KFS;
											machineState.SA.Cplt_State = CPLT;
											feedback_SA=60;
										
									}
								}
								break;
							default:
								break;
						}
						break;
					case 8:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(RTGK_LK,-40,cnt,500));
									stretch_DM.ctrl.pos_set=rampSignalFP(RTGK_DM,-0.3f,SA_cnt,500);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,-200);
								
								if(flags[3]&&SA_cnt>=500){
										lilState++;
										SA_cnt=0;
									
								}
								break;
							case 1:
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								SA_cnt++;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,-40);
										stretch_DM.ctrl.pos_set=-0.35f;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(-300,250,SA_cnt,1200));
										if(fabs(stretch_2006.angle-250)<2){
											lilState++;									
											SA_cnt=0;
										}
									
								
								break;
							case 2:
								if(SA_cnt<500){
									gravityCompensation_SA_state=1;
									pushAndPull(PUMP_RUN,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,-40);
										stretch_DM.ctrl.pos_set=-0.35f;
										DJI_Pos_CtrlWithoutPID(&stretch_2006,250);
									
									
									SA_cnt++;
								}else if(SA_cnt>=500){
									SA_cnt++;
									pushAndPull(PUMP_RUN,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(-40,HK_LK,SA_cnt-500,800));
										stretch_DM.ctrl.pos_set=rampSignalFP(-0.35f,HK_DM,SA_cnt-500,800);
										DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(250,HK_DJI,SA_cnt-500,800));
									
									if(SA_cnt>=1300&&fabs(stretch_2006.angle-HK_DJI)<2){
											lilState=0;
											SA_cnt=0;
											machineState.SA.SA_State = HOLD_KFS;
											machineState.SA.Cplt_State = CPLT;
											feedback_SA=80;
											
										
									}
								}
								break;
							default:
								break;
						}	
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
		case HOLD_KFS:
			if(machineState.SA.Cplt_State==CPLT){
				switch(action_SA){
					case 3://扔方块
						feedback_SA=3;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					case 4: //存方块
						if(machineState.DA.DA_State==READY_TO_STORAGE_KFS&&machineState.DA.Cplt_State==CPLT){
							feedback_SA=4;
							machineState.SA.Cplt_State=UNCPLT;
						}else{
							//一堆位控函数，保持进此状态时的位置；
							pushAndPull(PUMP_RUN,0);
							AirOperaterCtrlBuf[5]=BIG_SUCK;
							flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
							flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HK_DJI);
							stretch_DM.ctrl.pos_set=HK_DM;
						}
						break;
					case 5://用方块
						if(feedback_SA!=60){
							feedback_SA=5;
							machineState.SA.Cplt_State=UNCPLT;
						}else{
							pushAndPull(PUMP_RUN,0);
							AirOperaterCtrlBuf[5]=BIG_SUCK;
							flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
							flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HK_DJI);
							stretch_DM.ctrl.pos_set=HK_DM;
						}
						break;
					case 7://转向后扔方块
						feedback_SA=7;
						machineState.SA.Cplt_State=UNCPLT;
						break;
					case 11:
						if(feedback_SA!=12){
							feedback_SA=11;
							machineState.SA.Cplt_State=UNCPLT;
						}else{
							pushAndPull(PUMP_RUN,0);
							AirOperaterCtrlBuf[5]=BIG_SUCK;
							flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
							flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HK_DJI);
							stretch_DM.ctrl.pos_set=HK_DM;
						}
						break;
					default:
						//一堆位控函数，保持进此状态时的位置；
						pushAndPull(PUMP_RUN,0);
						AirOperaterCtrlBuf[5]=BIG_SUCK;
						flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
						flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HK_DJI);
						stretch_DM.ctrl.pos_set=HK_DM;
						break;
				}
			}else{
				switch(action_SA){
					case 3:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								if(height_flag==DOWN_KFS){
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,LKOD_LK,SA_cnt,1000));
									stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,LKOD_DM,SA_cnt,1000);
								}else{
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,LKOU_LK,SA_cnt,1000));
									stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,LKOU_DM,SA_cnt,1000);
								}
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,LKOU_DJI,SA_cnt,1000));
								if(flags[3]&&SA_cnt>=1000){
									lilState++;
									SA_cnt=0;
								}
								break;
							case 1:
								if(SA_cnt<200){
									gravityCompensation_SA_state=0;
									pushAndPull(PUMP_STOP,0);
									if(SA_cnt<50){
										AirOperaterCtrlBuf[5]=BIG_SPIT;
									}else{
										AirOperaterCtrlBuf[5]=BIG_SUCK;
									}
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,LKOD_LK);
										stretch_DM.ctrl.pos_set=LKOD_DM;
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,LKOU_LK);
										stretch_DM.ctrl.pos_set=LKOU_DM;
									}
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,LKOU_DJI);
									SA_cnt++;
								}else if(SA_cnt>=200){
									SA_cnt++;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(LKOD_LK,RTGK_LK,SA_cnt,1000));
										stretch_DM.ctrl.pos_set=rampSignalFP(LKOD_DM,RTGK_DM,SA_cnt-200,1000);
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(LKOU_LK,RTGK_LK,SA_cnt,1000));
										stretch_DM.ctrl.pos_set=rampSignalFP(LKOD_DM,RTGK_DM,SA_cnt-200,1000);
									}
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(LKOD_DJI,RTGK_DJI,SA_cnt-200,1000));
									if(flags[2]&&flags[3]){
										if(SA_cnt>1250){
											lilState=0;
											SA_cnt=0;
											machineState.SA.SA_State = READY_TO_GET_KFS;
											machineState.SA.Cplt_State = CPLT;
											feedback_SA=30;
										}
									}
								}
								break;
							default:
								break;
						}
						//如果动作完成，则令machineState.DA.DA_State=READY_TO_GET_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					case 4:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[2]=LIL_SUCK;
								pushAndPull(PUMP_RUN,1);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,HKWD_LK+1,SA_cnt,800));
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,HKWD_DJI,SA_cnt,800));
								stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,HKWD_DM,SA_cnt,800);
								if(SA_cnt>900){
									if(SA_cnt>850){
										lilState++;
										SA_cnt=0;
									}
								}
								break;
							case 1:
								if(SA_cnt<400){
									gravityCompensation_SA_state=0;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SPIT;
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HKWD_LK+1);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HKWD_DJI);
									stretch_DM.ctrl.pos_set=HKWD_DM;
									SA_cnt++;
								}else if(SA_cnt>=400){
									SA_cnt++;
									pushAndPull(PUMP_STOP,0);
									if(SA_cnt<500){
										AirOperaterCtrlBuf[5]=BIG_SPIT;
									}else{
										AirOperaterCtrlBuf[5]=BIG_SUCK;
									}
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HKWD_LK+1,RTGK_LK,SA_cnt-400,700));
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HKWD_DJI,RTGK_DJI,SA_cnt-400,700));
									stretch_DM.ctrl.pos_set=rampSignalFP(HKWD_DM,RTGK_DM,SA_cnt-400,700);
									if(SA_cnt>1200){
										lilState=0;
										SA_cnt=0;
										machineState.SA.SA_State = READY_TO_GET_KFS;
										machineState.SA.Cplt_State = CPLT;
										feedback_SA=40;
										machineState.DA.DA_State = STORAGE_KFS;
										feedback_DA=50;
										AirOperaterCtrlBuf[2]=LIL_SUCK;
										pushAndPull(PUMP_RUN,1);
									}
								}
								break;
							default:
								break;
						}
						//如果动作完成，则令machineState.DA.DA_State=READY_TO_GET_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					case 5:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,UK_DJI,SA_cnt,500));
								stretch_DM.ctrl.pos_set=HK_DM;
								if(fabs(stretch_2006.angle-UK_DJI)<1||SA_cnt>600){
									lilState++;
									SA_cnt=0;
								}
								break;
							case 1:
								SA_cnt++;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,UK_LK,SA_cnt,800));
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,UK_DJI);
								stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,UK_DM,SA_cnt,800);
								if(SA_cnt>800){
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SPIT;
									if(SA_cnt>2800){
										lilState++;
										SA_cnt=0;
									}
								}
								
								break;
							case 2:
								if(SA_cnt<200){
									gravityCompensation_SA_state=0;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SPIT;
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,UK_LK);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,UK_DJI);
									stretch_DM.ctrl.pos_set=UK_DM;
									SA_cnt++;
									feedback_SA=50;
								}else if(SA_cnt>=200&&SA_cnt<1000){
									SA_cnt++;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(UK_LK,HKWD_LK-2.5f,SA_cnt-200,600));
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(UK_DJI,HKWD_DJI+100,SA_cnt-200,600));
									stretch_DM.ctrl.pos_set=rampSignalFP(UK_DM,HK_DM+0.12f,SA_cnt-200,400);
								}else if(SA_cnt>=1000){
									if(SA_cnt<1600){
										AirOperaterCtrlBuf[2]=LIL_SUCK;
										pushAndPull(PUMP_RUN,1);
										pushAndPull(PUMP_RUN,0);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HKWD_LK-2.5f);
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HKWD_DJI+100);
										stretch_DM.ctrl.pos_set=HKWD_DM+0.12f;
										SA_cnt++;
									}else if(SA_cnt>=1600&&SA_cnt<2200){
										AirOperaterCtrlBuf[2]=LIL_SPIT;
										pushAndPull(PUMP_STOP,1);
										pushAndPull(PUMP_RUN,0);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HKWD_LK-2.5f);
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HKWD_DJI+100);
										stretch_DM.ctrl.pos_set=HKWD_DM+0.12f;
										SA_cnt++;
									}else if(SA_cnt>=2200){
										SA_cnt++;
										AirOperaterCtrlBuf[2]=LIL_SPIT;
										gravityCompensation_SA_state=1;
										pushAndPull(PUMP_RUN,0);
										pushAndPull(PUMP_STOP,1);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HKWD_LK-2.5f,HK_LK,SA_cnt-2200,800));
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HKWD_DJI+100,HK_DJI,SA_cnt-2200,800));
										stretch_DM.ctrl.pos_set=rampSignalFP(HKWD_DM+0.12f,HK_DM,SA_cnt-2200,800);
										if(SA_cnt>=3050){
												lilState=0;
												SA_cnt=0;
												machineState.SA.SA_State = HOLD_KFS;
												machineState.SA.Cplt_State = CPLT;
												feedback_SA=60;
											
										}
									}
								}
								break;
							default:
								break;
						}
						//如果动作完成，则令machineState.DA.DA_State=HOLD_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					case 11:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HK_LK);
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,UK_DJI,SA_cnt,500));
								stretch_DM.ctrl.pos_set=HK_DM;
								if(fabs(stretch_2006.angle-UK_DJI)<1||SA_cnt>600){
									lilState++;
									SA_cnt=0;
								}
								break;
							case 1:
								SA_cnt++;
								flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,UK_LK,SA_cnt,800));
								flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,UK_DJI);
								stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,UK_DM,SA_cnt,800);
								if(SA_cnt>1500){
										lilState++;
										SA_cnt=0;
								}
								break;
							case 2:
								if(SA_cnt<200){
									gravityCompensation_SA_state=0;
									pushAndPull(PUMP_STOP,0);
									if(SA_cnt<50){
										AirOperaterCtrlBuf[5]=BIG_SPIT;
									}else{
										AirOperaterCtrlBuf[5]=BIG_SUCK;
									}
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,UK_LK);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,UK_DJI);
									stretch_DM.ctrl.pos_set=UK_DM;
									SA_cnt++;
									feedback_SA=50;
								}else if(SA_cnt>=200&&SA_cnt<1000){
									SA_cnt++;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(UK_LK,HKWD_LK-2.5f,SA_cnt-200,600));
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(UK_DJI,HKWD_DJI+100,SA_cnt-200,600));
									stretch_DM.ctrl.pos_set=rampSignalFP(UK_DM,HK_DM+0.12f,SA_cnt-200,400);
								}else if(SA_cnt>=1000){
									if(SA_cnt<1600){
										AirOperaterCtrlBuf[2]=LIL_SUCK;
										pushAndPull(PUMP_RUN,1);
										pushAndPull(PUMP_RUN,0);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HKWD_LK-2.5f);
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HKWD_DJI+100);
										stretch_DM.ctrl.pos_set=HKWD_DM+0.12f;
										SA_cnt++;
									}else if(SA_cnt>=1600&&SA_cnt<2200){
										AirOperaterCtrlBuf[2]=LIL_SPIT;
										pushAndPull(PUMP_STOP,1);
										pushAndPull(PUMP_RUN,0);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,HKWD_LK-2.5f);
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,HKWD_DJI+100);
										stretch_DM.ctrl.pos_set=HKWD_DM+0.12f;
										SA_cnt++;
									}else if(SA_cnt>=2200){
										SA_cnt++;
										AirOperaterCtrlBuf[2]=LIL_SPIT;
										gravityCompensation_SA_state=1;
										pushAndPull(PUMP_RUN,0);
										pushAndPull(PUMP_STOP,1);
										AirOperaterCtrlBuf[5]=BIG_SUCK;
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HKWD_LK-2.5f,HK_LK,SA_cnt-2200,800));
										flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HKWD_DJI+100,HK_DJI,SA_cnt-2200,800));
										stretch_DM.ctrl.pos_set=rampSignalFP(HKWD_DM+0.12f,HK_DM,SA_cnt-2200,800);
										if(SA_cnt>=3050){
												lilState=0;
												SA_cnt=0;
												machineState.SA.SA_State = HOLD_KFS;
												machineState.SA.Cplt_State = CPLT;
												feedback_SA=12;
											
										}
									}
								}
								break;
							default:
								break;
						}
						//如果动作完成，则令machineState.DA.DA_State=HOLD_KFS;并令machineState.DA.Cplt_State=CPLT;
						break;
					case 7:
						//一堆位控函数；
						switch(lilState){
							case 0:
								SA_cnt++;
								pushAndPull(PUMP_RUN,0);
								AirOperaterCtrlBuf[5]=BIG_SUCK;
								if(height_flag==DOWN_KFS){
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,GKFD_LK,SA_cnt,1000));
									stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,GKFD_DM,SA_cnt,1000);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,GKFD_DJI,SA_cnt,1000));
								}else{
									flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(HK_LK,GKFU_LK,SA_cnt,1000));
									stretch_DM.ctrl.pos_set=rampSignalFP(HK_DM,GKFU_DM-0.15f,SA_cnt,1000);
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(HK_DJI,GKFU_DJI,SA_cnt,1000));
								}
								
								if(SA_cnt>=1000&&flags[3]){
									
										lilState++;
										SA_cnt=0;
									
								}
								break;
							case 1:
								if(SA_cnt<200){
									gravityCompensation_SA_state=0;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SPIT;
									if(SA_cnt<50){
										AirOperaterCtrlBuf[5]=BIG_SPIT;
									}else{
										AirOperaterCtrlBuf[5]=BIG_SUCK;
									}
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFD_LK);
										stretch_DM.ctrl.pos_set=GKFD_DM;
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,GKFU_LK);
										stretch_DM.ctrl.pos_set=GKFU_DM-0.15f;
									}
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,GKFU_DJI);
									SA_cnt++;
								}else if(SA_cnt>=200){
									SA_cnt++;
									pushAndPull(PUMP_STOP,0);
									AirOperaterCtrlBuf[5]=BIG_SUCK;
									if(height_flag==DOWN_KFS){
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(GKFD_LK,RTGK_LK,SA_cnt-200,500));
										stretch_DM.ctrl.pos_set=rampSignalFP(GKFD_DM,RTGK_DM,SA_cnt-200,400);
									}else{
										flags[2]=LK_Pos_CtrlWithoutPID(&stretch_LK,rampSignalFP(GKFD_LK,RTGK_LK,SA_cnt-200,500));
										stretch_DM.ctrl.pos_set=rampSignalFP(GKFD_DM,RTGK_DM,SA_cnt-200,400);
									}
									flags[3]=DJI_Pos_CtrlWithoutPID(&stretch_2006,rampSignalFP(GKFD_DJI,RTGK_DJI,SA_cnt-200,400));
									if(SA_cnt>600&&flags[3]){
										
											lilState=0;
											SA_cnt=0;
											machineState.SA.SA_State = READY_TO_GET_KFS;
											machineState.SA.Cplt_State = CPLT;
											feedback_SA=50;
										
									}
								}
								break;
							default:
								break;
						}							
						break;
					default:
						machineState.DA.Cplt_State=CPLT;
						break;
				}
			}
			break;
	}
	}
}

void stateMachineTest(void){
	switch(action_DA){
		case 0:
			//复位
			//若复位完成，令action_DA=1；
			break;
		case 1:
			//准备取武器
			//若准备完成，令action_DA=2
			break;
		case 2:
			//取武器
			//若取完武器，令action_DA=3
			break;
		case 3:
			//保持动作一段时间后，令action_DA=0；
			break;
		default:
			//复位
			//若复位完成，令action_DA=0；
			break;
	}
}
