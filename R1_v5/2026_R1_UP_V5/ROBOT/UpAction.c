#include "UpAction.h"
#include "MathAlgorithm.h"
#include "tim.h"
u8 lock=0;

u32	sm_cnt1_1=0,sm_cnt1_2=0,sm_cnt1_3=0,sm_cnt1_4=0,sm_cnt1_5=0,sm_cnt1_6=0,sm_cnt1_7=0;
u32	sm_cnt2_1=0,sm_cnt2_2=0,sm_cnt2_3=0,sm_cnt2_4=0,sm_cnt2_5=0,sm_cnt2_6=0,sm_cnt2_7=0,sm_cnt2_8=0;
u32 sm_cnt3_1=0,sm_cnt3_2=0,sm_cnt3_3=0;

void UpAction_weapon(void){
    if(weapon_cplt==0){
        switch(weapon_player_action){
            case INIT_ACTION:
                init_get_weapon();
                break;
						case DEINIT://DEINIT:
								deinit_get_weapon();
								break;
            case GET_WEAPON:
                get_weapon();
                break;
            case STORE_WEAPON:
                store_weapon();
                break;
            case COMBINE_WEAPON:
                combine_weapon();
                break;
            case USE_WEAPON:
                use_weapon();
                break;
						case MOVE_WEAPON_OUT:
								move_weapon_out();
								break;
            default:
                break;
        }
    }else if(weapon_cplt==1){
				if(weapon_player_action!=KEEP_QUIET){
						sm_cnt1_1=0;
						sm_cnt1_2=0;
						sm_cnt1_3=0;
						sm_cnt1_4=0;
						sm_cnt1_5=0;
						sm_cnt1_6=0;
						weapon_cplt=0;
				}
		}
}

void init_get_weapon(void){
		sm_cnt1_1++;
		QD_show_1=0;
		airOperator.airOperatorTxBuf[0]=OPEN_0;//夹爪张开
    airOperator.airOperatorTxBuf[1]=OPEN_1;//轨道打开
		airOperator.airOperatorTxBuf[9]=OPEN_2;//轨道打开
		move2006_mode=0;
		claw3508.outerTarget=INIT_C3508;
    move2006.outerTarget=INIT_M2006;
    friction3508.innerTarget=INIT_F3508;
		controlPushPull(PULL_PIN);
    if(sm_cnt1_1>=300){
			controlPushPull(DEFAULT_PIN);
			weapon_player_state=INIT;
      sm_cnt1_1=0;
      weapon_cplt=1;
			weapon_player_action=KEEP_QUIET;
    }
}

void deinit_get_weapon(void){
	sm_cnt1_2++;
	airOperator.airOperatorTxBuf[0]=OPEN_0;//夹爪张开
  airOperator.airOperatorTxBuf[1]=OPEN_1;//轨道打开
	airOperator.airOperatorTxBuf[9]=OPEN_2;//轨道打开

  claw3508.outerTarget=STANDBY_C3508;
  move2006.outerTarget=STANDBY_M2006;
  friction3508.innerTarget=INIT_F3508;
  if(sm_cnt1_2>=1000){
    weapon_player_state=STAND_BY;
    sm_cnt1_2=0;
    weapon_cplt=1;
		weapon_player_action=KEEP_QUIET;
  }
}

void get_weapon(void){
	if(sm_cnt1_3<300){
		sm_cnt1_3++;
		move2006_mode=0;
		airOperator.airOperatorTxBuf[0]=OPEN_0;
		airOperator.airOperatorTxBuf[1]=OPEN_1;
    move2006.outerTarget=INIT_M2006;
    friction3508.innerTarget=0;
    claw3508.outerTarget=GET_WEAPON_C3508;
		
  }else if(sm_cnt1_3>=300&&sm_cnt1_3<800){
		if(complete_weapon==1){
      sm_cnt1_3++;
			shoulderJ60CMD.position_=INIT_SJ60+(-0.9f-INIT_SJ60)*rampSignalFP(sm_cnt1_3-300,500);
      airOperator.airOperatorTxBuf[0]=CLOSE_0;//夹爪夹住
			if(sm_cnt1_3>500) weapon_compensation_mode=1;
			claw3508.outerTarget=GET_WEAPON_C3508+(-GET_WEAPON_C3508+GRAB_HALFWEAPON_C3508)*rampSignalFP(sm_cnt1_3-500,300);
      }
  }else if(sm_cnt1_3>=800){
		claw_mode=0;
		weapon_player_state=GRAB_HALFWEAPON;
		complete_weapon=0;
    weapon_cplt=1;
    sm_cnt1_3=0;
		weapon_player_action=KEEP_QUIET;
	}
}

void store_weapon(void){
	
	move2006.outerTarget=INIT_M2006;
  if(sm_cnt1_4<1000){
		sm_cnt1_4++;
		claw3508.outerTarget=GRAB_HALFWEAPON_C3508+(-GRAB_HALFWEAPON_C3508+STORE_WEAPON_C3508+2.f)*rampSignalFP(sm_cnt1_4-200,700); 
  }else if(sm_cnt1_4>=1000&&sm_cnt1_4<1050){
//		if(complete_weapon){
//			sm_cnt1_4++;
//		}
		sm_cnt1_4++;
		airOperator.airOperatorTxBuf[1]=CLOSE_1;//轨道夹住
//		if(sm_cnt1_4==1200){
//			complete_weapon=0;
//		}
	}else if(sm_cnt1_4>=1050&&sm_cnt1_4<1100){
		sm_cnt1_4++;
		airOperator.airOperatorTxBuf[0]=OPEN_0;//夹爪张开
	}else if(sm_cnt1_4>=1100){
		weapon_compensation_mode=0;
		claw_mode=0;
    weapon_player_state=HOLD_HALFWEAPON;
		complete_weapon=0;
    weapon_cplt=1;
    sm_cnt1_4=0;
		weapon_player_action=KEEP_QUIET;
  }
}

void combine_weapon(void){
	
	if(sm_cnt1_5<=100){
		move2006_mode = 1;
		claw3508.motor_pid.outer.fpKp=2.f;
		claw3508.outerTarget=STANDBY_C3508;
		
		sm_cnt1_5++;
	}else if(sm_cnt1_5>100&&sm_cnt1_5<=1100){
		if(complete_weapon){
			shoulderJ60CMD.position_=-0.9f+(INIT_SJ60+0.9f)*rampSignalFP(sm_cnt1_5-100,1000);
			QD_show_1=1;
			sm_cnt1_5++;
		}
	}else{
		sm_cnt1_5=0;	
		complete_weapon=0;
		move2006.outerTarget=move2006.angle;
    move2006_mode=0;
    weapon_player_state=HOLD_WEAPON;
    weapon_cplt=1;
		weapon_player_action=KEEP_QUIET;
  }
}
fp32 temp_angle_c=0;
void move_weapon_out(void){
	static fp32 temp_rad=0;
	if(sm_cnt1_6<=300){
		move2006_mode = 0;
		move2006.outerTarget=INIT_M2006;
		move2006.motor_pid.outer.fpKp=2.5f;
		airOperator.airOperatorTxBuf[0]=OPEN_0;
		temp_rad=shoulderJ60.position_;
		temp_angle_c=claw3508.angle;
		sm_cnt1_6++;
	}else if(sm_cnt1_6>300&&sm_cnt1_6<=800){
		sm_cnt1_6++;
		
		claw3508.outerTarget=temp_angle_c+(180.f-temp_angle_c)*rampSignalFP(sm_cnt1_6-300,300);
		if(sm_cnt1_6>700){
			airOperator.airOperatorTxBuf[0]=CLOSE_0;
		}
		
	}else if(sm_cnt1_6>800&&sm_cnt1_6<=2500){
		sm_cnt1_6++;
		
		if(sm_cnt1_6>1700){
			airOperator.airOperatorTxBuf[1]=OPEN_1;
			controlPushPull(DEFAULT_PIN);
		}else{
			controlPushPull(PUSH_PIN);
		}
		shoulderJ60CMD.position_=temp_rad+(-0.9f-temp_rad)*rampSignalFP(sm_cnt1_6-1700,300);
		claw3508.outerTarget=STORE_WEAPON_C3508+(GRAB_HALFWEAPON_C3508-STORE_WEAPON_C3508)*rampSignalFP(sm_cnt1_6-1700,800);
	}else{
		sm_cnt1_6=0;	
		claw_mode=1;
		complete_weapon=0;
    weapon_player_state=HOLD_WEAPON;
    weapon_cplt=1;
		weapon_player_action=KEEP_QUIET;
  }
}

void use_weapon(void){
	sm_cnt1_7++;
	controlPushPull(PULL_PIN);
	if(sm_cnt1_7>1000){
		sm_cnt1_7=0;
		controlPushPull(DEFAULT_PIN);
		airOperator.airOperatorTxBuf[0]=OPEN_0;//夹爪夹住
		complete_weapon=0; 
		claw_mode=0;
		claw3508.outerTarget=0;
    weapon_player_state=STAND_BY;
    weapon_cplt=1;
		weapon_player_action=KEEP_QUIET;
	}
}

fp32 disCheck[2][12]={{2211,2649,5222,3435,0,4031,4580,0,2887,5787,2665,1634},
											{130,80,130,80,0,70,130.f,0,130,80,130,80}};
u8 checkPosition(void){
	u8 result=0;
	fp32 dis[2]={0};
	switch(ii_robot.now_KFS_ID){
		case 2:
		case 11:
			dis[0]=fabsf(airOperator.dt35_right-disCheck[0][ii_robot.now_KFS_ID-1]);
			dis[1]=fabsf(airOperator.dt35_front-disCheck[1][ii_robot.now_KFS_ID-1]);
			if(dis[0]<600){
				if(dis[0]<60){
					if(dis[1]<50){
						result=2;
					}else{
						result=1;
					}
				}else{
					result=1;
				}
			}
			break;
		default:
			dis[0]=fabsf(airOperator.dt35_left-disCheck[0][ii_robot.now_KFS_ID-1]);
			dis[1]=fabsf(airOperator.dt35_front-disCheck[1][ii_robot.now_KFS_ID-1]);
			if(dis[0]<600){
				if(dis[0]<100){
					if(dis[1]<100){
						result=2;
					}else{
						result=1;
					}
				}else{
					result=1;
				}
			}
			break;
	}
	return result;
}

void UpAction_KFS(void){
		if(KFS_cplt==0){
				switch(KFS_master_action){
            case INIT_ACTION:
                init_KFS();
                break;
						case DEINIT:
								deinit_KFS();
								break;
						case GET_AND_STORE:
								get_and_store_KFS();
								break;
          	case GET_KFS:
                get_KFS();
                break;
            case GET_STORED_KFS:
                get_stored_KFS();
                break;
            case STORE_KFS:
                store_KFS();
                break;
            case USE_KFS:
                use_KFS();
                break;
            case HANDOVER_KFS:
								handover_KFS();
                break;
            default:
                break;
        }
    }else if(KFS_cplt==1){
				if(KFS_master_action!=KEEP_QUIET){
						sm_cnt2_1=0;
						sm_cnt2_2=0;
						sm_cnt2_3=0;
						sm_cnt2_4=0;
						sm_cnt2_5=0;
						sm_cnt2_6=0;
						sm_cnt2_7=0;
						sm_cnt2_8=0;
						KFS_cplt=0;
				}				 
		}
}
void init_KFS(void){
	sm_cnt2_1++;
  airOperator.airOperatorTxBuf[2]=STOP_MAIN;//pump not work
  airOperator.airOperatorTxBuf[3]=SUCK_MAIN;//relate to pump
  airOperator.airOperatorTxBuf[4]=STOP_SUB_0;//pump not work
  airOperator.airOperatorTxBuf[5]=1;//relate to bottle
  airOperator.airOperatorTxBuf[6]=0;//pump not work
  airOperator.airOperatorTxBuf[7]=1;//relate to bottle
	airOperator.airOperatorTxBuf[8]=0;//pump not work
	shoulderJ60CMD.kp_=40;
	elbowJ60CMD.kp_=40;
	wrist3508.motor_pid.outer.fpKp=2;
  shoulderJ60CMD.position_=INIT_SJ60;
	elbowJ60CMD.position_=INIT_EJ60;
	wrist3508.outerTarget=INIT_W3508;
  gimbalJ60CMD.position_=INIT_GJ60;
	use_KFS_orientaton=1;
	if(sm_cnt2_1>510){
		shoulderJ60CMD.kp_=140;
		elbowJ60CMD.kp_=140;
		wrist3508.motor_pid.outer.fpKp=6;
		sm_cnt2_1=0;
		KFS_cplt=1;
		KFS_master_state=INIT;
		KFS_master_action=KEEP_QUIET;
	}
}

void deinit_KFS(void){
	sm_cnt2_2++;
	airOperator.airOperatorTxBuf[2]=STOP_MAIN;//pump not work
  airOperator.airOperatorTxBuf[3]=SUCK_MAIN;//relate to pump
  airOperator.airOperatorTxBuf[4]=STOP_SUB_0;//pump not work
  airOperator.airOperatorTxBuf[5]=SUCK_SUB_0;//relate to pump
  airOperator.airOperatorTxBuf[6]=STOP_SUB_1;//pump not work
  airOperator.airOperatorTxBuf[7]=SUCK_SUB_1;//relate to pump
	shoulderJ60CMD.position_=STANDBY_SJ60;
	elbowJ60CMD.position_=STANDBY_EJ60;
	wrist3508.outerTarget=STANDBY_W3508;
	gimbalJ60CMD.position_=INIT_GJ60;
	if(sm_cnt2_2>1000){
		sm_cnt2_2=0;
		KFS_cplt=1;
		KFS_master_state=STAND_BY;
		KFS_master_action=KEEP_QUIET;
	}
}

void get_and_store_KFS(void){
	move2006.outerTarget=STANDBY_M2006;
	if(sm_cnt2_3<500){
    sm_cnt2_3++;
		airOperator.airOperatorTxBuf[2]=RUN_MAIN;//pump not work
    if(get_KFS_height==0){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.78f-INIT_SJ60)*rampSignalFP(sm_cnt2_3,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.5f-INIT_EJ60)*rampSignalFP(sm_cnt2_3,500);
        wrist3508.outerTarget=INIT_W3508+(GET_GROUND_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_3,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==1){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.88f-INIT_SJ60)*rampSignalFP(sm_cnt2_3,500);
        elbowJ60CMD.position_=INIT_EJ60+(1.15f-INIT_EJ60)*rampSignalFP(sm_cnt2_3,500);
        wrist3508.outerTarget=INIT_W3508+(GET_LOW_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_3,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==2){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.43f-INIT_SJ60)*rampSignalFP(sm_cnt2_3,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.707f-INIT_EJ60)*rampSignalFP(sm_cnt2_3,500);
        wrist3508.outerTarget=INIT_W3508+(GET_HIGH_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_3,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==3){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.16f-INIT_SJ60)*rampSignalFP(sm_cnt2_3,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.23f-INIT_EJ60)*rampSignalFP(sm_cnt2_3,500);
        wrist3508.outerTarget=INIT_W3508+(GET_TOP_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_3,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }
	}else if(sm_cnt2_3>=500&&sm_cnt2_3<700){
//    if(complete_KFS==1||airOperator.dt35_left>2060){
			if(complete_KFS||get_allowed){
        sm_cnt2_3++;
        if(get_KFS_height==0){
            shoulderJ60CMD.position_=-1.78f+(GET_GROUND_KFS_SJ60+1.78f)*rampSignalFP_1(sm_cnt2_3-500,100);
            elbowJ60CMD.position_=0.5f+(GET_GROUND_KFS_EJ60-0.5f)*rampSignalFP_1(sm_cnt2_3-500,100);
            wrist3508.outerTarget=GET_GROUND_KFS_W3508;
        }else if(get_KFS_height==1){
            shoulderJ60CMD.position_=-1.88f+(GET_LOW_KFS_SJ60+1.88f)*rampSignalFP_1(sm_cnt2_3-500,100);
            elbowJ60CMD.position_=1.15f+(GET_LOW_KFS_EJ60-1.15f)*rampSignalFP_1(sm_cnt2_3-500,100);
            wrist3508.outerTarget=GET_LOW_KFS_W3508;
        }else if(get_KFS_height==2){
            shoulderJ60CMD.position_=-1.43f+(GET_HIGH_KFS_SJ60+1.43f)*rampSignalFP_1(sm_cnt2_3-500,100);
            elbowJ60CMD.position_=0.71f+(GET_HIGH_KFS_EJ60-0.71f)*rampSignalFP_1(sm_cnt2_3-500,100);
            wrist3508.outerTarget=GET_HIGH_KFS_W3508;
        }else if(get_KFS_height==3){
            shoulderJ60CMD.position_=-1.16f+(GET_TOP_KFS_SJ60+1.16f)*rampSignalFP_1(sm_cnt2_3-500,100);
            elbowJ60CMD.position_=0.23f+(GET_TOP_KFS_EJ60-0.18f)*rampSignalFP_1(sm_cnt2_3-500,100);
            wrist3508.outerTarget=GET_TOP_KFS_W3508;
        }
    }
	}else if(sm_cnt2_3>=700&&sm_cnt2_3<1660){
    sm_cnt2_3++;
		KFS_compensation_mode=1;
    if(sm_cnt2_3>1000){
        chassis_move=1;
    }
		gimbalJ60CMD.kp_=80;
    if(get_KFS_height==0){
        shoulderJ60CMD.position_=GET_GROUND_KFS_SJ60+(-0.14f-GET_GROUND_KFS_SJ60)*rampSignalFP(sm_cnt2_3-700,720);
        elbowJ60CMD.position_=GET_GROUND_KFS_EJ60+(-0.32f-GET_GROUND_KFS_EJ60)*rampSignalFP(sm_cnt2_3-700,720);
        wrist3508.outerTarget=GET_GROUND_KFS_W3508+(-185.f-GET_GROUND_KFS_W3508)*rampSignalFP(sm_cnt2_3-700,720);
				if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
					gimbalJ60CMD.position_=INIT_GJ60+(1.94f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}else{
					gimbalJ60CMD.position_=INIT_GJ60+(-1.9f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}
    }else if(get_KFS_height==1){
        shoulderJ60CMD.position_=GET_LOW_KFS_SJ60+(-0.14f-GET_LOW_KFS_SJ60)*rampSignalFP(sm_cnt2_3-700,720);
        elbowJ60CMD.position_=GET_LOW_KFS_EJ60+(-0.32f-GET_LOW_KFS_EJ60)*rampSignalFP(sm_cnt2_3-700,720);
        wrist3508.outerTarget=GET_LOW_KFS_W3508+(-185.f-GET_LOW_KFS_W3508)*rampSignalFP(sm_cnt2_3-700,720);
				if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
					gimbalJ60CMD.position_=INIT_GJ60+(1.94f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}else{
					gimbalJ60CMD.position_=INIT_GJ60+(-1.9f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}
    }else if(get_KFS_height==2){
        shoulderJ60CMD.position_=GET_HIGH_KFS_SJ60+(-0.14f-GET_HIGH_KFS_SJ60)*rampSignalFP(sm_cnt2_3-700,640);
        elbowJ60CMD.position_=GET_HIGH_KFS_EJ60+(-0.33f-GET_HIGH_KFS_EJ60)*rampSignalFP(sm_cnt2_3-700,480);
        wrist3508.outerTarget=GET_HIGH_KFS_W3508+(-185.f-GET_HIGH_KFS_W3508)*rampSignalFP(sm_cnt2_3-700,560);
				if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
					gimbalJ60CMD.position_=INIT_GJ60+(1.94f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}else{
					gimbalJ60CMD.position_=INIT_GJ60+(-1.9f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}
    }else if(get_KFS_height==3){
        shoulderJ60CMD.position_=GET_TOP_KFS_SJ60+(-0.14f-GET_TOP_KFS_SJ60)*rampSignalFP(sm_cnt2_3-1020,560);
        elbowJ60CMD.position_=GET_TOP_KFS_EJ60+(-0.33f-GET_TOP_KFS_EJ60)*rampSignalFP(sm_cnt2_3-1020,560);
        wrist3508.outerTarget=GET_TOP_KFS_W3508+(-185.f-GET_TOP_KFS_W3508)*rampSignalFP(sm_cnt2_3-700,560);
				if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
					gimbalJ60CMD.position_=INIT_GJ60+(1.94f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}else{
					gimbalJ60CMD.position_=INIT_GJ60+(-1.9f-INIT_GJ60)*rampSignalFP(sm_cnt2_3-860,800);
				}
    } 
	}else if(sm_cnt2_3>=1660&&sm_cnt2_3<1980){
			sm_cnt2_3++;
//			gimbalJ60CMD.kp_=60;
			gimbalJ60CMD.kp_=320;
			shoulderJ60CMD.position_=-0.14f+(-0.18f+0.14f)*rampSignalFP(sm_cnt2_3-1660,160);
			elbowJ60CMD.position_=-0.33f+(-0.06f+0.33f)*rampSignalFP(sm_cnt2_3-1660,160);
			wrist3508.outerTarget=-185.f;
			if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				gimbalJ60CMD.position_=1.94f+(2.56f-1.94f)*rampSignalFP(sm_cnt2_3-1660,320);
			}else{
				gimbalJ60CMD.position_=-1.9f+(-2.52f+1.9f)*rampSignalFP(sm_cnt2_3-1660,320);
			}
			if(sm_cnt2_3>1880){
				if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				airOperator.airOperatorTxBuf[6]=1;
			}else{
				airOperator.airOperatorTxBuf[8]=1;
			}
			}
	}else if(sm_cnt2_3>=1980&&sm_cnt2_3<2380){
		sm_cnt2_3++;
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				airOperator.airOperatorTxBuf[4]=1;
				airOperator.airOperatorTxBuf[5]=0;
			}else{
				airOperator.airOperatorTxBuf[4]=1;
				airOperator.airOperatorTxBuf[7]=0;
			}
		if(sm_cnt2_3>2250){
			airOperator.airOperatorTxBuf[2]=0;
			airOperator.airOperatorTxBuf[3]=1;
		}
	}else if(sm_cnt2_3>=2380&&sm_cnt2_3<2480){
		gimbalJ60CMD.kp_=320;
		sm_cnt2_3++;
		KFS_compensation_mode=0;
		airOperator.airOperatorTxBuf[3]=0;
		
		shoulderJ60CMD.position_=-0.18f+(-0.04f+0.18f)*rampSignalFP(sm_cnt2_3-2380,100);		
    elbowJ60CMD.position_=-0.27f+(-0.44f+0.27f)*rampSignalFP(sm_cnt2_3-2380,100);
		wrist3508.outerTarget=-185.f;
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=2.54f+(-2.54f+2.18f)*rampSignalFP(sm_cnt2_3-2400,80);
		}else{
			gimbalJ60CMD.position_=-2.46f+(2.46f-2.15f)*rampSignalFP(sm_cnt2_3-2400,80);
		}
	}else if(sm_cnt2_3>=2480&&sm_cnt2_3<2730){
		sm_cnt2_3++;
		
		shoulderJ60CMD.position_=-0.04f+(-0.61f+0.04f)*rampSignalFP(sm_cnt2_3-2480,250);		
    elbowJ60CMD.position_=-0.44f+(-0.19f+0.44f)*rampSignalFP(sm_cnt2_3-2480,250);
		wrist3508.outerTarget=-185.f+145*rampSignalFP(sm_cnt2_3-2480,250);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=2.18f+(1.85f-2.18f)*rampSignalFP(sm_cnt2_3-2480,250);
		}else{
			gimbalJ60CMD.position_=-2.15f+(-1.8f+2.15f)*rampSignalFP(sm_cnt2_3-2480,250);
		}
	}else if(sm_cnt2_3>=2730&&sm_cnt2_3<3030){
		sm_cnt2_3++;
		
		shoulderJ60CMD.position_=-0.61f+(INIT_SJ60+0.61f)*rampSignalFP(sm_cnt2_3-2730,300);		
    elbowJ60CMD.position_=-0.19f+(INIT_EJ60+0.19f)*rampSignalFP(sm_cnt2_3-2730,300);
		wrist3508.outerTarget=-40.f-145*rampSignalFP(sm_cnt2_3-2730,300);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=1.85f+(INIT_GJ60-1.85f)*rampSignalFP_1(sm_cnt2_3-2730,300);
		}else{
			gimbalJ60CMD.position_=-1.8f+(INIT_GJ60+1.8f)*rampSignalFP_1(sm_cnt2_3-2730,300);
		}
	}else if(sm_cnt2_3>=3030){
    KFS_cplt=1;
    complete_KFS=0;
		KFS_master_action=KEEP_QUIET;
    KFS_master_state=INIT;
    sm_cnt2_3=0;
		get_allowed=0;
    chassis_move=0;
		KFS_master_action=KEEP_QUIET;
	}
}
void get_KFS(void){
	move2006.outerTarget=STANDBY_M2006;
	if(sm_cnt2_4<500){
    sm_cnt2_4++;
		airOperator.airOperatorTxBuf[2]=RUN_MAIN;//pump not work
    if(get_KFS_height==0){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.61f-INIT_SJ60)*rampSignalFP(sm_cnt2_4,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.5f-INIT_EJ60)*rampSignalFP(sm_cnt2_4,500);
        wrist3508.outerTarget=INIT_W3508+(GET_GROUND_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_4,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==1){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.88f-INIT_SJ60)*rampSignalFP(sm_cnt2_4,500);
        elbowJ60CMD.position_=INIT_EJ60+(1.12f-INIT_EJ60)*rampSignalFP(sm_cnt2_4,500);
        wrist3508.outerTarget=INIT_W3508+(GET_LOW_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_4,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==2){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.43f-INIT_SJ60)*rampSignalFP(sm_cnt2_4,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.657f-INIT_EJ60)*rampSignalFP(sm_cnt2_4,500);
        wrist3508.outerTarget=INIT_W3508+(GET_HIGH_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_4,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }else if(get_KFS_height==3){
        shoulderJ60CMD.position_=INIT_SJ60+(-1.16f-INIT_SJ60)*rampSignalFP(sm_cnt2_4,500);
        elbowJ60CMD.position_=INIT_EJ60+(0.23f-INIT_EJ60)*rampSignalFP(sm_cnt2_4,500);
        wrist3508.outerTarget=INIT_W3508+(GET_TOP_KFS_W3508-INIT_W3508)*rampSignalFP(sm_cnt2_4,500);
        gimbalJ60CMD.position_=INIT_GJ60;
    }
	}else if(sm_cnt2_4>=500&&sm_cnt2_4<800){
//    if(complete_KFS==1||airOperator.dt35_left>2060){
			if(complete_KFS||get_allowed){
        sm_cnt2_4++;
        if(get_KFS_height==0){
            shoulderJ60CMD.position_=-1.61f+(GET_GROUND_KFS_SJ60+1.61f)*rampSignalFP_1(sm_cnt2_4-500,100);
            elbowJ60CMD.position_=0.5f+(GET_GROUND_KFS_EJ60-0.5f)*rampSignalFP_1(sm_cnt2_4-500,100);
            wrist3508.outerTarget=GET_GROUND_KFS_W3508;
        }else if(get_KFS_height==1){
            shoulderJ60CMD.position_=-1.88f+(GET_LOW_KFS_SJ60+1.88f)*rampSignalFP_1(sm_cnt2_4-500,100);
            elbowJ60CMD.position_=1.12f+(GET_LOW_KFS_EJ60-1.12f)*rampSignalFP_1(sm_cnt2_4-500,100);
            wrist3508.outerTarget=GET_LOW_KFS_W3508;
        }else if(get_KFS_height==2){
            shoulderJ60CMD.position_=-1.43f+(GET_HIGH_KFS_SJ60+1.43f)*rampSignalFP_1(sm_cnt2_4-500,100);
            elbowJ60CMD.position_=0.66f+(GET_HIGH_KFS_EJ60-0.66f)*rampSignalFP_1(sm_cnt2_4-500,100);
            wrist3508.outerTarget=GET_HIGH_KFS_W3508;
        }else if(get_KFS_height==3){
            shoulderJ60CMD.position_=-1.16f+(GET_TOP_KFS_SJ60+1.16f)*rampSignalFP_1(sm_cnt2_4-500,100);
            elbowJ60CMD.position_=0.18f+(GET_TOP_KFS_EJ60-0.18f)*rampSignalFP_1(sm_cnt2_4-500,100);
            wrist3508.outerTarget=GET_TOP_KFS_W3508;
        }
    }
	}else if(sm_cnt2_4>=800&&sm_cnt2_4<2000){
		sm_cnt2_4++;
		KFS_compensation_mode=1;
    if(sm_cnt2_4>1000){
      chassis_move=1;
    }
    if(get_KFS_height==0){
			shoulderJ60CMD.position_=GET_GROUND_KFS_SJ60+(HOLD_KFS_SJ60-GET_GROUND_KFS_SJ60)*rampSignalFP(sm_cnt2_4-800,1200);
      elbowJ60CMD.position_=GET_GROUND_KFS_EJ60+(HOLD_KFS_EJ60-GET_GROUND_KFS_EJ60)*rampSignalFP(sm_cnt2_4-800,1200);
      wrist3508.outerTarget=GET_GROUND_KFS_W3508+(HOLD_KFS_W3508-GET_GROUND_KFS_W3508)*rampSignalFP(sm_cnt2_4-1000,1000);
//			gimbalJ60CMD.position_=-1.55f+(INIT_GJ60+1.55f)*rampSignalFP(sm_cnt2_4-1000,1000);
    }else if(get_KFS_height==1){
      shoulderJ60CMD.position_=GET_LOW_KFS_SJ60+(HOLD_KFS_SJ60-GET_LOW_KFS_SJ60)*rampSignalFP(sm_cnt2_4-800,600);
      elbowJ60CMD.position_=GET_LOW_KFS_EJ60+(HOLD_KFS_EJ60-GET_LOW_KFS_EJ60)*rampSignalFP(sm_cnt2_4-800,600);
      wrist3508.outerTarget=GET_LOW_KFS_W3508+(HOLD_KFS_W3508-GET_LOW_KFS_W3508)*rampSignalFP(sm_cnt2_4-800,600);
			if(sm_cnt2_4>1400){
				sm_cnt2_4=2000;
			}
    }else if(get_KFS_height==2){
      shoulderJ60CMD.position_=GET_HIGH_KFS_SJ60+(HOLD_KFS_SJ60-GET_HIGH_KFS_SJ60)*rampSignalFP(sm_cnt2_4-800,600);
      elbowJ60CMD.position_=GET_HIGH_KFS_EJ60+(HOLD_KFS_EJ60-GET_HIGH_KFS_EJ60)*rampSignalFP(sm_cnt2_4-800,600);
      wrist3508.outerTarget=GET_HIGH_KFS_W3508+(HOLD_KFS_W3508-GET_HIGH_KFS_W3508)*rampSignalFP(sm_cnt2_4-800,600);
			if(sm_cnt2_4>1400){
				sm_cnt2_4=2000;
			}
    }else if(get_KFS_height==3){
      shoulderJ60CMD.position_=GET_TOP_KFS_SJ60+(HOLD_KFS_SJ60-GET_TOP_KFS_SJ60)*rampSignalFP(sm_cnt2_4-800,600);
      elbowJ60CMD.position_=GET_TOP_KFS_EJ60+(HOLD_KFS_EJ60-GET_TOP_KFS_EJ60)*rampSignalFP(sm_cnt2_4-800,600);
      wrist3508.outerTarget=GET_TOP_KFS_W3508+(HOLD_KFS_W3508-GET_TOP_KFS_W3508)*rampSignalFP(sm_cnt2_4-800,600);
			if(sm_cnt2_4>1400){
				sm_cnt2_4=2000;
			}
    }
  }else if(sm_cnt2_4>=2000){
		shoulderJ60CMD.kp_=160;
		elbowJ60CMD.kp_=160;
    KFS_cplt=1;
    complete_KFS=0;
    KFS_master_state=HOLD_KFS;
    sm_cnt2_4=0;
    chassis_move=0;
		KFS_master_action=KEEP_QUIET;
  }
}
void get_stored_KFS(void){
	sm_cnt2_5++;
  if(sm_cnt2_5<500){
		shoulderJ60CMD.position_=INIT_SJ60+(-0.63f-INIT_SJ60)*rampSignalFP(sm_cnt2_5,500);
		elbowJ60CMD.position_=INIT_EJ60+(0.59f-INIT_EJ60)*rampSignalFP(sm_cnt2_5,500);
    wrist3508.outerTarget=INIT_W3508+(-25.f-INIT_W3508)*rampSignalFP(sm_cnt2_5,500);
    if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=INIT_GJ60+(1.54f-INIT_GJ60)*rampSignalFP(sm_cnt2_5,500);
    }else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
			gimbalJ60CMD.position_=INIT_GJ60+(-1.46f-INIT_GJ60)*rampSignalFP(sm_cnt2_5,500);
		}
  }else if(sm_cnt2_5>=500&&sm_cnt2_5<1000){
    shoulderJ60CMD.position_=-0.63f+(-0.03f+0.63f)*rampSignalFP(sm_cnt2_5-500,500);
		elbowJ60CMD.position_=0.59f+(-0.09f-0.59f)*rampSignalFP(sm_cnt2_5-500,500);
    wrist3508.outerTarget=-25.f+(-190.f+25.f)*rampSignalFP(sm_cnt2_5-750,250);
    if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=1.54f+(2.34f-1.54f)*rampSignalFP(sm_cnt2_5-500,500);
    }else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
			gimbalJ60CMD.position_=-1.46f+(-2.29f+1.46f)*rampSignalFP(sm_cnt2_5-500,500);
		}
  }else if(sm_cnt2_5>=1000&&sm_cnt2_5<1300){
    airOperator.airOperatorTxBuf[2]=RUN_MAIN;//pump work
    airOperator.airOperatorTxBuf[3]=SUCK_MAIN;//relate2pump
		
		shoulderJ60CMD.position_=-0.03f+(-0.35f+0.03f)*rampSignalFP(sm_cnt2_5-1000,200);
		elbowJ60CMD.position_=-0.09f;
    wrist3508.outerTarget=-190.f;
  }else if(sm_cnt2_5>=1300&&sm_cnt2_5<1500){
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
      airOperator.airOperatorTxBuf[5]=1;//relate to bottle
    }else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
      airOperator.airOperatorTxBuf[7]=1;//relate to bottle
    }
  }else if(sm_cnt2_5>=1500&&sm_cnt2_5<2100){
    KFS_compensation_mode=1;
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			airOperator.airOperatorTxBuf[6]=0;//吸盘停止通气
		}else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
			airOperator.airOperatorTxBuf[8]=0;//吸盘停止通气
		}
		
		shoulderJ60CMD.position_=-0.35f+(-0.63f+0.35f)*rampSignalFP(sm_cnt2_5-1600,500);
		elbowJ60CMD.position_=-0.03f+(HOLD_KFS_EJ60+0.03f)*rampSignalFP(sm_cnt2_5-1600,500);
		wrist3508.outerTarget=-190.f+(-100.f+190.f)*rampSignalFP(sm_cnt2_5-1700,400);
    if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
      gimbalJ60CMD.position_=2.34f+(1.8f-2.34f)*rampSignalFP(sm_cnt2_5-1600,500);
    }else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
      gimbalJ60CMD.position_=-2.29f+(-1.74f+2.29f)*rampSignalFP(sm_cnt2_5-1600,500);
    }
	}else if(sm_cnt2_5>=2100&&sm_cnt2_5<2800){
		shoulderJ60CMD.position_=-0.63f+(HOLD_KFS_SJ60+0.63f)*rampSignalFP(sm_cnt2_5-2100,700);
		elbowJ60CMD.position_=HOLD_KFS_EJ60;
		wrist3508.outerTarget=-100.f+(HOLD_KFS_W3508+100.f)*rampSignalFP(sm_cnt2_5-2100,700);
    if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
      gimbalJ60CMD.position_=1.8f+(INIT_GJ60-1.8f)*rampSignalFP(sm_cnt2_5-2100,700);
    }else if(store_KFS_orientation==STORE_AND_GET_STORED_LEFT){
      gimbalJ60CMD.position_=-1.74f+(INIT_GJ60+1.74f)*rampSignalFP(sm_cnt2_5-2100,700);
    }
	}else if(sm_cnt2_5>=2800){
    KFS_cplt=1;
    complete_KFS=0;
    KFS_master_state=HOLD_KFS;
    sm_cnt2_5=0;
		KFS_master_action=KEEP_QUIET;
  }
}
void store_KFS(void){
	if(sm_cnt2_6<1240){
//		if(sm_cnt2_6==0){
//			gimbalTD.x1=0;
//			gimbalTD.x2=0;
//			gimbalTD.x=0;
//			gimbalTD.r=500.f;
//			gimbalTD.h=0.1f;
//			if(use_KFS_orientaton==0){
//				gimbalTD.aim=INIT_GJ60+(1.88f-INIT_GJ60);
//			}else{
//				gimbalTD.aim=INIT_GJ60+(-1.84f-INIT_GJ60);
//			}
//		}
		sm_cnt2_6++;
		if(sm_cnt2_6<680){
//			gimbalJ60CMD.kp_=80;
//			CalTD(&gimbalTD);
//			gimbalJ60CMD.position_=gimbalTD.x1;
//			gimbalJ60CMD.velocity_=gimbalTD.x2;
			shoulderJ60CMD.position_=HOLD_KFS_SJ60+(-0.14f-HOLD_KFS_SJ60)*rampSignalFP(sm_cnt2_6-300,380);
			elbowJ60CMD.position_=HOLD_KFS_EJ60+(-0.25f-HOLD_KFS_EJ60)*rampSignalFP(sm_cnt2_6,680);
			wrist3508.outerTarget=HOLD_KFS_W3508+(-185.f-HOLD_KFS_W3508)*rampSignalFP(sm_cnt2_6,680);
			if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				gimbalJ60CMD.position_=INIT_GJ60+(1.84f-INIT_GJ60)*rampSignalFP(sm_cnt2_6,680);
			}else{
				gimbalJ60CMD.position_=INIT_GJ60+(-1.8f-INIT_GJ60)*rampSignalFP(sm_cnt2_6,680);
			}
		}else if(sm_cnt2_6>=680&&sm_cnt2_6<1240){
//			gimbalJ60.kp_=60;
			gimbalJ60CMD.velocity_=0;
			shoulderJ60CMD.position_=-0.14f+(-0.18f+0.14f)*rampSignalFP(sm_cnt2_6-680,120);
			elbowJ60CMD.position_=-0.25f+(-0.06f+0.25f)*rampSignalFP(sm_cnt2_6-680,120);
			wrist3508.outerTarget=-185.f;
			if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				gimbalJ60CMD.position_=1.88f+(2.54f-1.88f)*rampSignalFP(sm_cnt2_6-680,300);
			}else{
				gimbalJ60CMD.position_=-1.88f+(-2.54f+1.88f)*rampSignalFP(sm_cnt2_6-680,300);
			}
		}
		if(sm_cnt2_6>800&&sm_cnt2_6<900){
			if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				airOperator.airOperatorTxBuf[6]=1;//连吸盘
			}else{
				airOperator.airOperatorTxBuf[8]=1;//连吸盘
			}
		}
		if(sm_cnt2_6>900){
			if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
				airOperator.airOperatorTxBuf[4]=1;//开吸盘气泵
				airOperator.airOperatorTxBuf[5]=0;//连气泵
			}else{
				airOperator.airOperatorTxBuf[4]=1;//开吸盘气泵
				airOperator.airOperatorTxBuf[7]=0;//连气泵
			}
		}
		if(sm_cnt2_6>1080){
			airOperator.airOperatorTxBuf[2]=STOP_MAIN;//关机械臂气泵
			airOperator.airOperatorTxBuf[3]=SPIT_MAIN;//吐会气
		}
	}else if(sm_cnt2_6>=1240&&sm_cnt2_6<1400){
		gimbalJ60.kp_=60;
		sm_cnt2_6++;
		KFS_compensation_mode=0;
		airOperator.airOperatorTxBuf[3]=0;//停止吐气
		
		shoulderJ60CMD.position_=-0.35f+(-0.04f+0.35f)*rampSignalFP(sm_cnt2_6-1240,160);
		elbowJ60CMD.position_=-0.06f+(-0.44f+0.09f)*rampSignalFP(sm_cnt2_6-1240,160);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=2.54f+(2.16f-2.54f)*rampSignalFP(sm_cnt2_6-1240,160);
		}else{
			gimbalJ60CMD.position_=-2.54f+(-2.14f+2.54f)*rampSignalFP(sm_cnt2_6-1240,160);
		}
	}else if(sm_cnt2_6>=1400&&sm_cnt2_6<1660){
		sm_cnt2_6++;
		
		shoulderJ60CMD.position_=-0.04f+(-0.61f+0.04f)*rampSignalFP(sm_cnt2_6-1400,260);
		elbowJ60CMD.position_=-0.44f+(-0.19f+0.44f)*rampSignalFP(sm_cnt2_6-1400,260);
		wrist3508.outerTarget=-185.f+(-40.f+185.f)*rampSignalFP(sm_cnt2_6-1400,260);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=2.16f+(1.8f-2.16f)*rampSignalFP(sm_cnt2_6-1400,260);
		}else{
			gimbalJ60CMD.position_=-2.14f+(-1.75f+2.14f)*rampSignalFP(sm_cnt2_6-1400,260);
		}
	}else if(sm_cnt2_6>=1660&&sm_cnt2_6<1860){
		sm_cnt2_6++;
		
		shoulderJ60CMD.position_=-0.61f+(-0.8f+0.61f)*rampSignalFP(sm_cnt2_6-1660,200);
		elbowJ60CMD.position_=-0.19f+(0.37f+0.19f)*rampSignalFP(sm_cnt2_6-1660,200);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=1.8f+(1.4f-1.8f)*rampSignalFP(sm_cnt2_6-1660,200);
		}else{
			gimbalJ60CMD.position_=-1.75f+(-1.35f+1.75f)*rampSignalFP(sm_cnt2_6-1660,200);
		}
	}else if(sm_cnt2_6>=1860&&sm_cnt2_6<2160){
		sm_cnt2_6++;
		
		shoulderJ60CMD.position_=-0.8f+(INIT_SJ60+0.8f)*rampSignalFP(sm_cnt2_6-1860,300);
		elbowJ60CMD.position_=0.37f+(INIT_EJ60-0.37f)*rampSignalFP(sm_cnt2_6-1860,300);
		wrist3508.outerTarget=-40.f+(INIT_W3508+40.f)*rampSignalFP(sm_cnt2_6-1860,300);
		if(store_KFS_orientation==STORE_AND_GET_STORED_RIGHT){
			gimbalJ60CMD.position_=1.4f+(INIT_GJ60-1.4f)*rampSignalFP_1(sm_cnt2_6-1860,300);
		}else{
			gimbalJ60CMD.position_=-1.34f+(INIT_GJ60+1.34f)*rampSignalFP_1(sm_cnt2_6-1860,300);
		}
	}else if(sm_cnt2_6>=2160){
		gimbalJ60CMD.kp_=320;
		KFS_cplt=1;
    complete_KFS=0;
    KFS_master_state=INIT;
    sm_cnt2_6=0;
		KFS_master_action=KEEP_QUIET;
  }
}

fp32 test_angle=0;

ZVD_Shaper gimbalZVD={0};
fp32 wn=0,zeta=0;
void use_KFS(void){
	if(sm_cnt2_7<840){
		sm_cnt2_7++;
    shoulderJ60CMD.position_=HOLD_KFS_SJ60+(-0.78f-HOLD_KFS_SJ60)*rampSignalFP_1(sm_cnt2_7-300,500);
    elbowJ60CMD.position_=HOLD_KFS_EJ60+(0.53f-HOLD_KFS_EJ60)*rampSignalFP_1(sm_cnt2_7-300,500);
    wrist3508.outerTarget=HOLD_KFS_W3508+(USE_KFS_W3508-HOLD_KFS_W3508)*rampSignalFP_1(sm_cnt2_7,500);
		if(use_KFS_orientaton==USE_KFS_LEFT_OF_R1){
			gimbalJ60CMD.position_=INIT_GJ60+(USE_KFS_GJ60_L-INIT_GJ60)*curveSignalFP(sm_cnt2_7,840);
		}else{
			gimbalJ60CMD.position_=gimbalJ60CMD.position_=INIT_GJ60+(USE_KFS_GJ60_R-INIT_GJ60)*curveSignalFP(sm_cnt2_7,840);
		}
	}else if(sm_cnt2_7>=840&&sm_cnt2_7<1080){
		if(complete_KFS==1){
			sm_cnt2_7++;
			shoulderJ60CMD.position_=-0.78f+(USE_KFS_SJ60+0.78f)*rampSignalFP(sm_cnt2_7-840,240);
      elbowJ60CMD.position_=0.53f+(USE_KFS_EJ60-0.53f)*rampSignalFP(sm_cnt2_7-840,240);
      wrist3508.outerTarget=USE_KFS_W3508;
		}
		if(sm_cnt2_7==1080){
			complete_KFS=0;
		}
		
  }else if(sm_cnt2_7>=1080&&sm_cnt2_7<2140){
		if(complete_KFS){
			sm_cnt2_7++;
			if(sm_cnt2_7>=1080&&sm_cnt2_7<1220){
				airOperator.airOperatorTxBuf[2]=STOP_MAIN;//pump not work
				airOperator.airOperatorTxBuf[3]=SPIT_MAIN;//pump not work
				KFS_compensation_mode=0;
			}else{
				airOperator.airOperatorTxBuf[2]=STOP_MAIN;//pump not work
				airOperator.airOperatorTxBuf[3]=SUCK_MAIN;//pump not work
			}
			if(sm_cnt2_7>1300){
				chassis_move=1;
			}
			shoulderJ60CMD.position_=USE_KFS_SJ60+(INIT_SJ60-USE_KFS_SJ60)*rampSignalFP(sm_cnt2_7-1220,340);
			elbowJ60CMD.position_=USE_KFS_EJ60+(INIT_EJ60-USE_KFS_EJ60)*rampSignalFP(sm_cnt2_7-1220,340);
			wrist3508.outerTarget=USE_KFS_W3508+(INIT_W3508-USE_KFS_W3508)*rampSignalFP(sm_cnt2_7-1280,340);
			if(use_KFS_orientaton==USE_KFS_LEFT_OF_R1){
				gimbalJ60CMD.position_=USE_KFS_GJ60_L+(INIT_GJ60-USE_KFS_GJ60_L)*rampSignalFP(sm_cnt2_7-1520,640);
			}else{
				gimbalJ60CMD.position_=USE_KFS_GJ60_R+(INIT_GJ60-USE_KFS_GJ60_R)*rampSignalFP(sm_cnt2_7-1520,640);
			}
		}
  }else if(sm_cnt2_7>=2140){
		KFS_cplt=1;
		chassis_move=0;
    complete_KFS=0;
    KFS_master_state=INIT;
    sm_cnt2_7=0;
		KFS_master_action=KEEP_QUIET;
	}
}

void handover_KFS(void){
	if(sm_cnt2_8<500){
		sm_cnt2_8++;
    shoulderJ60CMD.position_=HOLD_KFS_SJ60+(HANDOVER_KFS_SJ60-HOLD_KFS_SJ60)*rampSignalFP(sm_cnt2_8,500);
    elbowJ60CMD.position_=HOLD_KFS_EJ60+(HANDOVER_KFS_EJ60-HOLD_KFS_EJ60)*rampSignalFP(sm_cnt2_8,500);
    wrist3508.outerTarget=HOLD_KFS_W3508+(HANDOVER_KFS_W3508-HOLD_KFS_W3508)*rampSignalFP(sm_cnt2_8,500);
		if(use_KFS_orientaton==HANDOVER_KFS_AT_LEFT_OF_R2){
			gimbalJ60CMD.position_=INIT_GJ60+(HANDOVER_KFS_GJ60-INIT_GJ60)*rampSignalFP(sm_cnt2_8-200,300);
		}else{
			gimbalJ60CMD.position_=INIT_GJ60+(HANDOVER_KFS_GJ60_RIGHT-INIT_GJ60)*rampSignalFP(sm_cnt2_8-200,300);
		}
  }else if(sm_cnt2_8>=500&&sm_cnt2_8<1000){
    if(complete_KFS==1){
      sm_cnt2_8++;
			if(sm_cnt2_8<700){
				airOperator.airOperatorTxBuf[2]=STOP_MAIN;
				airOperator.airOperatorTxBuf[3]=SPIT_MAIN;
			}else{
				airOperator.airOperatorTxBuf[3]=SUCK_MAIN;
				KFS_compensation_mode=0;
			}
			if(use_KFS_orientaton==HANDOVER_KFS_AT_LEFT_OF_R2){
				gimbalJ60CMD.position_=HANDOVER_KFS_GJ60+(-HANDOVER_KFS_GJ60+INIT_GJ60)*rampSignalFP(sm_cnt2_8-700,300);
			}else if(use_KFS_orientaton==HANDOVER_KFS_AT_RIGHT_OF_R2){
				gimbalJ60CMD.position_=HANDOVER_KFS_GJ60_RIGHT+(-HANDOVER_KFS_GJ60_RIGHT+INIT_GJ60)*rampSignalFP(sm_cnt2_8-700,300);
			}
      shoulderJ60CMD.position_=HANDOVER_KFS_SJ60+(INIT_SJ60-HANDOVER_KFS_SJ60)*rampSignalFP(sm_cnt2_8-700,300);
      elbowJ60CMD.position_=HANDOVER_KFS_EJ60+(INIT_EJ60-HANDOVER_KFS_EJ60)*rampSignalFP(sm_cnt2_8-700,300);
      wrist3508.outerTarget=HANDOVER_KFS_W3508+(INIT_W3508-HANDOVER_KFS_W3508)*rampSignalFP(sm_cnt2_8-700,300);
    }
	}else if(sm_cnt2_8>=1000){
		KFS_cplt=1;
    sm_cnt2_8=0;
    complete_KFS=0;
    KFS_master_state=INIT;
		KFS_master_action=KEEP_QUIET;
  }
}


void UpAction_platform(void){
    if(platform_cplt==0){
        switch(platform_action){
						case INIT_ACTION:
                init_platform();              
								break;
            case MOVEOUT:
								platform_move_out();
                break;
            case MOVEIN:
                platform_move_in();
                break;
            default:
                break;
        }
    }else if(platform_cplt==1){
        if(platform_action!=KEEP_QUIET){
						sm_cnt3_1=0;
						sm_cnt3_2=0;
						sm_cnt3_3=0;
            platform_cplt=0;
				}
    }
}

void init_platform(void){
	sm_cnt3_1++;
  if(sm_cnt3_1<=200){
		platform_L2006.innerTarget=-10;
		platform_R2006.innerTarget=10;
  }else{
		platform_L2006.innerTarget=-5;
		platform_R2006.innerTarget=5;
    platform_cplt=1;
    sm_cnt3_1=0;
    platform_state=IN;
		platform_action=KEEP_QUIET;
  }
}

void platform_move_in(void){
	sm_cnt3_2++;
	if(sm_cnt3_2<=1500){
    airOperator.airOperatorTxBuf[9]=PULL_IN;
		platform_L2006.innerTarget=-300+300*rampSignalFP(sm_cnt3_2-800,400);
		platform_R2006.innerTarget=300-300*rampSignalFP(sm_cnt3_2-800,400);
	}else if(sm_cnt3_2>1500){
		platform_L2006.innerTarget=-5;
		platform_R2006.innerTarget=5;
    platform_cplt=1;
    sm_cnt3_2=0;
    platform_state=IN;
		platform_action=KEEP_QUIET;
	}
}
void platform_move_out(void){
	sm_cnt3_3++;
  if(sm_cnt3_3<=1500){
		if(sm_cnt3_3>1400){
			airOperator.airOperatorTxBuf[9]=PUSH_OUT;
		}
		platform_L2006.innerTarget=300-300*rampSignalFP(sm_cnt3_3-800,400);
		platform_R2006.innerTarget=-300+300*rampSignalFP(sm_cnt3_3-800,400);
	}else if(sm_cnt3_3>1500){
		platform_L2006.innerTarget=5;
		platform_R2006.innerTarget=-5;
    platform_cplt=1;
    sm_cnt3_3=0;
    platform_state=OUT;
		platform_action=KEEP_QUIET;
  }
}
