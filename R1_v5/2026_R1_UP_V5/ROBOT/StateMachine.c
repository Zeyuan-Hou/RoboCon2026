#include "StateMachine.h"
#include "UpAction.h"

#define START_1 0
#define PART_1 1
#define PART_2 2
#define PART_3 3
#define START_2 4


//u8 update_ii_action(u8* refresh){
//    static u8 already_get=0,already_store=0;
////	KFS_cnt_update=0;
//    if(*refresh){
//        *refresh=0;
//        already_get=0;
//        already_store=0;
////      	KFS_cnt_update=0;
//    }
//    if(ii_robot.KFS_store){
//        if(!already_store){
//            nav_target=serachRowAndLine(0x20000+ii_robot.now_KFS_ID);
//            if(KFS_master_state==HOLD_KFS){
//                if(ii_robot.R1_KFS_cnt==1){
//                    store_KFS_orientation=0;
//                }else if(ii_robot.R1_KFS_cnt==2){
//                    store_KFS_orientation=1;
//                }
//                KFS_master_action=STORE_KFS;
//            }
//            if(KFS_master_state==INIT){
//                complete_KFS=0;
//                get_KFS_height=ii_robot.KFS_height;
//                
//                already_store=1;
//            }
//        }else{
//            if(!already_get){
//                if(nav_cplt){
//                    nav_cplt=0;
//										KFS_master_action=GET_KFS;
//                    complete_KFS=1;
//                }
//                if(chassis_move){
//                    ii_robot.R1_KFS_cnt++;
//                    already_get=1;
//                }
//            }else{
//                return 1;
//            }
//        }
//    }else{
//        if(!already_get){
//            nav_target=serachRowAndLine(0x20000+ii_robot.now_KFS_ID);
//            if(KFS_master_state==INIT){
//                complete_KFS=0;
//                get_KFS_height=ii_robot.KFS_height;
//                
//            }
//            if(nav_cplt){
//                nav_cplt=0;
//								KFS_master_action=GET_KFS;
//                complete_KFS=1;
//            }
//            if(chassis_move){
//                already_get=1;
//                ii_robot.R1_KFS_cnt++;
//            }
//        }else{
//            return 1;
//        }
//    }
//    return 0;
//}
u8 autoUpdate_ii_upAction(u8* data,fp32 vel){
	static u8 step=0,KFS_index=0;
	u8 trash=0;
	for(u8 i=0;i<3;i++){
		trash+=(data[i]>0);
	}
	switch(step){
		case 0:
			if(ii_robot.move_in_flag==1){
//				if(ableToGetKFS(data[KFS_index],vel,ii_robot.KFS_GET_MODE[KFS_index])==1){
//					step++;
//					cmf=0;
//				}
				step++;
				cmf=0;
			}
			break;
		case 1:
			if(KFS_index<trash){
				update_KFS_msgs(data[KFS_index]);
				KFS_master_action=GET_AND_STORE;
				if(KFS_index==0){
					store_KFS_orientation=STORE_AND_GET_STORED_RIGHT;
				}else{
					store_KFS_orientation=STORE_AND_GET_STORED_LEFT;
				}
			}else if(KFS_index==trash){
				update_KFS_msgs(data[KFS_index]);
				KFS_master_action=GET_KFS;
			}else{
				return 1;
			}
			step++;
			break;
		case 2:
			if(nav_cplt){
				nav_cplt=0;
				complete_KFS=1;
				step++;
			}
			if(ii_robot.move_in_flag==2){
				complete_KFS=1;
				step++;
			}
			break;
		case 3:
			if(chassis_move){
				cmf=1;
			}
			if(KFS_master_action==KEEP_QUIET||KFS_master_state==HOLD_KFS){
				step=0;
				ii_robot.R1_KFS_cnt++;
				KFS_index++;
				return 1;
			}
			break;
		default:
			break;
	}
	return 0;
}
fp32 getKFSPosition[16]={2253,2642,5291,3418,0,4078,4610,0,2882,5811,2687,1688,1438,3797,1461,1454};//10-13:mode=2:1,3,10,12
fp32 getKFSPositions[16]={-2132,2642,5291,-3364,0,4078,-4531,0,2882,-5732.f,2687,1688,1438,3797,1461,1454};//10-13:mode=2:1,3,10,12
u8 ableToGetKFS(u8 id,fp32 vel,u8 mode){
	u8 trust = (u8)(((u16)(robotPos.Q*100))%157);
	if(trust>10&&trust<147){
		return 0;
	}
	if(!mode){
		return 0;
	}
	fp32 dx=0;
	switch (id){
		case 1:
			if(mode==1){
				dx=fabsf(robotPos.pos_y - getKFSPosition[id-1]);
			}else{
				dx=fabsf(robotPos.pos_x - getKFSPosition[12]);
			}
			break;
		case 3:
			if(mode==1){
				dx=fabsf(robotPos.pos_y - getKFSPosition[id-1]);
			}else{
				dx=fabsf(robotPos.pos_x - getKFSPosition[13]);
			}
			break;
		case 10:
			if(mode==1){
				dx=fabsf(robotPos.pos_y - getKFSPosition[id-1]);
			}else{
				dx=fabsf(robotPos.pos_x - getKFSPosition[14]);
			}
			break;
		case 12:
			if(mode==1){
				dx=fabsf(robotPos.pos_y - getKFSPosition[id-1]);
			}else{
				dx=fabsf(robotPos.pos_x - getKFSPosition[15]);
			}
			break;
		case 4:
		case 7:
		case 6:
		case 9:
			dx=fabsf(robotPos.pos_y - getKFSPosition[id-1]);
			break;
		case 2:
		case 11:
			dx=fabsf(robotPos.pos_x - getKFSPosition[id-1]);
			break;
		default:
			break;
	}
	if(dx<600){
		if(dx>(vel*0.35f)){
			return 1;
		}else{
			if(vel<0.1f){
				return 0;
			}
			return 2;
		}
	}else{
		return 0;
	}
}
u8 update_ii_action(u8* refresh){
	static u8 step=0;
	switch(step){
		case 0:
			input_mode =0;//自锁，防止突然接收新消息产生错误
//			chassis_action=3;//startNav
//			nav_target=serachRowAndLine(0x20000+ii_robot.now_KFS_ID);
			timer++;
			if(timer>100){
				step++;
				timer=0; 
			}
			break;
		case 1:
			if(!KFS_cplt) return 0;
			if(ii_robot.move_in_flag||nav_progress>0.98f){
//			if(nav_progress>0.98f){
//				get_KFS_height=ii_robot.KFS_height;
				if(ii_robot.R1_KFS_cnt==0){
					KFS_master_action=GET_AND_STORE;
					store_KFS_orientation=STORE_AND_GET_STORED_RIGHT;
//				}else if(ii_robot.R1_KFS_cnt==1){
//					KFS_master_action=GET_AND_STORE;
//					store_KFS_orientation=1;
				}else if(ii_robot.R1_KFS_cnt==1){
					KFS_master_action=GET_KFS;
				}
				step++;
			}
			break;
		case 2:
			if(nav_cplt){
				complete_KFS=1;
				chassis_move=0;
				nav_cplt=0;
				step++;
			}
			break;
		case 3:
			if(chassis_move){
				step=0;
				input_mode=1;//解除自锁
				ii_robot.R1_KFS_cnt++;
				return 1;
			}
			break;
		default:
			break;
	}
	return 0;
}
void StateMachine(void){
    switch(part_flag){
        case START_1:
						switch(part_step){
							case 0:
								weapon_player_action=INIT_ACTION;
								KFS_master_action=INIT_ACTION;
								platform_action=INIT_ACTION;
								chassis_action = 1;//INITIALIZE
								part_step++;
								break;
							default:
								break;
						}
            
            break;
        case PART_1:
            switch(part_step){
							case 0://导航去取杆
                    weapon_player_action=GET_WEAPON;
                    chassis_action=3;//NAV
                    nav_target=0x1001;//get first weapon
                    if(nav_cplt){
                        nav_cplt=0;
                        complete_weapon=1;
												part_step++;
                    }
                    break;
							case 1:
									if(weapon_player_state==GRAB_HALFWEAPON){
                        part_step++;
                    }
              case 10://当机器人第一次没抓到武器,待机器人做完GET_WEAPON的所有动作后,遥控器按x+1,x+2,x+3,x+4再进此case
                    if(!part1_refresh){
                        weapon_player_state=INIT;
                        part1_refresh=1;
                    }
                    weapon_player_action=GET_WEAPON;
                    chassis_action=3;//NAV
                    nav_target=0x1000+weapon_num;//get target weapon
                    if(nav_cplt){
                        nav_cplt=0;
                        complete_weapon=1;
												part_step=1;
                        part1_refresh=0;
                    }
                    break;
                case 2://导航去对接
                    timer++;
                    weapon_player_action=STORE_WEAPON;
										if(timer>2){
											nav_target=0x1005;//move to combine weapon
											part_step++;
											timer=0;
										}
                    break;
								case 3:
                    if(weapon_player_state==HOLD_HALFWEAPON){
													weapon_player_action=COMBINE_WEAPON;
													timer=0;
													part_step++;
                    }
										break;
								case 4:
									if(nav_cplt){
										nav_cplt=0;
										chassis_action=2;
										part_step=9;
									}
                case 5://对接
									if(nav_cplt){//边导航边对接，导航完毕时则认为对接成功
												nav_cplt=0;
												complete_weapon=1;
										}
                    if(weapon_player_state==HOLD_WEAPON){
                        part_flag++;//切换到二区状态机
												weapon_player_state=INIT;
												weapon_player_action=KEEP_QUIET;
                        part_step=1;
                        chassis_action=2;//manual
                    }
                    break;
                default:
                    break;
            }
            break;
        case PART_2:
            switch(part_step){
							case 20:
								chassis_action=11;
								if(autoUpdate_ii_upAction(ii_robot.KFS_IDs,chassis_vel_rec1)){
									part_step++;
								}
								break;
							case 21:
								if(autoUpdate_ii_upAction(ii_robot.KFS_IDs,chassis_vel_rec2)){
									part_step++;
								}
								break;
//							case 22:
//								if(autoUpdate_ii_upAction(ii_robot.KFS_IDs,chassis_vel_rec)){
//									part_step++;
//								}
//								break;
							case 22:
								if(KFS_master_state==HOLD_KFS){
									KFS_master_action=USE_KFS;
								}
								if(nav_cplt){
									nav_cplt=0;
									part_step=0;
									part_flag++;
								}
								break;
                case 0://导航取方块
                    if(update_ii_action(&refresh)){
												refresh=1;
                        part_step=1;
                        chassis_action=2;//manual
                    }
                    break;
                case 1://手操去每列导航起始点
                    // chassis_action=2;//manual
                    break;
                case 10://刷新二区状态
                    if(!part2_refresh){
                        ii_robot.now_KFS_ID=0;
                        ii_robot.R1_KFS_cnt=0;
                        ii_robot.pre_KFS_ID=0;
                        KFS_master_state=INIT;
                        KFS_master_action=KEEP_QUIET;
                        part2_refresh=1;
												chassis_action=2;//manual
                        part_step=1;
                    }
                    break;
                case 2:
                    chassis_action=3;//NAV
                    nav_target=0x3002;//move up to part_III
                    if(nav_cplt){
                        nav_cplt=0;
                        part_flag=3;
												part_step=0;
                    }
                    break;
                default:
                    break;
            }
            break;
        case PART_3:
            switch(part_step){
							case 30:
									switch(lil_step){
										case 0:
											nav_target=PUT_KFS_IN_SECOND_COLUMN;
											chassis_action=3;
											lil_step++;
											break;
										case 1:
											if(robotPos.pos_y>9550){
													lil_step++;
											}
											break;
										case 2:
											timer++;
											if(timer>1000){
												KFS_master_action=USE_KFS;
												use_KFS_orientaton=USE_KFS_LEFT_OF_R1;
												timer=0;
												lil_step++;
											}
											break;
										case 3:
											timer++;
											if(timer>10){
												timer=0;
												lil_step++;
											}
											break;
										case 4:
											if(KFS_cplt){
												KFS_master_action=GET_STORED_KFS;
												lil_step++;
											}
											break;
										case 5:
											timer++;
											if(timer>1400){
												weapon_player_action=MOVE_WEAPON_OUT;
												timer=0;
												lil_step++;
											}
											break;
										default:
											break;
									}
									break;
								case 31:
									switch(lil_step){
										case 0:
											nav_target=0x3051;
											chassis_action=3;
											lil_step++;
											break;
										case 1:
											if(nav_progress>0.8f){
												weapon_player_action=USE_WEAPON;
												lil_step++;
											}
											break;
										case 2:
											if(weapon_cplt){
												nav_cplt=0;
												lil_step=0;
												part_step=0;
											}
											break;
										default:
											break;
									}
									break;
                case 0:
                    // chassis_action=2;//manual
                    break;
                case 1:
                    switch(lil_step){
                        case 0:
                            nav_target=0x3001;//move to part_III left; 30002:straight; 30003:right
                            if(nav_progress>0.5f){
                                complete_KFS=0;
                                KFS_master_action=USE_KFS;
                            }
                            if(nav_cplt){
                                nav_cplt=0;
                                lil_step++;
                            }
                            break;
                        case 1:
                            complete_KFS=1;
                            if(KFS_cplt){
                                KFS_cplt=0;
                                nav_target=0x3100;//move to left to let R2 use KFS
                                ii_robot.R1_KFS_cnt--;
                            }
                            if(nav_cplt){
                                if(ii_robot.R1_KFS_cnt==2){
                                    store_KFS_orientation=1;
                                }else if(ii_robot.R1_KFS_cnt==1){
                                    store_KFS_orientation=0;
                                }
                                KFS_master_action=GET_STORED_KFS;//get stored KFS
                                nav_cplt=0;
                                lil_step++;
                            }
                            break;
                        case 2:
                            if(KFS_master_state==HOLD_KFS){
                                part_step=0;
                                lil_step=0;
                                chassis_action=2;//manual
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case 2://
                    switch(lil_step){
                        case 0:
                            nav_target=0x3002;//move to part_III left; 30002:straight; 30003:right
                            if(nav_progress>0.5f){
                                complete_KFS=0;
                                KFS_master_action=USE_KFS;
                            }
                            if(nav_cplt){
                                nav_cplt=0;
                                lil_step++;
                            }
                            break;
                        case 1:
                            complete_KFS=1;
                            if(KFS_cplt){
                                KFS_cplt=0;
                                nav_target=0x3100;//move to left to let R2 use KFS
                                ii_robot.R1_KFS_cnt--;
                            }
                            if(nav_cplt){
                                if(ii_robot.R1_KFS_cnt==2){
                                    store_KFS_orientation=1;
                                }else if(ii_robot.R1_KFS_cnt==1){
                                    store_KFS_orientation=0;
                                }
                                KFS_master_action=GET_STORED_KFS;//get stored KFS
                                nav_cplt=0;
																
                                lil_step++;
                            }
                            break;
                        case 2:
                            if(KFS_master_state==HOLD_KFS){
                                part_step=0;
                                lil_step=0;
                                chassis_action=2;//manual
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case 3:
                    switch(lil_step){
                        case 0:
                            nav_target=0x3003;//move to part_III left; 30002:straight; 30003:right
                            if(nav_progress>0.5f){
                                complete_KFS=0;
                                KFS_master_action=USE_KFS;
                            }
                            if(nav_cplt){
                                nav_cplt=0;
                                lil_step++;
                            }
                            break;
                        case 1:
                            complete_KFS=1;
                            if(KFS_cplt){
                                KFS_cplt=0;
                                nav_target=0x3300;//move to right to let R2 use KFS
                                ii_robot.R1_KFS_cnt--;
                            }
                            if(nav_cplt){
                                if(ii_robot.R1_KFS_cnt==2){
                                    store_KFS_orientation=1;
                                }else if(ii_robot.R1_KFS_cnt==1){
                                    store_KFS_orientation=0;
                                }
                                KFS_master_action=GET_STORED_KFS;//get stored KFS
                                nav_cplt=0;
                                lil_step++;
                            }
                            break;
                        case 2:
                            if(KFS_master_state==HOLD_KFS){
                                part_step=0;
                                lil_step=0;
                                chassis_action=2;//manual
                            }
                            break;
												case 3:
													if(platform_state==IN){
														platform_action=MOVEOUT;
													}
													if(platform_state==OUT){
														part_step=4;
													}
													break;
												case 4:
													if(platform_state==OUT){
														platform_action=MOVEIN;
													}
													if(platform_state==IN){
														part_step=3;
													}
													break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
        }
}
u16 serachRowAndLine(u32 ID){
	u16 ral=0;
	switch(ID){
		case 0x20001:
			ral= 0x2010;
			break;
		case 0x20002:
			ral= 0x2002;
			break;
		case 0x20003:
			ral= 0x2014;
			break;
		case 0x20004:
			ral= 0x2020;
			break;
		case 0x20006:
			ral= 0x2024;
			break;
		case 0x20007:
			ral= 0x2030;
			break;
		case 0x20009:
			ral= 0x2034;
			break;
		case 0x2000A:
			ral= 0x2040;
			break;
		case 0x2000B:
			ral= 0x2052;
			break;
		case 0x2000C:
			ral= 0x2044;
			break;
		default:
			ral= 0;
			break;
	}
	return ral;
}
void update_KFS_msgs(u8 id){
	switch(id){
		case 1:
			get_KFS_height=2;
			break;
		case 2:
			get_KFS_height=1;
			break;
		case 3:
			get_KFS_height=2;
			break;
		case 4:
			get_KFS_height=1;
			break;
		case 6:
			get_KFS_height=3;
			break;
		case 7:
			get_KFS_height=2;
			break;
		case 9:
			get_KFS_height=2;
			break;
		case 10:
			get_KFS_height=1;
			break;
		case 11:
			get_KFS_height=2;
			break;
		case 12:
			get_KFS_height=1;
			break;
		default:
			break;
	}
	if(ii_robot.R1_KFS_cnt==0){
		store_KFS_orientation=0;
		ii_robot.KFS_store=1;
	}else if(ii_robot.R1_KFS_cnt==1){
		store_KFS_orientation=1;
		ii_robot.KFS_store=1;
	}else if(ii_robot.R1_KFS_cnt==2){
		ii_robot.KFS_store=0;
	}
}
