#include "route_plan.h"
//****************************************************************************//
//优先级：1：避假块且取两个块  2：避免走4个块 3：走中间避R1  4：避免走3个块
//10下>12下
//左右额外取>路上有3个块
//可能出现：路上有3个块，需转身扔；路上有一个块，需左右另取//优先级未排，随机
//不会出现：除非不走那条路就取不到2个块否则路上不会有4个块
//需要填充各个点位坐标
//****************************************************************************//
void Step_State(void)//根据视觉发填充必要数据
{
    for(int y=0;y<3;y++)//每个台阶数组赋值
    {
        for(int x=0;x<4;x++)
        {
            step_state.judge[x*3+y]=vision_judge[x*3+y];
						if(vision_judge[x*3+y]==3)
						{
								vision_judge[x*3+y]=0;
						}
            if(step_state.judge[x*3+y]==2)
            {
                step_state.flag_fake=y+1;//假块列数
            }
        }
    }
    for(int i=0;i<3;i++)//计算每列R2个数(除123)
    {
        for(int x=1;x<4;x++)
        {
					if(step_state.judge[x*3+i]==1)
            step_state.count_row[i]+=step_state.judge[x*3+i];
        }
    }
    //给route_plan.flag_123赋值
    if(vision_judge[1]==1)
    {
        route_plan.flag_123=2;
    }
    else 
    {
        if(vision_judge[2]==1)
        {
            route_plan.flag_123=3;
        }
        else
        {
            if(vision_judge[0]==1)
            {
                route_plan.flag_123=1;
            }
            else
            {
                route_plan.flag_123=0;
            }
        }
    }
    //计算123共几个R2
    for(int i=0;i<3;i++)
    {
        if(vision_judge[i]==1)
        step_state.sum_123++;
    }    
}
void Route_Plan_choose(void)
{
    switch(step_state.sum_123)
    {
        case 0:
        case 1:
        Route_Plan_1();
        break;
        case 2:
        case 3:
        Route_Plan_2();
        break;
    }
}
void Route_Plan_1(void)//123有0或1个R2
{
    switch(step_state.flag_fake)
    {
        case 1://假块在第1列  //yes
				route_plan.every_state[0].next_id = 5;
				route_plan.every_state[1].next_id = 8;
				if(step_state.count_row[1]==3)//防止走4个块
        {
						if(step_state.judge[4-1]==2)//4是假块
						{
								if ((step_state.judge[7-1]+step_state.judge[10-1]) > (step_state.judge[9-1]+step_state.judge[12-1]))
                {
										route_plan.every_state[2].next_id = 9;
                    route_plan.every_state[3].next_id = 12;
                }
                else
                {
										route_plan.every_state[2].next_id = 7;
                    route_plan.every_state[3].next_id = 10;
                }
						}
						else //7或10是假块
						{
								route_plan.every_state[2].next_id = 9;
								route_plan.every_state[3].next_id = 12;
						}
        }
				else if(step_state.count_row[1]==2&&step_state.sum_123!=0)//防止走4个块
				{
						if(step_state.judge[10-1]!=0)//10号有块 
						{
								if(step_state.judge[12-1]!=1)//12无块
								{
										route_plan.every_state[2].next_id = 11;
										route_plan.every_state[3].next_id = 12;
								}
								else
								{
										if(step_state.judge[5-1]==1&&step_state.judge[8-1]==1)
										{
												route_plan.every_state[1].next_id = 6;
												route_plan.every_state[2].next_id = 9;
												route_plan.every_state[3].next_id = 12;
										}
										else
										{
												route_plan.every_state[2].next_id = 9;
												route_plan.every_state[3].next_id = 12;
										}
								}
								
						}
						else 
						{
								route_plan.every_state[2].next_id = 11;
								route_plan.every_state[3].next_id = 10;
						}
				}
				else if(step_state.judge[10-1]==2)
				{
						route_plan.every_state[2].next_id = 11;
						route_plan.every_state[3].next_id = 12;
				}
				else
				{
						route_plan.every_state[2].next_id = 11;
						route_plan.every_state[3].next_id = 10;
				}
            break;
        case 2://假块在中间  
        if(step_state.judge[5-1]==2)//5为假
        {
				if(step_state.count_row[2]==3||(step_state.sum_123==0&&step_state.count_row[2]==2&&step_state.count_row[0]<2))//防止取不到2个块则此种情况路上可能会有4个块
            {
                route_plan.every_state[0].next_id = 3;
                route_plan.every_state[1].next_id = 6;
                route_plan.every_state[2].next_id = 9;
                route_plan.every_state[3].next_id = 12;
            }
            else
            {
                route_plan.every_state[0].next_id = 1;
                route_plan.every_state[1].next_id = 4;
                route_plan.every_state[2].next_id = 7;
                route_plan.every_state[3].next_id = 10;
            }
        }
        else // 5为真  //yes
        {
            route_plan.every_state[0].next_id = 5;
            if (step_state.judge[8 - 1] == 2) // 8为假    
            {
								if(step_state.sum_123==0&&step_state.count_row[2]==2&&(step_state.count_row[0]+step_state.judge[5])<2)
								{
										route_plan.every_state[1].next_id = 6;
                    route_plan.every_state[2].next_id = 9;
                    route_plan.every_state[3].next_id = 12;
								}
                else if (step_state.count_row[0]>step_state.count_row[2])
                {
                    route_plan.every_state[1].next_id = 6;
                    route_plan.every_state[2].next_id = 9;
                    route_plan.every_state[3].next_id = 12;
                }
                else
                {
                    route_plan.every_state[1].next_id = 4;
                    route_plan.every_state[2].next_id = 7;
                    route_plan.every_state[3].next_id = 10;
                }
            }
            else // 8为真//则11为假
            {
                route_plan.every_state[1].next_id = 8;
                if ((step_state.judge[7-1]+step_state.judge[10-1]) > (step_state.judge[9-1]+step_state.judge[12-1]))
                {
										route_plan.every_state[2].next_id = 9;
                    route_plan.every_state[3].next_id = 12;
                }
                else
                {
										route_plan.every_state[2].next_id = 7;
                    route_plan.every_state[3].next_id = 10;
                }
            }
        }
            break;
        case 3://假块在第3列
        if(step_state.count_row[1]==3)//走3个块>走两侧且避免走4个块
        {
            route_plan.every_state[0].next_id = 5;
            route_plan.every_state[1].next_id = 8;
            route_plan.every_state[2].next_id = 7;
            route_plan.every_state[3].next_id = 10;
        }
				else if(step_state.count_row[1]==2&&step_state.sum_123!=0&&step_state.judge[10-1]==1)//防止走4个块
				{
						if(step_state.judge[11-1]==1)
						{
								route_plan.every_state[0].next_id = 5;
								route_plan.every_state[1].next_id = 8;
								route_plan.every_state[2].next_id = 7;
								route_plan.every_state[3].next_id = 10;
						}
						else
						{
								if(step_state.judge[12-1]==0)
								{
										route_plan.every_state[0].next_id = 5;
										route_plan.every_state[1].next_id = 8;
										route_plan.every_state[2].next_id = 11;
										route_plan.every_state[3].next_id = 12;
								}
								else
								{
										route_plan.every_state[0].next_id = 5;
										route_plan.every_state[1].next_id = 4;
										route_plan.every_state[2].next_id = 7;
										route_plan.every_state[3].next_id = 10;
								}
						}
				}
        else
        {
            route_plan.every_state[0].next_id = 5;
            route_plan.every_state[1].next_id = 8;
            route_plan.every_state[2].next_id = 11;
            route_plan.every_state[3].next_id = 10;
        }
            break;
    }    
}
void Route_Plan_2(void)//123有2或3个R2
{
    switch(step_state.flag_fake){
        case 1://假块在第1列
				if(step_state.judge[10-1]!=2)//10号非假
        {
						route_plan.every_state[0].next_id = 5;
            route_plan.every_state[1].next_id = 8;
            route_plan.every_state[2].next_id = 11;
            route_plan.every_state[3].next_id = 10;
				}
				else
			  {
						route_plan.every_state[0].next_id = 5;
						route_plan.every_state[1].next_id = 8;
						route_plan.every_state[2].next_id = 11;
						route_plan.every_state[3].next_id = 12;
				}
            break;
        case 2://假块在中间
        if(step_state.judge[5-1]==2)//5为假
        {
						if(step_state.count_row[0]>step_state.count_row[2])//可优化+枚举
						{
								route_plan.every_state[0].next_id = 3;
								route_plan.every_state[1].next_id = 6;
								route_plan.every_state[2].next_id = 9;
								route_plan.every_state[3].next_id = 12;
						}
						else
						{
								route_plan.every_state[0].next_id = 1;
								route_plan.every_state[1].next_id = 4;
								route_plan.every_state[2].next_id = 7;
								route_plan.every_state[3].next_id = 10;
						}
        }
        else // 5为真
        {
            route_plan.every_state[0].next_id = 5;
            if (step_state.judge[8 - 1] == 2) // 8为假
            {
								if(step_state.count_row[0]+step_state.judge[5-1]==2)
								{
										route_plan.every_state[1].next_id = 6;
										route_plan.every_state[2].next_id = 9;
										route_plan.every_state[3].next_id = 12;
								}
								else
								{
										route_plan.every_state[1].next_id = 4;
										route_plan.every_state[2].next_id = 7;
										route_plan.every_state[3].next_id = 10;
								}
            }
            else // 8为真//则11为假
            {
								route_plan.every_state[1].next_id = 8;
                if((2==(step_state.judge[7-1]+step_state.judge[10-1]+step_state.judge[5-1]+step_state.judge[8-1]))&&
									(2>(step_state.judge[9-1]+step_state.judge[12-1]+step_state.judge[5-1]+step_state.judge[8-1])))
                {
                    route_plan.every_state[2].next_id = 9;
                    route_plan.every_state[3].next_id = 12;
                }
                else
                {
                    route_plan.every_state[2].next_id = 7;
                    route_plan.every_state[3].next_id = 10;
                }
            }
        }
            break;
        case 3://假块在第3列
					if(step_state.count_row[1]==1&&step_state.judge[10-1]==1&&step_state.judge[12-1]==0)
					{
							route_plan.every_state[0].next_id = 5;
							route_plan.every_state[1].next_id = 8;
							route_plan.every_state[2].next_id = 11;
							route_plan.every_state[3].next_id = 12;
					}
					else
					{
							route_plan.every_state[0].next_id = 5;
							route_plan.every_state[1].next_id = 8;
							route_plan.every_state[2].next_id = 11;
							route_plan.every_state[3].next_id = 10;
					}
            break;
    }
}
void Block_Plan(void)//路径规划结束后填充方块数据
{
    for(int i=0;i<4;i++)//填充是否有方块及路上方块总数//走123会出bug!!!!!
    {
        route_plan.every_state[i].judge = step_state.judge[route_plan.every_state[i].next_id-1];
        if( route_plan.every_state[i].judge!=1)
        {
            route_plan.every_state[i].judge = 0;
        }
        else 
        {
            route_plan.count_num++;
        }
    }
		if(step_state.sum_123!=0)//排除取过1或3但又走1或3导致重复计算的情况
    {
        if(route_plan.flag_123==route_plan.every_state[0].next_id)
        {
            route_plan.count_num--;
						route_plan.every_state[0].judge=0;
        }
    }
    //填充extra_id
    if((route_plan.count_num==1&&route_plan.flag_123==0)||(route_plan.count_num==0&&route_plan.flag_123!=0))
    {
        for(int i=3;i>=0;i--)
        {
						int j;
            int x=(route_plan.every_state[i].next_id-1)/3;
						int y=(route_plan.every_state[i].next_id-1)%3;
						if(y==2)
						{
								j=y;
						}
						else
						{
								j=y+1;
						}
						if(route_plan.extra_id1!=0)
								break;
            for(;j>=y-1&&j>0;j--)
            {
                if(step_state.judge[3*x+j]==1)
                {
                    route_plan.extra_id1=3*x+j+1;
                    break;
                }
            }    
        }
				if(route_plan.extra_id1==0)
				{
						if(route_plan.flag_123==3)
						{
								route_plan.extra_id1=1;
						}
						else
						{
								route_plan.extra_id1=3;
						}
				}
    }
    else if(route_plan.count_num==0)
    {
        for(int i=3;i>=0;i--)
        {
						int j,k;
            int x=(route_plan.every_state[i].next_id-1)/3;
						int y=(route_plan.every_state[i].next_id-1)%3;
						if(y==2)
						{
								j=y;k=y;
						}
						else
						{
								j=y+1;k=y+1;
						}
            for(;j>=y-1&&j>0;j--)
            {
                if(step_state.judge[3*x+j]==1&&route_plan.extra_id2==0)
                {
                    route_plan.extra_id2=3*x+j+1;
                    k=j-1;
                    break;
                }
            }
            for(;k>=y-1&&k>0;k--)
            {
                if(step_state.judge[3*x+k]==1)
                {
                    route_plan.extra_id1=3*x+k+1;
                    break;
                }
            }
            if(route_plan.extra_id1!=0&&route_plan.extra_id2!=0)
            break;
        }
				if(route_plan.extra_id1==0)
				{
						if(route_plan.flag_123==3)
						{
								route_plan.extra_id1=1;
						}
						else
						{
								route_plan.extra_id1=3;
						}
				}
    }
}
void Route_Update(void)
{
	  vision_data_recieve.flag_123=route_plan.flag_123;
	  vision_data_recieve.count_num=route_plan.count_num;
		if(flag_use_vision==0)
		{
			  vision_data_recieve.I_max=5;
				vision_data_recieve.extra_id1=route_plan.extra_id1;
				vision_data_recieve.extra_id2=route_plan.extra_id2;
				switch(flag_vision_update)
				{
					case 0:
						vision_data_recieve.next_id=route_plan.every_state[0].next_id;
						vision_data_recieve.judge=route_plan.every_state[0].judge;
						break;
					case 1:
						vision_data_recieve.next_id=route_plan.every_state[1].next_id;
						vision_data_recieve.judge=route_plan.every_state[1].judge;
						break;
					case 2:
						vision_data_recieve.next_id=route_plan.every_state[2].next_id;
						vision_data_recieve.judge=route_plan.every_state[2].judge;
						break;
					case 3:
						vision_data_recieve.next_id=route_plan.every_state[3].next_id;
						vision_data_recieve.judge=route_plan.every_state[3].judge;
						break;
					case 4:
						vision_data_recieve.next_id=route_plan.every_state[3].next_id;
						vision_data_recieve.judge=route_plan.every_state[3].judge;
						break;
					default:
						break;
					
				}
		}
		
}


