#include "action.h"

uint8_t decision_tree[4][10] = {
    {1, 3, 14, 4, 47, 2, 70, 4, 100, 4},
    {2, 2, 25, 2, 58, 2, 81, 4, 110, 5},
    {10, 2, 107, 2, 74, 4, 41, 2, 255, 5},
    {11, 3, 118, 2, 85, 4, 52, 4, 20, 4}};

void Main_Task()
{
    switch (ace.state){
        case Total_Init:
            if (ace.flag == 1)
            {
                nav.nav_state = CHASSIS_INIT;
                foot.foot_state = FOOT_INIT;
                ace.flag = 0;
            }
            if (nav.nav_state == CHASSIS_INIT_DONE && foot.foot_state == FOOT_CLEAR)
            {
                ace.state = Total_Clear;
                ace.flag = 1;
            }
            break;

        case Total_Clear:
            break;

        case TASK_1_4_7_10: 
            ace.path_inx = 0;
            Task_Across_Forest();
            break;

        case TASK_2_5_8_11:
            ace.path_inx = 1;
            Task_Across_Forest();
            break;

        case TASK_10_7_4_1:
            ace.path_inx = 2;
            Task_Across_Forest();
            break;

        case TASK_11_8_5_2:
            ace.path_inx = 3;
            Task_Across_Forest();
            break;
        
        default:
            break;
    }
}

void Task_Across_Forest()
{
    switch (ace.across){
        case from_One_to_2_1:
            if (ace.flag_ == 0)
            {
                nav.auto_path.number_point = decision_tree[ace.path_inx][2 * ace.across];
                Path_Point.flag_point_to_point = 1;
                nav.nav_state = NAV_POINT_TO_POINT;
                ace.flag_ = 1;
            }
            if (ace.flag_ == 1 && nav.nav_state == NAV_LOCK)
            {
                foot.foot_state = decision_tree[ace.path_inx][2 * ace.across + 1];
                ace.flag_ = 2;
            }
            if (ace.flag_ == 2 && foot.foot_state == FOOT_CLEAR)
            {
                ace.across = from_2_1_to_2_2;
                ace.flag_ = 0;
            }
            break;

        case from_2_1_to_2_2:
            if (ace.flag_ == 0)
            {
                nav.auto_path.number_point = decision_tree[ace.path_inx][2 * ace.across];
                Path_Point.flag_point_to_point = 1;
                nav.nav_state = NAV_POINT_TO_POINT;
                ace.flag_ = 1;
            }
            if (ace.flag_ == 1 && nav.nav_state == NAV_LOCK)
            {
                foot.foot_state = decision_tree[ace.path_inx][2 * ace.across + 1];
                ace.flag_ = 2;
            }
            if (ace.flag_ == 2 && foot.foot_state == FOOT_CLEAR)
            {
                ace.across = from_2_2_to_2_3;
                ace.flag_ = 0;
            }
            break;

        case from_2_2_to_2_3:
            if (ace.flag_ == 0)
            {
                nav.auto_path.number_point = decision_tree[ace.path_inx][2 * ace.across];
                Path_Point.flag_point_to_point = 1;
                nav.nav_state = NAV_POINT_TO_POINT;
                ace.flag_ = 1;
            }
            if (ace.flag_ == 1 && nav.nav_state == NAV_LOCK)
            {
                foot.foot_state = decision_tree[ace.path_inx][2 * ace.across + 1];
                ace.flag_ = 2;
            }
            if (ace.flag_ == 2 && foot.foot_state == FOOT_CLEAR)
            {
                ace.across = from_2_3_to_2_4;
                ace.flag_ = 0;
            }
            break;

        case from_2_3_to_2_4:
            if (ace.flag_ == 0)
            {
                nav.auto_path.number_point = decision_tree[ace.path_inx][2 * ace.across];
                Path_Point.flag_point_to_point = 1;
                nav.nav_state = NAV_POINT_TO_POINT;
                ace.flag_ = 1;
            }
            if (ace.flag_ == 1 && nav.nav_state == NAV_LOCK)
            {
                foot.foot_state = decision_tree[ace.path_inx][2 * ace.across + 1];
                ace.flag_ = 2;
            }
            if (ace.flag_ == 2 && foot.foot_state == FOOT_CLEAR)
            {
                ace.across = from_2_4_to_Three;
                ace.flag_ = 0;
            }
            break;

        case from_2_4_to_Three:
            if (ace.flag_ == 0)
            {
                nav.auto_path.number_point = decision_tree[ace.path_inx][2 * ace.across];
                Path_Point.flag_point_to_point = 1;
                nav.nav_state = NAV_POINT_TO_POINT;
                ace.flag_ = 1;
            }
            if (ace.flag_ == 1 && nav.nav_state == NAV_LOCK)
            {
                foot.foot_state = decision_tree[ace.path_inx][2 * ace.across + 1];
                ace.flag_ = 2;
            }
            if (ace.flag_ == 2 && foot.foot_state == FOOT_CLEAR)
            {
                ace.across = from_One_to_2_1;
                ace.flag_ = 0;
                ace.state = Total_Clear;
            }
            break;
        
        default:
            break;
    }
} 

