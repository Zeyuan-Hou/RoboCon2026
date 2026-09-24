RC_R2_v2_part1是上下台阶完成后，再启动point_to_point；RC_R2_v2.1_part1则是在上台阶收后脚时提前启用point_to_point。RC_R2_v2.1_part1主状态机flag太多不便于理解，耦合度高，若要二次开发建议参考RC_R2_v2_part1，进一步地给二区自动流程提速再参考RC_R2_v2.1_part1。

RC_R2_v2.1_part1在RC_R2_v2_part1的基础上修改了action.c，在foot.c的case UP14、case UP24中增加了切换nav_state启用point_to_point。

底盘：
第三代R2修改了底盘三舵轮位置、车后方挂钩的位置，需要修改global_declare.h中关于底盘的宏定义，也需要调节自旋中心的位置。由第二代R2底盘知，舵轮三角形外切圆圆心和整车几何中心不重合，若自旋中心是舵轮三角形外切圆圆心，能最大化舵轮的性能，但是在台阶上自旋时车后方挂钩的架子可能会蹭到高一级台阶，建议自旋中心设在靠近整车几何中心的位置。
舵轮驱动电机的控制采用PI_Feedforward_Calc，相当于自带摩擦补偿，所以仅在上斜坡时再启用friction_compensation。

小脚：
FOOT_INIT ---> FOOT_CLEAR
FOOT_UPSTAIRS_LV1 {UP11 ---> UP12 ---> UP13 ---> UP14} ---> FOOT_INIT ---> FOOT_CLEAR
FOOT_UPSTAIRS_LV2 {UP21 ---> UP22 ---> UP23 ---> UP24} ---> FOOT_INIT ---> FOOT_CLEAR
FOOT_DOWNSTAIRS_LV1 {DOWN11 ---> DOWN12 ---> DOWN13 ---> DOWN14} ---> FOOT_INIT ---> FOOT_CLEAR
FOOT_DOWNSTAIRS_LV2 {DOWN21 ---> DOWN22 ---> DOWN23 ---> DOWN24} ---> FOOT_INIT ---> FOOT_CLEAR
例如把foot.foot_state改成FOOT_UPSTAIRS_LV2，即可向车身正前方上四百，foot.foot_state变回FOOT_CLEAR上台阶过程结束。
两个j60控一个小脚：j60_1用MIT控制，用j60_1的反馈角度和目标角度pid得到力矩，纯力矩控制j60_2。

微调：
集成了只走xy、只走x、只走y、只走yaw、同时xyyaw五种模式，每一小段路径都能用不同模式。需要调节判定到目标位置的死区。PID_Fuzzy_Calc和CalTD互斥，不能同时使用，追求速度用PID_Fuzzy_Calc，追求平滑用普通PID+CalTD。

主状态机：
相当于一键走完一列台阶，直接用二维数组decision_tree枚举每个子状态的point_to_point路径、foot.foot_state。

可能出现的问题：
遥控舵轮走不了直线，大概率不是pid的问题，如果让转向电机正转360度反转360度发现会偏，就说明2006联轴器寄了。
小脚位控不精准，可能是因为连接电机和齿轮/电机和小脚的螺栓松了/断了。