#ifndef TASK_STATE_MACHINE_HPP_
#define TASK_STATE_MACHINE_HPP_

#include <chrono>
#include <map>
#include <string>
#include <vector>

// ROS 2 Core and Standard Messages
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

// Custom Messages
#include "quad/msg/state_command.hpp"

/**
 * @brief 上层任务状态枚举
 */
enum class State {
    IDLE,               // 空闲，保持站立
    AUTO_NAV,           // 自动导航模式
    MANUAL_NAV,         // 手动遥控模式
    QR_RECOGNITION,     // 二维码视觉伺服对准
    TASK_WALL_CROSS,    // 过高墙
    TASK_WALL_POLICY_RESTORE, // 高墙 jump 后恢复小跑策略
    TASK_SAND_INOUT,    // 过沙坑
    TASK_BRIDGE_CROSS,  // 过木桥
    TASK_CRAWL_CROSS,   // 匍匐过矮杆 (ID: 1)
    TASK_CRAWL_MOVING,  // 匍匐盲走中（细分状态，供执行器监听）
    TASK_STAIR_UP,      // 上楼梯 (ID: 5)
    TASK_STAIR_MOVING,  // 上楼梯闭环控制中（细分状态，供执行器监听）
    TASK_STAIR_DOWN,    // 下楼梯 (预留)
    TASK_PATH_POLICY_PREP,    // 路径跟踪前策略切换
    TASK_PATH_POLICY_RESTORE, // 路径跟踪后策略恢复
    TASK_POLE_AROUND,   // 绕杆 (ID: 6)
    TASK_SLOPE_CROSS,   // 过斜坡
    LOGISTICS_INIT_MANIP, // 任务赛开始前机械臂初始化
    LOGISTICS_PRE_SCAN,  // 任务赛低位左右看预扫描
    LOGISTICS_LIFT_ARM,  // 任务赛预扫描后机械臂抬起
    LOGISTICS_WAIT_REMOTE_START, // 任务赛预扫描后等待遥控确认
    LOGISTICS_NAV,      // 任务赛物流导航
    TASK_PICKUP_BLOCK,  // 任务赛取物块
    TASK_DROPOFF_BLOCK, // 任务赛放物块
    LOGISTICS_DONE,     // 任务赛结束
    TASK_OTHER          // 其他备用任务
};

/**
 * @brief 底层状态机状态ID枚举
 */
enum class LowerStateID : int {
    STOP = 0,
    PASSIVE = 1,
    FIXED_DOWN = 2,
    FIXED_STAND = 3,
    FREE_STAND = 4,
    RL_MOVE = 5,
    JUMP = 6,
    STRIDE = 7,
    SMALL_JUMP = 8
};

/**
 * @brief 发送给底层状态机的事件ID枚举
 */
enum class LowerEvent : int {
    ENABLE = 0,         
    DISABLE,            
    UP_DOWN,            
    DAMPING,            
    ENTER_RL,           
    STRIDE,             
    SMALL_JUMP,         
    JUMP,               
    POLICY_TROT = 9,        
    POLICY_CREEP = 10,       
    POLICY_UPSTAIR = 11,      
    ENTER_AUTO = 12,          
    ENTER_MANUAL,        
    QR_RECOGNITION,      
    PICK_UP,
    POLICY_KNEEL_CRAWL = 21
}; 

/**
 * @brief 任务状态机主类：纯粹的离散事件状态机 (DES)
 * 
 * 负责管理宏观状态，下发动作指令给底层，不包含任何具体的连续速度控制算法。
 */
class TaskStateMachine : public rclcpp::Node {
public:
    TaskStateMachine();

private:
    enum class ManipPhase {
        STOP_NAV,
        PRE_STAND_WAIT,
        TO_STAND,
        STAND_SETTLE,
        TRIGGER_ARM,
        ARM_CMD_SETTLE,
        WAIT_ARM,
        BACK_TO_RL,
        RL_SETTLE,
        DONE
    };

    enum class ManipulationType {
        PICKUP,
        DROPOFF
    };

    enum class LogisticsInitPhase {
        INIT_CMD,
        WAIT_AFTER_INIT,
        PLACE_LOW_CMD,
        WAIT_AFTER_PLACE_LOW,
        DONE
    };

    enum class KneelPolicyPhase {
        TO_FIXED_STAND,
        TO_FIXED_DOWN,
        CHANGE_POLICY_IN_FIXED_DOWN,
        BACK_TO_FIXED_STAND,
        ENTER_RL,
        DONE
    };

    struct TaskDurations {
        double prep_time;    
        double action_time;  
        double buffer_time;  
        double recover_time; 
    };
    TaskDurations task_durations_;
    TaskDurations policy_durations_;

    struct ObstacleSlot {
        std::string name;
        std::string action;
        int expected_task_id = -1;
        std::vector<double> nav_target;
        std::vector<double> visual_offset;
        bool is_obstacle = false;
        bool require_qr = false;
        bool require_visual_servo = false;
        std::string path_policy = "trot";
        std::string restore_policy = "none";
    };

    // --- 核心函数 ---
    void load_parameters();
    void control_loop();
    void publish_state_and_nav_enable();
    void publish_obstacle_sequence_context();
    void dispatch_current_state();
    void switch_state(State new_state);
    std::string state_to_string(State state);
    bool load_obstacle_sequence();
    bool validate_obstacle_slot(const ObstacleSlot& slot) const;
    const ObstacleSlot* active_obstacle_slot() const;
    const ObstacleSlot* obstacle_slot_for_task_id(int task_id) const;
    bool obstacle_sequence_finished() const;
    void advance_obstacle_slot();
    void finish_obstacle_sequence_task();
    void switch_state_from_obstacle_action(const ObstacleSlot& slot);

    // --- 各上层状态周期处理 ---
    void handle_idle_state();
    void handle_auto_nav_state();
    void handle_manual_nav_state();
    void handle_qr_recognition_state();
    void handle_crawl_cross_state();
    void handle_crawl_moving_state();
    void handle_stair_up_state();
    void handle_stair_moving_state();
    void handle_stair_down_state();
    void handle_path_policy_prep_state();
    void handle_path_policy_restore_state();
    void handle_pole_around_state();
    void handle_wall_cross_state();
    void handle_wall_policy_restore_state();
    void handle_sand_inout_state();
    void handle_bridge_cross_state();
    void handle_logistics_init_manip_state();
    void handle_logistics_pre_scan_state();
    void handle_logistics_lift_arm_state();
    void handle_logistics_wait_remote_start_state();
    void handle_logistics_nav_state();
    void handle_pickup_block_state();
    void handle_dropoff_block_state();
    void handle_logistics_done_state();
    void handle_other_state();

    // --- task_done 分支 ---
    void switch_state_from_task_id(int task_id);
    void on_executor_moving_section_done();
    void on_generic_task_done();

    // --- ROS 2 回调函数 ---
    void state_cmd_callback(const quad::msg::StateCommand::SharedPtr msg);
    void nav_status_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void nav_finished_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void state_array_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg); 
    void task_done_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void logistics_nav_event_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void manipulator_result_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void suction_detect_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void debug_cmd_callback(const std_msgs::msg::String::SharedPtr msg);
    bool body_motion_stopped() const;
    void proceed_after_manip_result(bool is_pickup);
    
    // --- 任务时序执行逻辑 ---
    void execute_complex_task(LowerEvent action); 
    void execute_manipulation_task(ManipulationType type);
    void publish_manipulator_init_cmd();
    void publish_manipulator_cmd(uint8_t cmd, const char * context);
    void publish_logistics_pickup_reached(bool reached);
    void publish_zero_cmd_vel();
    void enter_manip_phase(ManipPhase phase);
    bool change_policy(LowerEvent policy, int expected_policy = -1);
    bool run_kneel_policy_transition(LowerEvent policy, int expected_policy, const char * context);
    bool policy_from_name(const std::string& name, LowerEvent& event, int& expected_policy) const;
    void send_lower_command(LowerEvent event);
    void makesure_lowerstate();

    // --- 状态与数据成员 ---
    State current_state_;
    State nav_state; 
    std::string mission_mode_;
    bool is_logistics_mode_ = false;
    bool obstacle_sequence_enabled_ = false;
    bool obstacle_sequence_valid_ = false;
    std::vector<ObstacleSlot> obstacle_slots_;
    int active_obstacle_index_ = 0;
    int active_obstacle_segment_ = 0;
    std::string active_task_slot_name_;
    std::string active_path_policy_ = "trot";
    std::string active_path_restore_policy_ = "none";

    // 底层状态反馈
    bool is_auto_mode_ = false;
    int lower_state_ = -1;       
    int lower_policy_ = -1;      
    int expected_lower_state_ = -1; 

    // 复杂任务序列控制
    int sub_step_ = 0;
    int crawl_phase_ = 0;
    KneelPolicyPhase kneel_policy_phase_ = KneelPolicyPhase::TO_FIXED_STAND;
    rclcpp::Time step_start_time_;
    rclcpp::Time policy_confirm_start_time_;
    rclcpp::Time last_cmd_time_; 
    LowerEvent last_sent_lower_code_ = static_cast<LowerEvent>(-1); 
    double policy_confirm_settle_s_ = 0.12;
    bool policy_confirm_seen_ = false;

    // 任务赛取放物块
    ManipPhase manip_phase_ = ManipPhase::STOP_NAV;
    rclcpp::Time manip_phase_start_time_;
    bool manip_command_sent_ = false;
    bool manip_result_received_ = false;
    int last_logistics_event_seq_ = -1;
    std::string manipulator_command_topic_ = "/manipulator/cmd";
    bool manip_init_on_logistics_start_ = true;
    int manip_init_cmd_ = 0;
    int lift_standby_cmd_ = 4;
    bool place_low_after_init_ = true;
    double init_to_place_low_wait_s_ = 1.0;
    double after_place_low_wait_s_ = 2.0;
    double lift_standby_wait_s_ = 1.0;
    bool logistics_lift_cmd_sent_ = false;
    rclcpp::Time logistics_lift_start_time_;
    LogisticsInitPhase logistics_init_phase_ = LogisticsInitPhase::INIT_CMD;
    rclcpp::Time logistics_init_phase_start_time_;
    int pickup_cmd_ = 1;
    int place_low_cmd_ = 2;
    int place_high_cmd_ = 3;
    int dropoff_high_layer_min_index_ = 4;
    int active_manip_cmd_ = 0;
    bool active_dropoff_high_layer_ = false;
    bool pickup_dock_command_sent_ = false;
    int pickup_dock_waypoint_index_ = -1;
    double pickup_wait_s_ = 4.0;
    double dropoff_wait_s_ = 4.0;
    double pre_stand_wait_s_ = 0.5;
    double arm_cmd_settle_s_ = 0.5;
    double stand_settle_s_ = 0.5;
    double rl_settle_s_ = 0.3;
    double lower_state_timeout_s_ = 3.0;
    bool manipulation_enabled_ = true;
    double disabled_result_wait_s_ = 0.5;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    bool cmd_vel_received_ = false;

    // 吸取失败重试
    bool suction_detected_ = false;         // 最近一次 suction_detect 结果
    bool suction_failure_received_ = false;  // 是否收到过吸附失败信号
    bool suction_retry_active_ = false;      // 是否正在重试流程中（需要先站起再抓取）
    bool suction_retry_stand_enable_ = true; // 重试时是否先切站立
    int suction_failure_count_ = 0;          // 当前方块的累计重试次数
    int suction_max_retries_ = 3;            // 最大重试次数（从参数加载）

    // --- ROS 2 通信接口 ---
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_enable_pub_; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr logistics_nav_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr logistics_pickup_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lower_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr manipulator_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_nav_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacle_active_index_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacle_active_segment_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_current_slot_pub_;
    
    rclcpp::Subscription<quad::msg::StateCommand>::SharedPtr state_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_status_sub_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_finished_sub_; 
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr state_array_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr logistics_nav_event_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manipulator_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_detect_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

#endif // TASK_STATE_MACHINE_HPP_
