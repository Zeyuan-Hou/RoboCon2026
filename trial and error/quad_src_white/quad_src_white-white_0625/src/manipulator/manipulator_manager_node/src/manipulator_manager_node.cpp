/**
 * @file manipulator_manager_node.cpp
 * @brief 机械臂管理节点 - 基于时间的分段控制
 *
 * 功能说明：
 * - 协调arm节点和pump节点
 * - 实现抓取/放置/叠放动作流程
 * - 状态推进基于时间（分段函数）
 * - 超时保护机制
 *
 * 订阅话题：
 * - /manipulator/cmd : 顶层控制命令 (std_msgs/UInt8: 0=INIT_STANDBY, 1=GRAB, 2=PLACE_LOW, 3=PLACE_HIGH, 4=LIFT_STANDBY, 5=DISABLE)
 *
 * 发布话题：
 * - /quad/manipulator/state_cmd : 机械臂目标状态命令 (std_msgs/UInt8)
 * - /quad/arm/disable : 失能信号 (std_msgs/Bool: true=禁用机械臂电流)
 * - /pump_cmd : 气泵控制命令 (std_msgs/Int8)
 * - /manipulator/state : 当前状态 (std_msgs/String)
 * - /manipulator/result : 任务结果 (std_msgs/Bool)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mutex>

// 机械臂状态
enum ArmState : uint8_t {
    ARM_NONE = 0,        ///< 未初始化/安全状态
    ARM_GRAB = 1,        ///< 抓取物块状态
    ARM_LIFT = 2,        ///< 抬起物块状态
    ARM_PLACE_LOW = 3,   ///< 低层放置状态
    ARM_PLACE_HIGH = 4,  ///< 高层放置(叠放)状态
    ARM_LAY = 5,         ///< 平放状态
    ARM_TRACKING = 6     ///< 轨迹跟踪状态（manager通知arm开始轨迹跟踪的命令）
};

// 气泵命令
enum PumpCommand : int8_t {
    PUMP_STOP = 0,
    PUMP_GRAB = 1,
    PUMP_RELEASE = 2
};

// 任务阶段
enum TaskPhase {
    PHASE_IDLE = 0,
    PHASE_RUNNING,
    PHASE_DONE,
    PHASE_ERROR
};

// 任务类型
enum TaskType {
    TASK_INIT_STANDBY = 0,
    TASK_GRAB,
    TASK_PLACE_LOW,
    TASK_PLACE_HIGH,
    TASK_LIFT_STANDBY,
    TASK_DISABLE        ///< 失能任务：电流归零+关闭气泵，可抢占任何任务
};

class ManipulatorManagerNode : public rclcpp::Node {
   public:
    ManipulatorManagerNode() : Node("manipulator_manager_node"),
        current_phase_(PHASE_IDLE),
        current_task_(TASK_INIT_STANDBY),
        task_start_time_(this->now()),
        grab_phase_start_ms_(this->now()) {

        // 声明参数 - 各阶段时间配置(ms)
        // GRAB任务: Phase0->ARM_GRAB+PUMP_STOP等待机械臂到达, Phase1->等待approach_ready, Phase2->PUMP_GRAB, Phase3->ARM_LIFT+检测
        this->declare_parameter("arm_arrival_timeout_ms", 3000);  // 机械臂到达超时时间(ms)
        this->declare_parameter("approach_ready_timeout_ms", 10000);  // 等待approach_ready超时时间(ms)
        this->declare_parameter("trajectory_complete_timeout_ms", 15000);  // 等待轨迹完成超时时间(ms)
        this->declare_parameter("grab_pump_time_ms", 500);      // 气泵吸合时间
        this->declare_parameter("grab_lift_time_ms", 1500);     // 机械臂移动到LIFT时间

        // PLACE_LOW任务: 0-2000ms->PLACE_LOW位置, 2000-2500ms->释放, 2500-4500ms->LIFT位置
        this->declare_parameter("place_low_arm_time_ms", 2000);     // 机械臂移动到PLACE_LOW时间
        this->declare_parameter("place_low_release_time_ms", 500);  // 气泵释放时间
        this->declare_parameter("place_low_lift_time_ms", 2000);    // 机械臂移动到LIFT时间

        // PLACE_HIGH任务: 0-1500ms->PLACE_HIGH位置, 1500-2500ms->释放, 2500-4000ms->LIFT位置
        this->declare_parameter("place_high_arm_time_ms", 1500);     // 机械臂移动到PLACE_HIGH时间
        this->declare_parameter("place_high_release_time_ms", 1000);  // 气泵释放时间
        this->declare_parameter("place_high_lift_time_ms", 1500);    // 机械臂移动到LIFT时间

        // INIT_STANDBY任务: 移动到PLACE_LOW待机位置并保持目标，不执行释放/抬起流程
        this->declare_parameter("init_standby_time_ms", 1000);

        // LIFT_STANDBY任务: 移动到LIFT导航待机位置并保持目标，不执行抓取/释放流程
        this->declare_parameter("lift_standby_time_ms", 1000);

        // 吸取检测参数
        this->declare_parameter("suction_detect_enable", true);        // 是否启用吸取检测
        this->declare_parameter("suction_work_threshold", 150000.0);   // 吸取做功阈值
        this->declare_parameter("suction_pos_threshold", 0.3);         // 大臂位置阈值（低于此值认为无物块）
        this->declare_parameter("suction_pos_debounce_ms", 50.0);      // 位置消抖窗口(ms)

        this->declare_parameter("control_freq_hz", 100.0);

        control_freq_ = this->get_parameter("control_freq_hz").as_double();

        // 创建订阅器
        cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/manipulator/cmd", 10,
            std::bind(&ManipulatorManagerNode::cmd_callback, this, std::placeholders::_1));

        arm_motor_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/quad/arm/motor_state", 10,
            std::bind(&ManipulatorManagerNode::arm_motor_state_callback, this, std::placeholders::_1));

        // 订阅机械臂到达信号（上升沿检测）
        arm_arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/quad/arm/arrived", 10,
            std::bind(&ManipulatorManagerNode::arm_arrived_callback, this, std::placeholders::_1));

        // 订阅外部approach_ready信号（上升沿检测）
        approach_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/quad/logistics_pickup_reached", 10,
            std::bind(&ManipulatorManagerNode::approach_ready_callback, this, std::placeholders::_1));

        // 订阅机械臂轨迹完成信号（上升沿检测）
        trajectory_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/quad/arm/trajectory_complete", 10,
            std::bind(&ManipulatorManagerNode::trajectory_complete_callback, this, std::placeholders::_1));

        // 创建发布器
        arm_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/quad/arm/state_cmd", 10);

        pump_cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>(
            "/quad/pump_cmd", 10);

                arm_disable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/quad/arm/disable", rclcpp::QoS(1).transient_local());

        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/manipulator/state", 10);

        result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/manipulator/result", 10);

        suction_detect_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/manipulator/suction_detect", 10);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_freq_),
            std::bind(&ManipulatorManagerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Manipulator Manager Node started (time-based control)");
    }

   private:
    TaskPhase current_phase_;
    TaskType current_task_;
    rclcpp::Time task_start_time_;
    double control_freq_;

    // 吸取检测相关
    bool suction_detected_ = false;           ///< 是否检测到成功吸取
    bool suction_detection_active_ = false;   ///< 是否正在检测
    double mean_current_ = 0.0;               ///< 迭代平均电流
    double total_work_ = 0.0;                 ///< 累计做功（功率对时间积分）
    double latest_current_[2] = {0.0, 0.0};   ///< 最新电流值
    double latest_velocity_[2] = {0.0, 0.0};  ///< 最新速度值
    double latest_pos_[2] = {0.0, 0.0};       ///< 最新位置值（相对零位）
    std::mutex current_mutex_;                ///< 电流/位置数据互斥锁

    // GRAB子阶段状态
    int grab_sub_phase_ = 0;                  ///< GRAB子阶段（0/1/2/3）
    rclcpp::Time grab_phase_start_ms_;        ///< 当前子阶段起始时间

    // 上升沿检测
    bool arm_arrived_prev_ = false;           ///< 上一次arm_arrived值
    bool arm_arrived_rising_ = false;         ///< arm_arrived上升沿标志（读后清除）
    bool approach_ready_prev_ = false;        ///< 上一次approach_ready值
    bool approach_ready_level_ = false;       ///< 当前approach_ready电平
    bool approach_ready_rising_ = false;      ///< approach_ready上升沿标志（读后清除）
    bool trajectory_complete_prev_ = false;   ///< 上一次trajectory_complete值
    bool trajectory_complete_rising_ = false; ///< trajectory_complete上升沿标志（读后清除）

    // 位置消抖相关
    double lift_min_pos_ = 0.0;               ///< LIFT阶段motor0最小位置
    bool lift_pos_check_done_ = false;        ///< 位置消抖判定是否完成
    bool lift_has_block_ = false;             ///< 消抖后判定：下方是否有物块
    int lift_pos_below_count_ = 0;            ///< 连续低于阈值的采样计数
    int power_sample_count_ = 0;              ///< LIFT阶段功率采样数

    // ROS2通信
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_motor_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_arrived_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approach_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_complete_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pump_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_detect_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_disable_pub_;   ///< 失能信号发布到 arm_node_td
    rclcpp::TimerBase::SharedPtr timer_;

    bool disable_published_ = false;          ///< 进入 TASK_DISABLE 后是否已发布过失能信号（上升沿单发）

    void cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        TaskType task = static_cast<TaskType>(msg->data);

        // 抢占规则：常规任务运行时拒绝新指令，除非新指令是 DISABLE；
        // DISABLE 运行时允许任何新指令覆盖，避免卡死。
        if (current_phase_ == PHASE_RUNNING &&
            current_task_ != TASK_DISABLE &&
            task != TASK_DISABLE) {
            RCLCPP_WARN(this->get_logger(), "Task already running, ignoring command %d", msg->data);
            return;
        }

        if (task != TASK_INIT_STANDBY &&
            task != TASK_GRAB &&
            task != TASK_PLACE_LOW &&
            task != TASK_PLACE_HIGH &&
            task != TASK_LIFT_STANDBY &&
            task != TASK_DISABLE) {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %d", msg->data);
            return;
        }

        current_task_ = task;
        current_phase_ = PHASE_RUNNING;
        task_start_time_ = this->now();

        // 切换到非失能任务时，发布一次 false 让 arm_node 清掉失能锁存；
        // 切换到 TASK_DISABLE 时清掉 latch，以便下一次重新进入时能再次发布 true。
        if (task != TASK_DISABLE) {
            std_msgs::msg::Bool release_msg;
            release_msg.data = false;
            arm_disable_pub_->publish(release_msg);
        }
        disable_published_ = false;

        // 重置吸取检测状态
        suction_detected_ = false;
        suction_detection_active_ = false;
        mean_current_ = 0.0;
        total_work_ = 0.0;
        lift_min_pos_ = 0.0;
        lift_pos_check_done_ = false;
        lift_has_block_ = false;
        lift_pos_below_count_ = 0;
        power_sample_count_ = 0;

        // 重置GRAB子阶段状态
        grab_sub_phase_ = 0;
        grab_phase_start_ms_ = this->now();
        arm_arrived_prev_ = false;
        arm_arrived_rising_ = false;
        approach_ready_rising_ = false;
        trajectory_complete_prev_ = false;
        trajectory_complete_rising_ = false;

        RCLCPP_INFO(this->get_logger(), "Starting task %d", task);
        publish_state("RUNNING");
    }

    void arm_motor_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(current_mutex_);
        if (msg->data.size() >= 2) {
            latest_current_[0] = msg->data[0];
            latest_current_[1] = msg->data[1];
        }
        if (msg->data.size() >= 4) {
            latest_velocity_[0] = msg->data[2];
            latest_velocity_[1] = msg->data[3];
        }
        if (msg->data.size() >= 6) {
            latest_pos_[0] = msg->data[4];
            latest_pos_[1] = msg->data[5];
        }
    }

    void arm_arrived_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // 上升沿检测：从false变为true时置位标志
        if (msg->data && !arm_arrived_prev_) {
            arm_arrived_rising_ = true;
        }
        arm_arrived_prev_ = msg->data;
    }

    void approach_ready_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // 上升沿检测：从false变为true时置位标志
        if (msg->data && !approach_ready_prev_) {
            approach_ready_rising_ = true;
        }
        approach_ready_level_ = msg->data;
        approach_ready_prev_ = msg->data;
    }

    void trajectory_complete_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // 上升沿检测：从false变为true时置位标志
        if (msg->data && !trajectory_complete_prev_) {
            trajectory_complete_rising_ = true;
        }
        trajectory_complete_prev_ = msg->data;
    }

    void control_loop() {
        if (current_phase_ != PHASE_RUNNING) {
            return;
        }

        auto elapsed_ms = (this->now() - task_start_time_).seconds() * 1000;

        switch (current_task_) {
            case TASK_INIT_STANDBY:
                execute_init_standby_task(elapsed_ms);
                break;
            case TASK_GRAB:
                execute_grab_task(elapsed_ms);
                break;
            case TASK_PLACE_LOW:
                execute_place_low_task(elapsed_ms);
                break;
            case TASK_PLACE_HIGH:
                execute_place_high_task(elapsed_ms);
                break;
            case TASK_LIFT_STANDBY:
                execute_lift_standby_task(elapsed_ms);
                break;
            case TASK_DISABLE:
                execute_disable_task();
                break;
            default:
                break;
        }
    }

    void execute_init_standby_task(double elapsed_ms) {
        int standby_time = this->get_parameter("init_standby_time_ms").as_int();

        send_arm_cmd(ARM_LAY);
        send_pump_cmd(PUMP_STOP);

        if (elapsed_ms >= standby_time) {
            finish_task(true);
        }
    }

    void execute_lift_standby_task(double elapsed_ms) {
        int standby_time = this->get_parameter("lift_standby_time_ms").as_int();

        send_arm_cmd(ARM_LIFT);
        send_pump_cmd(PUMP_STOP);

        if (elapsed_ms >= standby_time) {
            finish_task(true);
        }
    }

    void execute_disable_task() {
        // 仅在进入失能时发一次失能信号（话题为 TRANSIENT_LOCAL 持久化），保持订阅者侧只触发一次。
        // 每拍仍发送 PUMP_STOP（pump_node 内部有 500ms 速率限制，正常情况只生效一次）。
        if (!disable_published_) {
            std_msgs::msg::Bool disable_msg;
            disable_msg.data = true;
            arm_disable_pub_->publish(disable_msg);
            disable_published_ = true;
            RCLCPP_INFO(this->get_logger(), "DISABLE task engaged: arm current=0, pump OFF");
        }
        send_pump_cmd(PUMP_STOP);
    }

    void execute_grab_task(double /*elapsed_ms*/) {
        int lift_time = this->get_parameter("grab_lift_time_ms").as_int();
        int arrival_timeout_ms = this->get_parameter("arm_arrival_timeout_ms").as_int();
        bool detect_enable = this->get_parameter("suction_detect_enable").as_bool();
        double work_threshold = this->get_parameter("suction_work_threshold").as_double();
        auto phase_elapsed_ms = (this->now() - grab_phase_start_ms_).seconds() * 1000.0;

        if (grab_sub_phase_ == 0) {
            // Phase 0: 发送ARM_GRAB + PUMP_STOP，等待arm_arrived上升沿
            send_arm_cmd(ARM_GRAB);
            send_pump_cmd(PUMP_STOP);

            if (arm_arrived_rising_) {
                arm_arrived_rising_ = false;  // 清除标志
                grab_sub_phase_ = 1;
                grab_phase_start_ms_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Phase 0 done: arm arrived, waiting for approach_ready");
            } else if (phase_elapsed_ms >= arrival_timeout_ms) {
                // 超时也进入下一阶段
                grab_sub_phase_ = 1;
                grab_phase_start_ms_ = this->now();
                RCLCPP_WARN(this->get_logger(), "Phase 0 timeout after %dms, proceeding to Phase 1", arrival_timeout_ms);
            }
        } else if (grab_sub_phase_ == 1) {
            // Phase 1: 等待approach_ready电平/上升沿，收到后发送ARM_TRACKING
            send_arm_cmd(ARM_GRAB);
            send_pump_cmd(PUMP_STOP);

            int approach_timeout_ms = this->get_parameter("approach_ready_timeout_ms").as_int();
            if (approach_ready_rising_ || approach_ready_level_) {
                // 上升沿或当前已为true（防止信号早于Phase 1到达）
                approach_ready_rising_ = false;
                // 通知arm开始轨迹跟踪
                send_arm_cmd(ARM_TRACKING);
                grab_sub_phase_ = 2;
                grab_phase_start_ms_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Phase 1 done: approach_ready received, sent ARM_TRACKING, starting pump");
            } else if (phase_elapsed_ms >= approach_timeout_ms) {
                // 超时也发送ARM_TRACKING
                send_arm_cmd(ARM_TRACKING);
                grab_sub_phase_ = 2;
                grab_phase_start_ms_ = this->now();
                RCLCPP_WARN(this->get_logger(), "Phase 1 timeout after %dms, sent ARM_TRACKING anyway", approach_timeout_ms);
            }
        } else if (grab_sub_phase_ == 2) {
            // Phase 2: 启动吸泵 + 等待轨迹完成（grab_pump_time_ms 最小等待）
            send_pump_cmd(PUMP_GRAB);
            int tc_timeout_ms = this->get_parameter("trajectory_complete_timeout_ms").as_int();

            if (trajectory_complete_rising_ || trajectory_complete_prev_) {
                // 轨迹完成，进入Phase 3
                trajectory_complete_rising_ = false;
                grab_sub_phase_ = 3;
                grab_phase_start_ms_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Phase 2 done: trajectory_complete received, starting lift");
            } else if (phase_elapsed_ms >= tc_timeout_ms) {
                // 超时保护：轨迹完成信号超时，强制进入Phase 3
                grab_sub_phase_ = 3;
                grab_phase_start_ms_ = this->now();
                RCLCPP_WARN(this->get_logger(),
                    "Phase 2 timeout after %.0fms waiting for trajectory_complete, proceeding to lift anyway",
                    phase_elapsed_ms);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Phase 2: pump active, waiting for trajectory_complete (%.0fms/%dms)",
                    phase_elapsed_ms, tc_timeout_ms);
            }
        } else if (grab_sub_phase_ == 3) {
            // Phase 3: 抬臂（ARM_LIFT）+ 吸取检测（grab_lift_time_ms）
            send_arm_cmd(ARM_LIFT);
            send_pump_cmd(PUMP_GRAB);

            // 吸取检测逻辑（在LIFT全程计算电流和做功）
            if (detect_enable) {
                if (!suction_detection_active_) {
                    suction_detection_active_ = true;
                    mean_current_ = 0.0;
                    total_work_ = 0.0;
                    power_sample_count_ = 0;
                    lift_min_pos_ = 0.0;
                    lift_pos_check_done_ = false;
                    lift_has_block_ = false;
                    lift_pos_below_count_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Suction detection started");
                }

                // 采集电流样本并迭代计算算术平均，同时计算做功
                {
                    std::lock_guard<std::mutex> lock(current_mutex_);
                    double total_current = std::abs(latest_current_[0]) + std::abs(latest_current_[1]);
                    mean_current_ = (total_current + mean_current_) / 2.0;

                    // 计算瞬时功率 P = I * omega (电流 * 角速度)
                    // 使用绝对值确保功率为正
                    double power = std::abs(latest_current_[0] * latest_velocity_[0])
                                 + std::abs(latest_current_[1] * latest_velocity_[1]);
                    // 积分累加做功（dt = 10ms = 0.01s，控制频率100Hz）
                    total_work_ += power / control_freq_;
                    ++power_sample_count_;
                }

                // 位置消抖：判断下方是否有物块
                if (!lift_pos_check_done_) {
                    double pos_threshold = this->get_parameter("suction_pos_threshold").as_double();
                    double debounce_ms = this->get_parameter("suction_pos_debounce_ms").as_double();
                    double pos0;
                    {
                        std::lock_guard<std::mutex> lock(current_mutex_);
                        pos0 = latest_pos_[0];
                    }

                    // 更新最小位置
                    if (pos0 < lift_min_pos_ || power_sample_count_ == 1) {
                        lift_min_pos_ = pos0;
                    }

                    // 消抖判定：如果连续N个采样都低于阈值，判定无物块
                    int debounce_samples = static_cast<int>(debounce_ms * control_freq_ / 1000.0);
                    if (debounce_samples < 1) debounce_samples = 1;

                    if (pos0 < pos_threshold) {
                        lift_pos_below_count_++;
                    } else {
                        lift_pos_below_count_ = 0;  // 只要有一次高于阈值，重新计数
                    }

                    if (lift_pos_below_count_ >= debounce_samples) {
                        // 连续低于阈值超过消抖窗口 → 确认无物块
                        lift_has_block_ = false;
                        lift_pos_check_done_ = true;
                        RCLCPP_WARN(this->get_logger(),
                                    "Suction detect: NO BLOCK (pos %.2f below threshold %.2f for %d samples)",
                                    lift_min_pos_, pos_threshold, lift_pos_below_count_);
                    } else if (power_sample_count_ >= debounce_samples && pos0 >= pos_threshold) {
                        // 已采够消抖窗口的样本数，且当前位置高于阈值 → 有物块
                        lift_has_block_ = true;
                        lift_pos_check_done_ = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "Suction detect: BLOCK PRESENT (pos %.2f above threshold %.2f)",
                                    pos0, pos_threshold);
                    }
                }
            }

            if (phase_elapsed_ms >= lift_time) {
                // Phase 3完成：任务完成
                if (detect_enable && mean_current_ > 0) {
                    bool work_ok = total_work_ >= work_threshold;
                    bool pos_ok = lift_has_block_;

                    RCLCPP_INFO(this->get_logger(),
                                "Suction final check: work=%.2f/%.2f, pos_ok=%d, min_pos=%.2f, samples=%d",
                                total_work_, work_threshold, pos_ok, lift_min_pos_, power_sample_count_);

                    if (!pos_ok) {
                        RCLCPP_WARN(this->get_logger(),
                                    "Suction FAILED: no block detected (min_pos=%.2f)",
                                    lift_min_pos_);
                        publish_suction_detect(false);
                    } else if (!work_ok) {
                        RCLCPP_WARN(this->get_logger(),
                                    "Suction FAILED: work %.2f below threshold %.2f",
                                    total_work_, work_threshold);
                        publish_suction_detect(false);
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                                    "Suction SUCCESS: work %.2f, block present",
                                    total_work_);
                        publish_suction_detect(true);
                    }
                }
                finish_task(true);
            }
        }
    }

    void execute_place_low_task(double elapsed_ms) {
        int arm_time = this->get_parameter("place_low_arm_time_ms").as_int();
        int release_time = this->get_parameter("place_low_release_time_ms").as_int();
        int lift_time = this->get_parameter("place_low_lift_time_ms").as_int();
        int total_time = arm_time + release_time + lift_time;

        if (elapsed_ms < arm_time) {
            // 阶段1: 移动到PLACE_LOW位置
            send_arm_cmd(ARM_PLACE_LOW);
            send_pump_cmd(PUMP_GRAB);
        } else if (elapsed_ms < arm_time + release_time) {
            // 阶段2: 气泵释放
            send_arm_cmd(ARM_PLACE_LOW);
            send_pump_cmd(PUMP_RELEASE);
        } else if (elapsed_ms < total_time) {
            // 阶段3: 移动到LIFT位置
            send_arm_cmd(ARM_LIFT);
            send_pump_cmd(PUMP_STOP);
        } else {
            // 任务完成
            finish_task(true);
        }
    }

    void execute_place_high_task(double elapsed_ms) {
        int arm_time = this->get_parameter("place_high_arm_time_ms").as_int();
        int release_time = this->get_parameter("place_high_release_time_ms").as_int();
        int lift_time = this->get_parameter("place_high_lift_time_ms").as_int();
        int total_time = arm_time + release_time + lift_time;

        if (elapsed_ms < arm_time) {
            // 阶段1: 移动到PLACE_HIGH位置
            send_arm_cmd(ARM_PLACE_HIGH);
            send_pump_cmd(PUMP_GRAB);
        } else if (elapsed_ms < arm_time + release_time) {
            // 阶段2: 气泵释放
            send_arm_cmd(ARM_PLACE_HIGH);
            send_pump_cmd(PUMP_RELEASE);
        } else if (elapsed_ms < total_time) {
            // 阶段3: 移动到LIFT位置
            send_arm_cmd(ARM_LIFT);
            send_pump_cmd(PUMP_STOP);
        } else {
            // 任务完成
            finish_task(true);
        }
    }

    void finish_task(bool success) {
        current_phase_ = success ? PHASE_DONE : PHASE_ERROR;
        publish_result(success);
        publish_state(success ? "DONE" : "ERROR");

        // 任务完成后保持最后状态，不重置
        // 等待下一个命令到来时再重置
        current_phase_ = PHASE_IDLE;
        // current_task_ = TASK_NONE;
    }

    void publish_suction_detect(bool success) {
        std_msgs::msg::Bool msg;
        msg.data = success;
        suction_detect_pub_->publish(msg);
    }

    void send_arm_cmd(ArmState state) {
        std_msgs::msg::UInt8 msg;
        msg.data = static_cast<uint8_t>(state);
        arm_cmd_pub_->publish(msg);
    }

    void send_pump_cmd(PumpCommand cmd) {
        std_msgs::msg::Int8 msg;
        msg.data = static_cast<int8_t>(cmd);
        pump_cmd_pub_->publish(msg);
    }

    void publish_state(const std::string& state) {
        std_msgs::msg::String msg;
        msg.data = state;
        state_pub_->publish(msg);
    }

    void publish_result(bool success) {
        std_msgs::msg::Bool msg;
        msg.data = success;
        result_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Task %s", success ? "completed" : "failed");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
