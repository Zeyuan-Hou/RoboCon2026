#include "task_manager/task_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace {
constexpr double kBodyStopLinVelEps = 0.02;
constexpr double kBodyStopAngVelEps = 0.05;
constexpr int kLogisticsPreScanDone = 0;
constexpr int kLogisticsPickupReached = 1;
constexpr int kLogisticsDropoffReached = 2;
constexpr int kLogisticsFinished = 3;
constexpr int kLogisticsPickupDockEntered = 4;
constexpr int kLogisticsPreScanSpinDone = 5;
}  // namespace

/**
 * @brief 构造函数：初始化ROS 2节点、加载参数、建立发布器与订阅器
 */
TaskStateMachine::TaskStateMachine() : Node("task_manager_node"), current_state_(State::IDLE) {
    load_parameters();

    // 2. 初始化发布器
    state_pub_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
    lower_cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>("cmd_evt", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    nav_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_enable", 10);
    logistics_nav_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("logistics_nav_enable", 10);
    logistics_pickup_reached_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("logistics_pickup_reached", 10);
    manipulator_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>(manipulator_command_topic_, 10);
    obstacle_nav_target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "obstacle_sequence/nav_target", 10);
    obstacle_active_index_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "obstacle_sequence/active_index", 10);
    obstacle_active_segment_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "obstacle_sequence/active_segment", 10);
    obstacle_current_slot_pub_ = this->create_publisher<std_msgs::msg::String>(
        "obstacle_sequence/current_slot", 10);

    // 3. 初始化订阅器
    state_cmd_sub_ = this->create_subscription<quad::msg::StateCommand>(
        "task_state_command", 10, std::bind(&TaskStateMachine::state_cmd_callback, this, std::placeholders::_1));
    state_array_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
        "state_array", 10, std::bind(&TaskStateMachine::state_array_callback, this, std::placeholders::_1));
    nav_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "nav_status", 10, std::bind(&TaskStateMachine::nav_status_callback, this, std::placeholders::_1));
    nav_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "nav_finished", 10, std::bind(&TaskStateMachine::nav_finished_callback, this, std::placeholders::_1));
    debug_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "debug_state_cmd", 10, std::bind(&TaskStateMachine::debug_cmd_callback, this, std::placeholders::_1));
    task_done_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "task_done", 10, std::bind(&TaskStateMachine::task_done_callback, this, std::placeholders::_1));
    logistics_nav_event_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "logistics_nav_event", 10,
        std::bind(&TaskStateMachine::logistics_nav_event_callback, this, std::placeholders::_1));
    manipulator_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/manipulator/result", 10,
        std::bind(&TaskStateMachine::manipulator_result_callback, this, std::placeholders::_1));
    suction_detect_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/manipulator/suction_detect", 10,
        std::bind(&TaskStateMachine::suction_detect_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&TaskStateMachine::cmd_vel_callback, this, std::placeholders::_1));

    // 4. 设置控制定时器
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TaskStateMachine::control_loop, this));
    
    nav_state = State::IDLE;
    step_start_time_ = this->now();
    manip_phase_start_time_ = this->now();
    logistics_init_phase_start_time_ = this->now();
    logistics_lift_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Task State Machine (DES Manager) Initialized.");
}

/**
 * @brief 参数加载函数
 */
void TaskStateMachine::load_parameters() {
    this->declare_parameter("mission_mode", "obstacle");
    mission_mode_ = this->get_parameter("mission_mode").as_string();
    is_logistics_mode_ = (mission_mode_ == "logistics");

    this->declare_parameter("task_timers.prep", 2.0);
    this->declare_parameter("task_timers.action", 3.5);
    this->declare_parameter("task_timers.buffer", 1.5);
    this->declare_parameter("task_timers.recover", 1.0);

    task_durations_.prep_time = this->get_parameter("task_timers.prep").as_double();
    task_durations_.action_time = this->get_parameter("task_timers.action").as_double();
    task_durations_.buffer_time = this->get_parameter("task_timers.buffer").as_double();
    task_durations_.recover_time = this->get_parameter("task_timers.recover").as_double();

    this->declare_parameter("policy_timers.prep", 1.0);
    this->declare_parameter("policy_timers.action", 0.5);
    this->declare_parameter("policy_timers.buffer", 0.5);
    this->declare_parameter("policy_timers.recover", 1.0);
    this->declare_parameter("policy_timers.confirm_settle", 0.12);

    policy_durations_.prep_time = this->get_parameter("policy_timers.prep").as_double();
    policy_durations_.action_time = this->get_parameter("policy_timers.action").as_double();
    policy_durations_.buffer_time = this->get_parameter("policy_timers.buffer").as_double();
    policy_durations_.recover_time = this->get_parameter("policy_timers.recover").as_double();
    policy_confirm_settle_s_ =
        std::max(0.0, this->get_parameter("policy_timers.confirm_settle").as_double());

    this->declare_parameter("manipulation.command_topic", "/manipulator/cmd");
    this->declare_parameter("manipulation.enable", true);
    this->declare_parameter("manipulation.disabled_result_wait_s", 0.5);
    this->declare_parameter("manipulation.init_on_logistics_start", true);
    this->declare_parameter("manipulation.init_cmd", 0);
    this->declare_parameter("manipulation.lift_standby_cmd", 4);
    this->declare_parameter("manipulation.place_low_after_init", true);
    this->declare_parameter("manipulation.init_to_place_low_wait_s", 1.0);
    this->declare_parameter("manipulation.after_place_low_wait_s", 2.0);
    this->declare_parameter("manipulation.lift_standby_wait_s", 2.0);
    this->declare_parameter("manipulation.pickup_cmd", 1);
    this->declare_parameter("manipulation.place_low_cmd", 2);
    this->declare_parameter("manipulation.place_high_cmd", 3);
    this->declare_parameter("manipulation.dropoff_high_layer_min_index", 4);
    this->declare_parameter("manipulation.pickup_wait_s", 4.0);
    this->declare_parameter("manipulation.dropoff_wait_s", 4.0);
    this->declare_parameter("manipulation.pre_stand_wait_s", 0.5);
    this->declare_parameter("manipulation.arm_cmd_settle_s", 0.5);
    this->declare_parameter("manipulation.stand_settle_s", 0.5);
    this->declare_parameter("manipulation.rl_settle_s", 0.3);
    this->declare_parameter("manipulation.lower_state_timeout_s", 3.0);
    this->declare_parameter("manipulation.suction_retry_max", 3);
    this->declare_parameter("manipulation.suction_retry_stand_enable", true);

    manipulator_command_topic_ = this->get_parameter("manipulation.command_topic").as_string();
    manipulation_enabled_ = this->get_parameter("manipulation.enable").as_bool();
    disabled_result_wait_s_ = this->get_parameter("manipulation.disabled_result_wait_s").as_double();
    manip_init_on_logistics_start_ = this->get_parameter("manipulation.init_on_logistics_start").as_bool();
    manip_init_cmd_ = this->get_parameter("manipulation.init_cmd").as_int();
    lift_standby_cmd_ = this->get_parameter("manipulation.lift_standby_cmd").as_int();
    place_low_after_init_ = this->get_parameter("manipulation.place_low_after_init").as_bool();
    init_to_place_low_wait_s_ = this->get_parameter("manipulation.init_to_place_low_wait_s").as_double();
    after_place_low_wait_s_ = this->get_parameter("manipulation.after_place_low_wait_s").as_double();
    lift_standby_wait_s_ = this->get_parameter("manipulation.lift_standby_wait_s").as_double();
    pickup_cmd_ = this->get_parameter("manipulation.pickup_cmd").as_int();
    place_low_cmd_ = this->get_parameter("manipulation.place_low_cmd").as_int();
    place_high_cmd_ = this->get_parameter("manipulation.place_high_cmd").as_int();
    dropoff_high_layer_min_index_ =
        this->get_parameter("manipulation.dropoff_high_layer_min_index").as_int();
    pickup_wait_s_ = this->get_parameter("manipulation.pickup_wait_s").as_double();
    dropoff_wait_s_ = this->get_parameter("manipulation.dropoff_wait_s").as_double();
    pre_stand_wait_s_ = this->get_parameter("manipulation.pre_stand_wait_s").as_double();
    arm_cmd_settle_s_ = this->get_parameter("manipulation.arm_cmd_settle_s").as_double();
    stand_settle_s_ = this->get_parameter("manipulation.stand_settle_s").as_double();
    rl_settle_s_ = this->get_parameter("manipulation.rl_settle_s").as_double();
    lower_state_timeout_s_ = this->get_parameter("manipulation.lower_state_timeout_s").as_double();
    suction_max_retries_ = this->get_parameter("manipulation.suction_retry_max").as_int();
    suction_retry_stand_enable_ = this->get_parameter("manipulation.suction_retry_stand_enable").as_bool();

    RCLCPP_INFO(this->get_logger(),
                "Manipulation %s (disabled_result_wait_s=%.2f)",
                manipulation_enabled_ ? "ENABLED" : "DISABLED (dry-run, no arm commands)",
                disabled_result_wait_s_);

    obstacle_sequence_valid_ = load_obstacle_sequence();
}

bool TaskStateMachine::load_obstacle_sequence() {
    this->declare_parameter("obstacle_sequence.enabled", false);
    this->declare_parameter("obstacle_sequence.order", std::vector<std::string>{});
    obstacle_sequence_enabled_ = this->get_parameter("obstacle_sequence.enabled").as_bool();
    auto order = this->get_parameter("obstacle_sequence.order").as_string_array();

    obstacle_slots_.clear();
    active_obstacle_index_ = 0;
    active_obstacle_segment_ = 0;

    if (order.empty()) {
        if (obstacle_sequence_enabled_) {
            RCLCPP_ERROR(this->get_logger(), "obstacle_sequence.enabled=true but order is empty.");
            return false;
        }
        RCLCPP_WARN(this->get_logger(),
                    "obstacle_sequence.order is empty; legacy task_id action mapping is unavailable.");
        return true;
    }

    bool ok = true;
    std::map<int, std::string> task_id_to_slot;
    auto declare_if_missing = [this](const std::string& name, const auto& default_value) {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_value);
        }
    };
    for (const auto& name : order) {
        const std::string prefix = "obstacle_sequence." + name + ".";
        declare_if_missing(prefix + "action", "skip");
        declare_if_missing(prefix + "expected_task_id", -1);
        declare_if_missing(prefix + "nav_target", std::vector<double>{});
        declare_if_missing(prefix + "visual_offset", std::vector<double>{});

        ObstacleSlot slot;
        slot.name = name;
        slot.action = this->get_parameter(prefix + "action").as_string();

        const bool legacy_is_obstacle = (slot.action != "skip");
        declare_if_missing(prefix + "is_obstacle", legacy_is_obstacle);
        declare_if_missing(prefix + "require_qr", legacy_is_obstacle);
        declare_if_missing(prefix + "require_visual_servo", legacy_is_obstacle);
        declare_if_missing(prefix + "path_policy", "trot");
        declare_if_missing(prefix + "restore_policy", "none");

        slot.expected_task_id = this->get_parameter(prefix + "expected_task_id").as_int();
        slot.nav_target = this->get_parameter(prefix + "nav_target").as_double_array();
        slot.visual_offset = this->get_parameter(prefix + "visual_offset").as_double_array();
        slot.is_obstacle = this->get_parameter(prefix + "is_obstacle").as_bool();
        slot.require_qr = this->get_parameter(prefix + "require_qr").as_bool();
        slot.require_visual_servo = this->get_parameter(prefix + "require_visual_servo").as_bool();
        slot.path_policy = this->get_parameter(prefix + "path_policy").as_string();
        slot.restore_policy = this->get_parameter(prefix + "restore_policy").as_string();

        if (!validate_obstacle_slot(slot)) {
            ok = false;
        }
        if (slot.require_visual_servo) {
            auto inserted = task_id_to_slot.emplace(slot.expected_task_id, slot.name);
            if (!inserted.second && inserted.first->second != slot.name) {
                RCLCPP_ERROR(this->get_logger(),
                             "Duplicate expected_task_id=%d in obstacle_sequence slots '%s' and '%s'.",
                             slot.expected_task_id, inserted.first->second.c_str(), slot.name.c_str());
                ok = false;
            }
        }
        if (slot.nav_target.size() == 3) {
            RCLCPP_INFO(this->get_logger(),
                        "Obstacle sequence slot[%zu] '%s': target=(%.2f, %.2f, %.2f), "
                        "action='%s', is_obstacle=%s, require_qr=%s, require_visual_servo=%s, expected_task_id=%d",
                        obstacle_slots_.size(), slot.name.c_str(),
                        slot.nav_target[0], slot.nav_target[1], slot.nav_target[2],
                        slot.action.c_str(),
                        slot.is_obstacle ? "true" : "false",
                        slot.require_qr ? "true" : "false",
                        slot.require_visual_servo ? "true" : "false",
                        slot.expected_task_id);
        }
        obstacle_slots_.push_back(slot);
    }

    RCLCPP_INFO(this->get_logger(), "Obstacle sequence table %s with %zu slots; sequence driving is %s.",
                ok ? "loaded" : "has errors", obstacle_slots_.size(),
                obstacle_sequence_enabled_ ? "ENABLED" : "DISABLED");
    return ok;
}

bool TaskStateMachine::validate_obstacle_slot(const ObstacleSlot& slot) const {
    bool ok = true;
    if (slot.nav_target.size() != 3) {
        RCLCPP_ERROR(this->get_logger(),
                     "Obstacle slot '%s' must set nav_target: [x, y, yaw].",
                     slot.name.c_str());
        ok = false;
    }

    const std::vector<std::string> valid_actions = {
        "skip", "path", "stride", "crawl", "line", "stair_combo", "wall"
    };
    if (std::find(valid_actions.begin(), valid_actions.end(), slot.action) == valid_actions.end()) {
        RCLCPP_ERROR(this->get_logger(), "Obstacle slot '%s' has unknown action '%s'.",
                     slot.name.c_str(), slot.action.c_str());
        ok = false;
    }

    if (slot.action != "skip") {
        if (!slot.is_obstacle) {
            RCLCPP_ERROR(this->get_logger(),
                         "Obstacle slot '%s' has executable action '%s' but is_obstacle=false.",
                         slot.name.c_str(), slot.action.c_str());
            ok = false;
        }
    }

    if (!slot.is_obstacle) {
        if (slot.action != "skip") {
            RCLCPP_ERROR(this->get_logger(),
                         "Non-obstacle slot '%s' must use action='skip', got '%s'.",
                         slot.name.c_str(), slot.action.c_str());
            ok = false;
        }
        if (slot.require_qr || slot.require_visual_servo) {
            RCLCPP_ERROR(this->get_logger(),
                         "Non-obstacle slot '%s' cannot require QR or visual servo.",
                         slot.name.c_str());
            ok = false;
        }
    }

    if (slot.is_obstacle && slot.action == "skip") {
        RCLCPP_ERROR(this->get_logger(),
                     "Obstacle slot '%s' must use an executable action, got 'skip'.",
                     slot.name.c_str());
        ok = false;
    }

    if (slot.require_visual_servo && !slot.require_qr) {
        RCLCPP_ERROR(this->get_logger(),
                     "Obstacle slot '%s' cannot require visual servo without QR verification.",
                     slot.name.c_str());
        ok = false;
    }

    if (slot.require_qr && !slot.require_visual_servo) {
        RCLCPP_ERROR(this->get_logger(),
                     "Obstacle slot '%s' requires QR without visual servo; this flow is not supported.",
                     slot.name.c_str());
        ok = false;
    }

    if (slot.require_visual_servo) {
        if (slot.expected_task_id <= 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Obstacle slot '%s' requires expected_task_id > 0 when visual servo is enabled.",
                         slot.name.c_str());
            ok = false;
        }
        if (slot.visual_offset.size() != 3) {
            RCLCPP_ERROR(this->get_logger(),
                         "Obstacle slot '%s' requires visual_offset: [x_mm, y_mm, yaw_deg] when visual servo is enabled.",
                         slot.name.c_str());
            ok = false;
        }
    }

    if (slot.action == "path") {
        const std::vector<std::string> valid_path_policies = {"trot", "upstair", "kneel_crawl"};
        if (std::find(valid_path_policies.begin(), valid_path_policies.end(), slot.path_policy) ==
            valid_path_policies.end()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Obstacle slot '%s' path_policy must be 'trot', 'upstair', or 'kneel_crawl', got '%s'.",
                         slot.name.c_str(), slot.path_policy.c_str());
            ok = false;
        }

        const std::vector<std::string> valid_restore_policies = {"none", "trot"};
        if (std::find(valid_restore_policies.begin(), valid_restore_policies.end(), slot.restore_policy) ==
            valid_restore_policies.end()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Obstacle slot '%s' restore_policy must be 'none' or 'trot', got '%s'.",
                         slot.name.c_str(), slot.restore_policy.c_str());
            ok = false;
        }
    }
    return ok;
}

/**
 * @brief 底层状态数组回调函数
 */
void TaskStateMachine::state_array_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    if (!msg->data.empty()) {
        is_auto_mode_ = (msg->data[0] == 1); 
        lower_state_ = msg->data[1];
        lower_policy_ = msg->data[2];
    }
}

/**
 * @brief 发布当前上层状态与导航使能
 *
 * 输入：current_state_
 * 输出：current_state 话题；AUTO_NAV 时 nav_enable=true
 * 处理：字符串化状态并 publish；nav_enable 仅 AUTO_NAV 为 true
 */
void TaskStateMachine::publish_state_and_nav_enable() {
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    if (nav_enable_pub_) {
        std_msgs::msg::Bool nav_msg;
        nav_msg.data = (current_state_ == State::AUTO_NAV);
        nav_enable_pub_->publish(nav_msg);
    }
    if (logistics_nav_enable_pub_) {
        std_msgs::msg::Bool logistics_nav_msg;
        logistics_nav_msg.data =
            (current_state_ == State::LOGISTICS_PRE_SCAN ||
             current_state_ == State::LOGISTICS_NAV);
        logistics_nav_enable_pub_->publish(logistics_nav_msg);
    }
    publish_obstacle_sequence_context();
}

const TaskStateMachine::ObstacleSlot* TaskStateMachine::active_obstacle_slot() const {
    if (!obstacle_sequence_enabled_ || !obstacle_sequence_valid_) {
        return nullptr;
    }
    if (active_obstacle_index_ < 0 ||
        static_cast<std::size_t>(active_obstacle_index_) >= obstacle_slots_.size()) {
        return nullptr;
    }
    return &obstacle_slots_[active_obstacle_index_];
}

const TaskStateMachine::ObstacleSlot* TaskStateMachine::obstacle_slot_for_task_id(int task_id) const {
    if (!obstacle_sequence_valid_) {
        return nullptr;
    }
    for (const auto& slot : obstacle_slots_) {
        if (slot.action != "skip" && slot.expected_task_id == task_id) {
            return &slot;
        }
    }
    return nullptr;
}

bool TaskStateMachine::obstacle_sequence_finished() const {
    return obstacle_sequence_enabled_ &&
           static_cast<std::size_t>(std::max(active_obstacle_index_, 0)) >= obstacle_slots_.size();
}

void TaskStateMachine::publish_obstacle_sequence_context() {
    if (!obstacle_sequence_enabled_ || !obstacle_sequence_valid_) {
        return;
    }

    std_msgs::msg::Int32 index_msg;
    index_msg.data = active_obstacle_index_;
    obstacle_active_index_pub_->publish(index_msg);

    std_msgs::msg::Int32 segment_msg;
    segment_msg.data = active_obstacle_segment_;
    obstacle_active_segment_pub_->publish(segment_msg);

    std_msgs::msg::String slot_msg;
    const auto* slot = active_obstacle_slot();
    slot_msg.data = slot ? slot->name : "DONE";
    obstacle_current_slot_pub_->publish(slot_msg);

    if (slot && slot->nav_target.size() == 3) {
        std_msgs::msg::Float32MultiArray target_msg;
        target_msg.data = {
            static_cast<float>(active_obstacle_index_),
            static_cast<float>(slot->nav_target[0]),
            static_cast<float>(slot->nav_target[1]),
            static_cast<float>(slot->nav_target[2]),
        };
        obstacle_nav_target_pub_->publish(target_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Active obstacle target: index=%d slot='%s' target=(%.2f, %.2f, %.2f), "
            "action='%s', is_obstacle=%s, require_qr=%s, require_visual_servo=%s",
            active_obstacle_index_, slot->name.c_str(),
            slot->nav_target[0], slot->nav_target[1], slot->nav_target[2],
            slot->action.c_str(),
            slot->is_obstacle ? "true" : "false",
            slot->require_qr ? "true" : "false",
            slot->require_visual_servo ? "true" : "false");
    }
}

void TaskStateMachine::advance_obstacle_slot() {
    if (!obstacle_sequence_enabled_) {
        return;
    }
    active_obstacle_index_++;
    active_obstacle_segment_ = 0;
    sub_step_ = 0;
    crawl_phase_ = 0;
    active_task_slot_name_.clear();
    active_path_policy_ = "trot";
    active_path_restore_policy_ = "none";
    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
    expected_lower_state_ = -1;
    RCLCPP_INFO(this->get_logger(), "Advanced obstacle sequence to slot index %d.", active_obstacle_index_);
}

void TaskStateMachine::finish_obstacle_sequence_task() {
    if (!obstacle_sequence_enabled_) {
        switch_state(nav_state);
        return;
    }
    advance_obstacle_slot();
    if (obstacle_sequence_finished()) {
        RCLCPP_INFO(this->get_logger(), "Obstacle sequence finished. Switching to IDLE.");
        switch_state(State::IDLE);
    } else {
        switch_state(State::AUTO_NAV);
    }
}

/**
 * @brief IDLE 状态周期处理
 *
 * 输入：lower_state_
 * 输出：可能发送 UP_DOWN 底层事件
 * 处理：非 FIXED_STAND 时请求站立
 */
void TaskStateMachine::handle_idle_state() {
    if (lower_state_ != static_cast<int>(LowerStateID::FIXED_STAND)) {
        send_lower_command(LowerEvent::UP_DOWN);
    }
}

/**
 * @brief AUTO_NAV 状态周期处理
 *
 * 输入：无
 * 输出：nav_state=AUTO_NAV；先确保 RL_MOVE，再发送 ENTER_AUTO
 * 处理：记录导航回退状态；站立/蹲下恢复后先回 RL_MOVE，速度控制才会生效
 */
void TaskStateMachine::handle_auto_nav_state() {
    if (obstacle_sequence_enabled_) {
        if (!obstacle_sequence_valid_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "Obstacle sequence config invalid. Holding IDLE.");
            switch_state(State::IDLE);
            return;
        }
        if (obstacle_sequence_finished()) {
            switch_state(State::IDLE);
            return;
        }
    }
    nav_state = State::AUTO_NAV;
    if (lower_state_ != static_cast<int>(LowerStateID::RL_MOVE)) {
        if (last_sent_lower_code_ == LowerEvent::ENTER_RL && expected_lower_state_ == -1) {
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "AUTO_NAV waiting for RL_MOVE before ENTER_AUTO.");
        send_lower_command(LowerEvent::ENTER_RL);
        return;
    }
    send_lower_command(LowerEvent::ENTER_AUTO);
}

/**
 * @brief MANUAL_NAV 状态周期处理
 *
 * 输入：无
 * 输出：nav_state=MANUAL_NAV
 * 处理：仅更新 nav_state，速度由遥控接管
 */
void TaskStateMachine::handle_manual_nav_state() {
    nav_state = State::MANUAL_NAV;
}

/**
 * @brief QR_RECOGNITION 状态周期处理
 *
 * 输入：无
 * 输出：发送 ENTER_RL
 * 处理：切 RL 模式，cmd_vel 由 task_executor 视觉伺服发布
 */
void TaskStateMachine::handle_qr_recognition_state() {
    send_lower_command(LowerEvent::ENTER_RL);
}

/**
 * @brief TASK_CRAWL_CROSS 状态周期处理（匍匐策略时序）
 *
 * 输入：crawl_phase_、lower_state_、lower_policy_、sub_step_
 * 输出：可能切 TASK_CRAWL_MOVING 或回到 nav_state
 * 处理：phase0 进 RL+CREEP；phase2 切回 TROT 后恢复导航
 */
void TaskStateMachine::handle_crawl_cross_state() {
    switch (crawl_phase_) {
        case 0:
            if (lower_state_ != static_cast<int>(LowerStateID::RL_MOVE)) {
                if (last_sent_lower_code_ == LowerEvent::ENTER_RL && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::ENTER_RL);
                break;
            }
            if (change_policy(LowerEvent::POLICY_CREEP, 1)) {
                switch_state(State::TASK_CRAWL_MOVING);
                RCLCPP_INFO(this->get_logger(), "Entered CREEP policy, moving to BLIND_CRAWL.");
            }
            break;
        case 2:
            if (change_policy(LowerEvent::POLICY_TROT, 0)) {
                RCLCPP_INFO(this->get_logger(), "Crawl task finished. Back to NAV.");
                crawl_phase_ = 0;
                finish_obstacle_sequence_task();
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            }
            break;
    }
}

/**
 * @brief TASK_CRAWL_MOVING 状态周期处理
 *
 * 输入：无
 * 输出：无（执行器控制 cmd_vel）
 * 处理：静默等待 task_done
 */
void TaskStateMachine::handle_crawl_moving_state() {
}

/**
 * @brief TASK_STAIR_UP 状态周期处理（上楼梯策略时序）
 *
 * 输入：crawl_phase_（复用）、lower_policy_
 * 输出：可能切 TASK_STAIR_MOVING 或回到 nav_state
 * 处理：phase0 切 UPSTAIR；phase2 切回 TROT
 */
void TaskStateMachine::handle_stair_up_state() {
    switch (crawl_phase_) {
        case 0:
            if (change_policy(LowerEvent::POLICY_UPSTAIR, 2)) {
                switch_state(State::TASK_STAIR_MOVING);
                RCLCPP_INFO(this->get_logger(), "Entered UPSTAIR policy, moving to STAIR_MOVING.");
            }
            break;
        case 2:
            if (change_policy(LowerEvent::POLICY_TROT, 0)) {
                RCLCPP_INFO(this->get_logger(), "Stair Up task finished. Back to NAV.");
                crawl_phase_ = 0;
                finish_obstacle_sequence_task();
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            }
            break;
    }
}

/**
 * @brief TASK_STAIR_MOVING 状态周期处理
 *
 * 输入：无
 * 输出：无
 * 处理：静默等待执行器上楼段完成
 */
void TaskStateMachine::handle_stair_moving_state() {
}

void TaskStateMachine::handle_stair_down_state() {
    if (change_policy(LowerEvent::POLICY_TROT, 0)) {
        switch_state(State::TASK_BRIDGE_CROSS);
        RCLCPP_INFO(this->get_logger(), "Entered TROT policy, moving to stair down line segment.");
    }
}

void TaskStateMachine::handle_path_policy_prep_state() {
    LowerEvent policy_event;
    int expected_policy = -1;
    if (!policy_from_name(active_path_policy_, policy_event, expected_policy)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Unsupported path_policy '%s' for slot '%s'. Stop.",
                     active_path_policy_.c_str(), active_task_slot_name_.c_str());
        switch_state(State::IDLE);
        return;
    }

    if (active_path_policy_ == "kneel_crawl") {
        if (run_kneel_policy_transition(policy_event, expected_policy, "enter kneel_crawl path policy")) {
            RCLCPP_INFO(this->get_logger(),
                        "Entered %s policy for path slot '%s'. Starting path replay.",
                        active_path_policy_.c_str(), active_task_slot_name_.c_str());
            switch_state(State::TASK_POLE_AROUND);
        }
        return;
    }

    if (change_policy(policy_event, expected_policy)) {
        RCLCPP_INFO(this->get_logger(),
                    "Entered %s policy for path slot '%s'. Starting path replay.",
                    active_path_policy_.c_str(), active_task_slot_name_.c_str());
        switch_state(State::TASK_POLE_AROUND);
    }
}

void TaskStateMachine::handle_path_policy_restore_state() {
    LowerEvent policy_event;
    int expected_policy = -1;
    if (!policy_from_name(active_path_restore_policy_, policy_event, expected_policy)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Unsupported restore_policy '%s' for slot '%s'. Stop.",
                     active_path_restore_policy_.c_str(), active_task_slot_name_.c_str());
        switch_state(State::IDLE);
        return;
    }

    if (active_path_policy_ == "kneel_crawl" && active_path_restore_policy_ == "trot") {
        if (run_kneel_policy_transition(policy_event, expected_policy, "restore trot after kneel_crawl path")) {
            RCLCPP_INFO(this->get_logger(),
                        "Restored %s policy after path slot '%s'.",
                        active_path_restore_policy_.c_str(), active_task_slot_name_.c_str());
            finish_obstacle_sequence_task();
        }
        return;
    }

    if (change_policy(policy_event, expected_policy)) {
        RCLCPP_INFO(this->get_logger(),
                    "Restored %s policy after path slot '%s'.",
                    active_path_restore_policy_.c_str(), active_task_slot_name_.c_str());
        finish_obstacle_sequence_task();
    }
}

/**
 * @brief TASK_POLE_AROUND 状态周期处理
 *
 * 输入：无
 * 输出：ENTER_RL
 * 处理：进入 RL，轨迹跟踪由执行器完成
 */
void TaskStateMachine::handle_pole_around_state() {
    send_lower_command(LowerEvent::ENTER_RL);
}

/**
 * @brief TASK_WALL_CROSS 状态周期处理
 *
 * 输入：sub_step_、task_durations_
 * 输出：execute_complex_task(JUMP) 驱动底层动作序列
 * 处理：站立-跳跃-恢复后回到 nav_state
 */
void TaskStateMachine::handle_wall_cross_state() {
    execute_complex_task(LowerEvent::JUMP);
}

void TaskStateMachine::handle_wall_policy_restore_state() {
    if (change_policy(LowerEvent::POLICY_TROT, 0)) {
        RCLCPP_INFO(this->get_logger(), "Wall jump finished. Restored POLICY_TROT before returning home.");
        finish_obstacle_sequence_task();
        last_sent_lower_code_ = static_cast<LowerEvent>(-3);
    }
}

/**
 * @brief TASK_SAND_INOUT 状态周期处理
 *
 * 输入：sub_step_、task_durations_
 * 输出：execute_complex_task(STRIDE)
 * 处理：跨步动作序列后回到 nav_state
 */
void TaskStateMachine::handle_sand_inout_state() {
    execute_complex_task(LowerEvent::STRIDE);
}

/**
 * @brief TASK_BRIDGE_CROSS 状态周期处理
 *
 * 输入：无
 * 输出：ENTER_RL
 * 处理：RL 模式下由执行器雷达直线过桥
 */
void TaskStateMachine::handle_bridge_cross_state() {
    send_lower_command(LowerEvent::ENTER_RL);
}

/**
 * @brief 任务赛开始前机械臂初始化状态
 *
 * 输入：logistics_init_phase_
 * 输出：向机械臂命令话题发布 init_cmd；等待低层待机稳定
 * 处理：非阻塞执行，完成后进入 LOGISTICS_PRE_SCAN。
 */
void TaskStateMachine::handle_logistics_init_manip_state() {
    const double elapsed = (this->now() - logistics_init_phase_start_time_).seconds();

    switch (logistics_init_phase_) {
        case LogisticsInitPhase::INIT_CMD:
            publish_manipulator_init_cmd();
            logistics_init_phase_ = LogisticsInitPhase::WAIT_AFTER_INIT;
            logistics_init_phase_start_time_ = this->now();
            break;

        case LogisticsInitPhase::WAIT_AFTER_INIT:
            if (elapsed >= init_to_place_low_wait_s_) {
                logistics_init_phase_ = LogisticsInitPhase::PLACE_LOW_CMD;
                logistics_init_phase_start_time_ = this->now();
            }
            break;

        case LogisticsInitPhase::PLACE_LOW_CMD:
            if (place_low_after_init_) {
                if (manipulation_enabled_) {
                    std_msgs::msg::UInt8 cmd;
                    cmd.data = static_cast<uint8_t>(place_low_cmd_);
                    manipulator_cmd_pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(),
                                "Manipulator place-low command after init published once: %d",
                                place_low_cmd_);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                "Manipulation disabled: skip place-low command after init");
                }
                logistics_init_phase_ = LogisticsInitPhase::WAIT_AFTER_PLACE_LOW;
                logistics_init_phase_start_time_ = this->now();
            } else {
                logistics_init_phase_ = LogisticsInitPhase::DONE;
            }
            break;

        case LogisticsInitPhase::WAIT_AFTER_PLACE_LOW:
            if (elapsed >= after_place_low_wait_s_) {
                logistics_init_phase_ = LogisticsInitPhase::DONE;
            }
            break;

        case LogisticsInitPhase::DONE:
            switch_state(State::LOGISTICS_PRE_SCAN);
            break;
    }
}

/**
 * @brief 任务赛低位左右看预扫描状态
 *
 * 输入：无
 * 输出：logistics_nav_enable=true；等待 lidar_nav_demo_node 发布 PRE_SCAN_DONE
 * 处理：机械臂保持低位，导航节点只执行 pre_path_spin 并在完成后暂停。
 */
void TaskStateMachine::handle_logistics_pre_scan_state() {
    nav_state = State::LOGISTICS_NAV;
    send_lower_command(LowerEvent::ENTER_AUTO);
}

/**
 * @brief 任务赛预扫描后机械臂抬起状态
 *
 * 输入：lift_standby_cmd_、lift_standby_wait_s_
 * 输出：发布一次机械臂抬起命令；等待完成后进入正式 LOGISTICS_NAV
 * 处理：此状态下 logistics_nav_enable=false，lidar_nav_demo_node 暂停等待恢复。
 */
void TaskStateMachine::handle_logistics_lift_arm_state() {
    if (!logistics_lift_cmd_sent_) {
        if (manipulation_enabled_) {
            std_msgs::msg::UInt8 cmd;
            cmd.data = static_cast<uint8_t>(lift_standby_cmd_);
            manipulator_cmd_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(),
                        "Manipulator lift-standby command after pre-scan published once: %d",
                        lift_standby_cmd_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Manipulation disabled: skip lift-standby command after pre-scan");
        }
        logistics_lift_cmd_sent_ = true;
        logistics_lift_start_time_ = this->now();
        return;
    }

    const double elapsed = (this->now() - logistics_lift_start_time_).seconds();
    if (elapsed >= lift_standby_wait_s_) {
        logistics_lift_cmd_sent_ = false;
        switch_state(State::LOGISTICS_NAV);
    }
}

/**
 * @brief 任务赛左右看扫描后等待遥控确认状态
 *
 * 输入：ALLOW_AUTO 任务命令
 * 输出：logistics_nav_enable=false，保持暂停
 * 处理：收到遥控确认后由 state_cmd_callback 切回 LOGISTICS_PRE_SCAN，让导航继续去观察点。
 */
void TaskStateMachine::handle_logistics_wait_remote_start_state() {
    nav_state = State::LOGISTICS_NAV;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Logistics pre-scan spin done. Waiting for LT+B to continue to record_pass point.");
}

/**
 * @brief 任务赛物流导航状态
 *
 * 输入：无
 * 输出：nav_state=LOGISTICS_NAV；发送 ENTER_AUTO
 * 处理：允许 lidar_nav_demo_cpp 接管 cmd_vel，等待 logistics_nav_event
 */
void TaskStateMachine::handle_logistics_nav_state() {
    nav_state = State::LOGISTICS_NAV;
    send_lower_command(LowerEvent::ENTER_AUTO);
}

/**
 * @brief 任务赛取物块状态
 */
void TaskStateMachine::handle_pickup_block_state() {
    execute_manipulation_task(ManipulationType::PICKUP);
}

/**
 * @brief 任务赛放物块状态
 */
void TaskStateMachine::handle_dropoff_block_state() {
    execute_manipulation_task(ManipulationType::DROPOFF);
}

/**
 * @brief 任务赛结束状态
 */
void TaskStateMachine::handle_logistics_done_state() {
    if (lower_state_ == static_cast<int>(LowerStateID::FIXED_STAND)) {
        switch_state(State::IDLE);
        return;
    }
    send_lower_command(LowerEvent::UP_DOWN);
}

/**
 * @brief TASK_OTHER 状态周期处理
 *
 * 输入：无
 * 输出：UP_DOWN
 * 处理：请求站立
 */
void TaskStateMachine::handle_other_state() {
    send_lower_command(LowerEvent::UP_DOWN);
}

/**
 * @brief 按 current_state_ 分发周期处理函数
 *
 * 输入：current_state_
 * 输出：调用对应 handle_* 及底层/执行器副作用
 * 处理：与原 switch 相同 case 顺序与 default
 */
void TaskStateMachine::dispatch_current_state() {
    switch (current_state_) {
        case State::IDLE:
            handle_idle_state();
            break;
        case State::AUTO_NAV:
            handle_auto_nav_state();
            break;
        case State::MANUAL_NAV:
            handle_manual_nav_state();
            break;
        case State::QR_RECOGNITION:
            handle_qr_recognition_state();
            break;
        case State::TASK_CRAWL_CROSS:
            handle_crawl_cross_state();
            break;
        case State::TASK_CRAWL_MOVING:
            handle_crawl_moving_state();
            break;
        case State::TASK_STAIR_UP:
            handle_stair_up_state();
            break;
        case State::TASK_STAIR_MOVING:
            handle_stair_moving_state();
            break;
        case State::TASK_STAIR_DOWN:
            handle_stair_down_state();
            break;
        case State::TASK_PATH_POLICY_PREP:
            handle_path_policy_prep_state();
            break;
        case State::TASK_PATH_POLICY_RESTORE:
            handle_path_policy_restore_state();
            break;
        case State::TASK_POLE_AROUND:
            handle_pole_around_state();
            break;
        case State::TASK_WALL_CROSS:
            handle_wall_cross_state();
            break;
        case State::TASK_WALL_POLICY_RESTORE:
            handle_wall_policy_restore_state();
            break;
        case State::TASK_SAND_INOUT:
            handle_sand_inout_state();
            break;
        case State::TASK_BRIDGE_CROSS:
            handle_bridge_cross_state();
            break;
        case State::LOGISTICS_INIT_MANIP:
            handle_logistics_init_manip_state();
            break;
        case State::LOGISTICS_PRE_SCAN:
            handle_logistics_pre_scan_state();
            break;
        case State::LOGISTICS_LIFT_ARM:
            handle_logistics_lift_arm_state();
            break;
        case State::LOGISTICS_WAIT_REMOTE_START:
            handle_logistics_wait_remote_start_state();
            break;
        case State::LOGISTICS_NAV:
            handle_logistics_nav_state();
            break;
        case State::TASK_PICKUP_BLOCK:
            handle_pickup_block_state();
            break;
        case State::TASK_DROPOFF_BLOCK:
            handle_dropoff_block_state();
            break;
        case State::LOGISTICS_DONE:
            handle_logistics_done_state();
            break;
        case State::TASK_OTHER:
            handle_other_state();
            break;
        default:
            break;
    }
}

/**
 * @brief 核心控制循环：任务状态机的决策中心
 *
 * 输入：current_state_、is_auto_mode_、底层反馈
 * 输出：状态话题、nav_enable、各状态 handle 副作用
 * 处理：发布状态→看门狗→自动模式下 dispatch_current_state
 */
void TaskStateMachine::control_loop() {
    publish_state_and_nav_enable();
    makesure_lowerstate();
    if (!is_auto_mode_) {
        return;
    }
    dispatch_current_state();
}

/**
 * @brief 策略切换时序机
 *
 * 输入：policy 事件、expected_policy、sub_step_、policy_durations_
 * 输出：发送底层策略命令，更新 sub_step_ 与 step_start_time_
 * 处理：反馈策略稳定 confirm_settle 后提前完成；action_time 作为兜底
 */
bool TaskStateMachine::change_policy(LowerEvent policy, int expected_policy) {
    const auto now = this->now();
    double elapsed = (now - step_start_time_).seconds();

    switch (sub_step_) {
        case 0:
            send_lower_command(policy);
            sub_step_ = 1;
            step_start_time_ = now;
            policy_confirm_start_time_ = now;
            policy_confirm_seen_ = false;
            break;
        case 1:
            if (expected_policy >= 0 && lower_policy_ == expected_policy) {
                if (!policy_confirm_seen_) {
                    policy_confirm_start_time_ = now;
                    policy_confirm_seen_ = true;
                }
                if ((now - policy_confirm_start_time_).seconds() >= policy_confirm_settle_s_) {
                    sub_step_ = 0;
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                    policy_confirm_seen_ = false;
                    return true;
                }
            } else {
                policy_confirm_seen_ = false;
            }
            if (elapsed > policy_durations_.action_time) {
                sub_step_ = 0;
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                policy_confirm_seen_ = false;
                return expected_policy < 0 || lower_policy_ == expected_policy;
            }
            break;
    }
    return false;
}

bool TaskStateMachine::run_kneel_policy_transition(LowerEvent policy,
                                                   int expected_policy,
                                                   const char * context) {
    switch (kneel_policy_phase_) {
        case KneelPolicyPhase::TO_FIXED_STAND:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_STAND)) {
                if (last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::UP_DOWN);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Kneel policy transition (%s): reached FIXED_STAND.", context);
            kneel_policy_phase_ = KneelPolicyPhase::TO_FIXED_DOWN;
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            return false;

        case KneelPolicyPhase::TO_FIXED_DOWN:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_DOWN)) {
                if (last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::UP_DOWN);
                return false;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Kneel policy transition (%s): reached FIXED_DOWN, changing policy.",
                        context);
            kneel_policy_phase_ = KneelPolicyPhase::CHANGE_POLICY_IN_FIXED_DOWN;
            sub_step_ = 0;
            step_start_time_ = this->now();
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            return false;

        case KneelPolicyPhase::CHANGE_POLICY_IN_FIXED_DOWN:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_DOWN)) {
                RCLCPP_WARN(this->get_logger(),
                            "Kneel policy transition (%s): left FIXED_DOWN before policy switch; returning.",
                            context);
                kneel_policy_phase_ = KneelPolicyPhase::TO_FIXED_DOWN;
                sub_step_ = 0;
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                return false;
            }
            if (change_policy(policy, expected_policy)) {
                RCLCPP_INFO(this->get_logger(),
                            "Kneel policy transition (%s): policy=%d confirmed in FIXED_DOWN.",
                            context, expected_policy);
                kneel_policy_phase_ = KneelPolicyPhase::BACK_TO_FIXED_STAND;
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            }
            return false;

        case KneelPolicyPhase::BACK_TO_FIXED_STAND:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_STAND)) {
                if (last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::UP_DOWN);
                return false;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Kneel policy transition (%s): returned to FIXED_STAND.",
                        context);
            kneel_policy_phase_ = KneelPolicyPhase::ENTER_RL;
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            return false;

        case KneelPolicyPhase::ENTER_RL:
            if (lower_state_ != static_cast<int>(LowerStateID::RL_MOVE)) {
                if (last_sent_lower_code_ == LowerEvent::ENTER_RL && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::ENTER_RL);
                return false;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Kneel policy transition (%s): returned to RL_MOVE.",
                        context);
            kneel_policy_phase_ = KneelPolicyPhase::DONE;
            return true;

        case KneelPolicyPhase::DONE:
            return true;
    }
    return false;
}

bool TaskStateMachine::policy_from_name(const std::string& name,
                                        LowerEvent& event,
                                        int& expected_policy) const {
    if (name == "trot") {
        event = LowerEvent::POLICY_TROT;
        expected_policy = 0;
        return true;
    }
    if (name == "upstair") {
        event = LowerEvent::POLICY_UPSTAIR;
        expected_policy = 2;
        return true;
    }
    if (name == "kneel_crawl") {
        event = LowerEvent::POLICY_KNEEL_CRAWL;
        expected_policy = 3;
        return true;
    }
    return false;
}

/**
 * @brief 复杂动作位控序列（高墙/沙坑等）
 *
 * 输入：action（JUMP/STRIDE）、task_durations_、sub_step_
 * 输出：分步 send_lower_command，完成后 switch_state(nav_state)
 * 处理：进入 FIXED_DOWN 并等待 prep→action→buffer→recover→回导航
 */
void TaskStateMachine::execute_complex_task(LowerEvent action) {
    double elapsed = (this->now() - step_start_time_).seconds();

    switch (sub_step_) {
        case 0:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_DOWN)) {
                if (last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                    last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                }
                send_lower_command(LowerEvent::UP_DOWN);
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Complex action prep: reached FIXED_DOWN, waiting %.2fs before action.",
                        task_durations_.prep_time);
            sub_step_++;
            step_start_time_ = this->now();
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            break;
        case 1: 
            if (elapsed > task_durations_.prep_time) {
                sub_step_++;
                step_start_time_ = this->now();
            }
            break;
        case 2:
            send_lower_command(action);
            if (elapsed > task_durations_.action_time) { 
                sub_step_++; 
                step_start_time_ = this->now(); 
            }
            break;
        case 3:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_DOWN) &&
                last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            }
            send_lower_command(LowerEvent::UP_DOWN);
            if (lower_state_ == static_cast<int>(LowerStateID::FIXED_DOWN) &&
                elapsed > task_durations_.buffer_time) {
                sub_step_++; 
                step_start_time_ = this->now(); 
                last_sent_lower_code_ = static_cast<LowerEvent>(-2);
            }
            break;
        case 4:
            if (lower_state_ != static_cast<int>(LowerStateID::FIXED_STAND) &&
                last_sent_lower_code_ == LowerEvent::UP_DOWN && expected_lower_state_ == -1) {
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            }
            send_lower_command(LowerEvent::UP_DOWN);
            if (lower_state_ == static_cast<int>(LowerStateID::FIXED_STAND) &&
                elapsed > task_durations_.recover_time) {
                sub_step_++; 
                step_start_time_ = this->now(); 
            }
            break;
        case 5:
            if (action == LowerEvent::JUMP) {
                switch_state(State::TASK_WALL_POLICY_RESTORE);
            } else {
                finish_obstacle_sequence_task();
            }
            sub_step_ = 0; 
            break;
    }
}

void TaskStateMachine::enter_manip_phase(ManipPhase phase) {
    manip_phase_ = phase;
    manip_phase_start_time_ = this->now();
    if (phase == ManipPhase::TRIGGER_ARM) {
        manip_command_sent_ = false;
        manip_result_received_ = false;
    }
    if (phase == ManipPhase::TO_STAND || phase == ManipPhase::BACK_TO_RL) {
        last_sent_lower_code_ = static_cast<LowerEvent>(-3);
        expected_lower_state_ = -1;
    }
}

void TaskStateMachine::publish_zero_cmd_vel() {
    if (!cmd_vel_pub_) {
        return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}

/**
 * @brief 任务赛取/放物块动作序列
 *
 * 取货：暂停导航 -> 静止等待 -> 等机体速度归零 -> 发机械臂命令 -> 等 /manipulator/result -> 恢复导航（不站立）。
 * 低层放货：与取货相同，走路模式下 PLACE_LOW，不切站立。
 * 高层放货：暂停导航 -> 静止等待 -> 切站立 -> 等机体速度归零 -> PLACE_HIGH -> 等结果 -> 回 RL -> 恢复导航。
 */
void TaskStateMachine::execute_manipulation_task(ManipulationType type) {
    publish_zero_cmd_vel();

    const double elapsed = (this->now() - manip_phase_start_time_).seconds();
    const bool is_pickup = (type == ManipulationType::PICKUP);
    if (is_pickup) {
        publish_logistics_pickup_reached(true);
    }

    switch (manip_phase_) {
        case ManipPhase::STOP_NAV:
            enter_manip_phase(ManipPhase::PRE_STAND_WAIT);
            break;

        case ManipPhase::PRE_STAND_WAIT:
            if (elapsed >= pre_stand_wait_s_) {
                if (is_pickup || !active_dropoff_high_layer_) {
                    enter_manip_phase(ManipPhase::TRIGGER_ARM);
                } else {
                    enter_manip_phase(ManipPhase::TO_STAND);
                }
            }
            break;

        case ManipPhase::TO_STAND:
            if (lower_state_ == static_cast<int>(LowerStateID::FIXED_STAND)) {
                enter_manip_phase(ManipPhase::STAND_SETTLE);
                break;
            }
            send_lower_command(LowerEvent::UP_DOWN);
            if (elapsed > lower_state_timeout_s_) {
                RCLCPP_WARN(this->get_logger(),
                            "Dropoff: waiting for FIXED_STAND timeout, resend UP_DOWN.");
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                manip_phase_start_time_ = this->now();
            }
            break;

        case ManipPhase::STAND_SETTLE:
            if (elapsed >= stand_settle_s_) {
                enter_manip_phase(ManipPhase::TRIGGER_ARM);
            }
            break;

        case ManipPhase::TRIGGER_ARM:
            enter_manip_phase(ManipPhase::ARM_CMD_SETTLE);
            break;

        case ManipPhase::ARM_CMD_SETTLE:
            if (manip_result_received_) {
                proceed_after_manip_result(is_pickup);
                break;
            }
            if (!manip_command_sent_) {
                if (body_motion_stopped() || elapsed >= arm_cmd_settle_s_) {
                    const int command_value = active_manip_cmd_ > 0
                        ? active_manip_cmd_
                        : (is_pickup ? pickup_cmd_ : place_low_cmd_);
                    if (manipulation_enabled_) {
                        std_msgs::msg::UInt8 cmd;
                        cmd.data = static_cast<uint8_t>(command_value);
                        manipulator_cmd_pub_->publish(cmd);
                        RCLCPP_INFO(this->get_logger(),
                                    "Manipulation command published once: %s (%d), body_motion_stopped=%s",
                                    is_pickup ? "PICKUP" : "DROPOFF", command_value,
                                    body_motion_stopped() ? "true" : "false");
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                                    "Manipulation disabled: skip %s command (%d), body_motion_stopped=%s",
                                    is_pickup ? "PICKUP" : "DROPOFF", command_value,
                                    body_motion_stopped() ? "true" : "false");
                    }
                    manip_command_sent_ = true;
                    enter_manip_phase(ManipPhase::WAIT_ARM);
                }
            }
            break;

        case ManipPhase::WAIT_ARM: {
            const double wait_s = manipulation_enabled_
                ? (is_pickup ? pickup_wait_s_ : dropoff_wait_s_)
                : disabled_result_wait_s_;

            // 仅取货时处理吸附失败重试
            if (is_pickup && suction_failure_received_) {
                if (suction_failure_count_ < suction_max_retries_) {
                    suction_failure_count_++;
                    if (suction_retry_stand_enable_) {
                        suction_retry_active_ = true;
                        RCLCPP_WARN(this->get_logger(),
                                    "Suction failed, retrying grab %d/%d (switching to FIXED_STAND)",
                                    suction_failure_count_, suction_max_retries_);
                        suction_failure_received_ = false;
                        suction_detected_ = false;
                        expected_lower_state_ = -1;
                        last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                        enter_manip_phase(ManipPhase::TO_STAND);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "Suction failed, retrying grab %d/%d (in-place)",
                                    suction_failure_count_, suction_max_retries_);
                        suction_failure_received_ = false;
                        suction_detected_ = false;
                        enter_manip_phase(ManipPhase::TRIGGER_ARM);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Suction failed after %d retries, skipping block",
                                suction_max_retries_);
                    suction_failure_count_ = 0;
                    if (suction_retry_active_) {
                        // 站立重试模式：需要恢复回 RL_MOVE
                        expected_lower_state_ = -1;
                        last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                        enter_manip_phase(ManipPhase::BACK_TO_RL);
                    } else {
                        // 原地重试模式：直接跳过
                        enter_manip_phase(ManipPhase::DONE);
                    }
                }
                break;
            }

            if (manip_result_received_) {
                suction_failure_count_ = 0;
                proceed_after_manip_result(is_pickup);
            } else if (elapsed >= wait_s) {
                RCLCPP_WARN(this->get_logger(),
                            "Manipulation: /manipulator/result timeout after %.1fs.",
                            wait_s);
                if (is_pickup || !active_dropoff_high_layer_) {
                    enter_manip_phase(ManipPhase::DONE);
                } else {
                    enter_manip_phase(ManipPhase::BACK_TO_RL);
                }
            }
            break;
        }

        case ManipPhase::BACK_TO_RL:
            if (lower_state_ == static_cast<int>(LowerStateID::RL_MOVE)) {
                enter_manip_phase(ManipPhase::RL_SETTLE);
                break;
            }
            send_lower_command(LowerEvent::ENTER_RL);
            if (elapsed > lower_state_timeout_s_) {
                RCLCPP_WARN(this->get_logger(),
                            "Dropoff: waiting for RL_MOVE timeout, resend ENTER_RL.");
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                manip_phase_start_time_ = this->now();
            }
            break;

        case ManipPhase::RL_SETTLE:
            if (elapsed >= rl_settle_s_) {
                RCLCPP_INFO(this->get_logger(), "Dropoff finished. Resume LOGISTICS_NAV.");
                enter_manip_phase(ManipPhase::DONE);
            }
            break;

        case ManipPhase::DONE:
            if (is_pickup) {
                pickup_dock_command_sent_ = false;
                pickup_dock_waypoint_index_ = -1;
                publish_logistics_pickup_reached(false);
            }
            suction_retry_active_ = false;
            enter_manip_phase(ManipPhase::STOP_NAV);
            switch_state(State::LOGISTICS_NAV);
            break;

        default:
            break;
    }
}

void TaskStateMachine::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_cmd_vel_ = *msg;
    cmd_vel_received_ = true;
}

bool TaskStateMachine::body_motion_stopped() const {
    if (!cmd_vel_received_) {
        return false;
    }
    return std::abs(latest_cmd_vel_.linear.x) <= kBodyStopLinVelEps &&
           std::abs(latest_cmd_vel_.linear.y) <= kBodyStopLinVelEps &&
           std::abs(latest_cmd_vel_.angular.z) <= kBodyStopAngVelEps;
}

void TaskStateMachine::proceed_after_manip_result(bool is_pickup) {
    if (is_pickup) {
        if (suction_retry_active_) {
            // 重试成功后，先恢复回 RL_MOVE
            RCLCPP_INFO(this->get_logger(), "Pickup finished after retry. Restoring RL_MOVE.");
            suction_retry_active_ = false;
            expected_lower_state_ = -1;
            last_sent_lower_code_ = static_cast<LowerEvent>(-3);
            enter_manip_phase(ManipPhase::BACK_TO_RL);
        } else {
            RCLCPP_INFO(this->get_logger(), "Pickup finished. Resume LOGISTICS_NAV.");
            enter_manip_phase(ManipPhase::DONE);
        }
    } else if (active_dropoff_high_layer_) {
        enter_manip_phase(ManipPhase::BACK_TO_RL);
    } else {
        RCLCPP_INFO(this->get_logger(), "Low-layer dropoff finished. Resume LOGISTICS_NAV.");
        enter_manip_phase(ManipPhase::DONE);
    }
}

void TaskStateMachine::manipulator_result_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
        return;
    }
    if (manip_phase_ != ManipPhase::ARM_CMD_SETTLE &&
        manip_phase_ != ManipPhase::WAIT_ARM) {
        return;
    }
    manip_result_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Manipulation result received: success=true");
}

void TaskStateMachine::suction_detect_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    suction_detected_ = msg->data;
    if (!msg->data && (manip_phase_ == ManipPhase::ARM_CMD_SETTLE ||
                       manip_phase_ == ManipPhase::WAIT_ARM)) {
        suction_failure_received_ = true;
    }
}

void TaskStateMachine::switch_state_from_task_id(int task_id) {
    const auto* slot = obstacle_slot_for_task_id(task_id);
    if (!slot) {
        RCLCPP_ERROR(this->get_logger(),
                     "No obstacle_sequence slot matches QR task_id=%d. Stop.",
                     task_id);
        switch_state(State::IDLE);
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "QR aligned for task_id=%d -> slot '%s'.",
                task_id, slot->name.c_str());
    switch_state_from_obstacle_action(*slot);
}

void TaskStateMachine::switch_state_from_obstacle_action(const ObstacleSlot& slot) {
    RCLCPP_INFO(this->get_logger(), "QR aligned for slot '%s', action='%s', task_id=%d.",
                slot.name.c_str(), slot.action.c_str(), slot.expected_task_id);
    active_obstacle_segment_ = 0;
    active_task_slot_name_ = slot.name;
    active_path_policy_ = slot.path_policy;
    active_path_restore_policy_ = slot.restore_policy;
    if (slot.action == "path") {
        if (slot.path_policy != "trot") {
            RCLCPP_INFO(this->get_logger(),
                        "Path slot '%s' requires policy '%s' before path replay.",
                        slot.name.c_str(), slot.path_policy.c_str());
            switch_state(State::TASK_PATH_POLICY_PREP);
        } else {
            switch_state(State::TASK_POLE_AROUND);
        }
    } else if (slot.action == "stride") {
        switch_state(State::TASK_SAND_INOUT);
    } else if (slot.action == "crawl") {
        switch_state(State::TASK_CRAWL_CROSS);
    } else if (slot.action == "line") {
        switch_state(State::TASK_BRIDGE_CROSS);
    } else if (slot.action == "stair_combo") {
        switch_state(State::TASK_STAIR_UP);
    } else if (slot.action == "wall") {
        switch_state(State::TASK_WALL_CROSS);
    } else {
        RCLCPP_WARN(this->get_logger(), "Slot '%s' has no executable action '%s'. Skipping.",
                    slot.name.c_str(), slot.action.c_str());
        finish_obstacle_sequence_task();
    }
}

/**
 * @brief 执行器完成匍匐/上楼运动段后的交接
 *
 * 输入：current_state_ 为 CRAWL_MOVING 或 STAIR_MOVING
 * 输出：回到 CRAWL_CROSS/STAIR_UP，crawl_phase_=2，重置 sub_step_
 * 处理：准备状态机切回 TROT 策略
 */
void TaskStateMachine::on_executor_moving_section_done() {
    RCLCPP_INFO(this->get_logger(), "Moving section finished. Changing policy back to TROT.");
    if (current_state_ == State::TASK_CRAWL_MOVING) {
        switch_state(State::TASK_CRAWL_CROSS);
    } else {
        switch_state(State::TASK_STAIR_UP);
    }
    crawl_phase_ = 2;
    sub_step_ = 0;
    step_start_time_ = this->now();
}

/**
 * @brief 常规障碍任务完成（如过桥、绕杆）
 *
 * 输入：nav_state
 * 输出：current_state_ 恢复为 nav_state
 * 处理：switch_state(nav_state)
 */
void TaskStateMachine::on_generic_task_done() {
    RCLCPP_INFO(this->get_logger(), "Task completed. Restoring nav_state.");
    finish_obstacle_sequence_task();
}

/**
 * @brief 任务完成信号回调
 *
 * 输入：msg->data 为 task_id（QR 识别为正 ID，段完成/任务完成为 -1）
 * 输出：按当前状态分发到各 on_* / switch_state_from_task_id
 * 处理：QR+正 ID→障碍任务；MOVING+-1→策略恢复；其它 -1→回 nav
 */
void TaskStateMachine::task_done_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    const int task_id = msg->data;

    if (current_state_ == State::QR_RECOGNITION && task_id > 0) {
        if (obstacle_sequence_enabled_) {
            const auto* slot = active_obstacle_slot();
            if (!slot) {
                RCLCPP_ERROR(this->get_logger(), "QR result received but no active obstacle slot.");
                switch_state(State::IDLE);
                return;
            }
            if (slot->expected_task_id != task_id) {
                RCLCPP_ERROR(this->get_logger(),
                             "QR task mismatch at slot '%s': expected %d, got %d. Stop.",
                             slot->name.c_str(), slot->expected_task_id, task_id);
                switch_state(State::IDLE);
                return;
            }
            switch_state_from_obstacle_action(*slot);
            return;
        }
        switch_state_from_task_id(task_id);
    } else if ((current_state_ == State::TASK_CRAWL_MOVING || current_state_ == State::TASK_STAIR_MOVING) &&
               task_id == -1) {
        if (obstacle_sequence_enabled_ && current_state_ == State::TASK_STAIR_MOVING) {
            const auto* slot = active_obstacle_slot();
            if (slot && slot->action == "stair_combo" && active_obstacle_segment_ == 0) {
                active_obstacle_segment_ = 1;
                sub_step_ = 0;
                step_start_time_ = this->now();
                last_sent_lower_code_ = static_cast<LowerEvent>(-3);
                switch_state(State::TASK_STAIR_DOWN);
                return;
            }
        }
        on_executor_moving_section_done();
    } else if (task_id == -1) {
        if (current_state_ == State::TASK_POLE_AROUND && active_path_restore_policy_ == "trot") {
            RCLCPP_INFO(this->get_logger(),
                        "Path slot '%s' finished. Restoring POLICY_TROT before continuing.",
                        active_task_slot_name_.c_str());
            switch_state(State::TASK_PATH_POLICY_RESTORE);
            return;
        }
        on_generic_task_done();
    }
}

/**
 * @brief 任务赛导航事件回调
 *
 * msg.data = [event_type, waypoint_index, seq, action_id]
 * event_type: 0=PRE_SCAN_DONE, 1=PICKUP_REACHED, 2=DROPOFF_REACHED,
 *             3=LOGISTICS_FINISHED, 4=PICKUP_DOCK_ENTERED, 5=PRE_SCAN_SPIN_DONE
 * action_id: DROPOFF 时为 dropoff_points 数组下标（0~3 低层，>=dropoff_high_layer_min_index 高层）
 */
void TaskStateMachine::logistics_nav_event_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!is_logistics_mode_) {
        return;
    }
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Invalid logistics_nav_event, expected [event, waypoint, seq].");
        return;
    }

    const int event_type = msg->data[0];
    const int waypoint_index = msg->data[1];
    const int seq = msg->data[2];
    const int action_id = (msg->data.size() >= 4) ? msg->data[3] : -1;
    if (seq == last_logistics_event_seq_) {
        return;
    }
    last_logistics_event_seq_ = seq;

    if (event_type == kLogisticsPreScanSpinDone) {
        if (current_state_ == State::LOGISTICS_PRE_SCAN) {
            RCLCPP_INFO(this->get_logger(), "Logistics pre-scan spin finished. Waiting for LT+B.");
            switch_state(State::LOGISTICS_WAIT_REMOTE_START);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Ignore PRE_SCAN_SPIN_DONE while current state is %s.",
                        state_to_string(current_state_).c_str());
        }
        return;
    }

    if (event_type == kLogisticsPreScanDone) {
        if (current_state_ == State::LOGISTICS_PRE_SCAN) {
            RCLCPP_INFO(this->get_logger(), "Logistics pre-scan finished. Lift manipulator before navigation.");
            logistics_lift_cmd_sent_ = false;
            logistics_lift_start_time_ = this->now();
            switch_state(State::LOGISTICS_LIFT_ARM);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Ignore PRE_SCAN_DONE while current state is %s.",
                        state_to_string(current_state_).c_str());
        }
        return;
    }

    if (current_state_ != State::LOGISTICS_NAV) {
        RCLCPP_WARN(this->get_logger(),
                    "Ignore logistics event %d at waypoint %d while current state is %s.",
                    event_type, waypoint_index, state_to_string(current_state_).c_str());
        return;
    }

    if (event_type == kLogisticsPickupDockEntered) {
        RCLCPP_INFO(this->get_logger(),
                    "Logistics waypoint %d entered PICKUP dock. Publish pickup command and keep navigation enabled.",
                    waypoint_index);
        active_dropoff_high_layer_ = false;
        active_manip_cmd_ = pickup_cmd_;
        pickup_dock_command_sent_ = true;
        pickup_dock_waypoint_index_ = waypoint_index;
        suction_failure_count_ = 0;
        suction_failure_received_ = false;
        suction_detected_ = false;
        suction_retry_active_ = false;
        manip_result_received_ = false;
        enter_manip_phase(ManipPhase::WAIT_ARM);
        publish_manipulator_cmd(static_cast<uint8_t>(pickup_cmd_), "PICKUP_DOCK_ENTERED");
    } else if (event_type == kLogisticsPickupReached) {
        RCLCPP_INFO(this->get_logger(), "Logistics waypoint %d reached: PICKUP.", waypoint_index);
        const bool pickup_command_already_sent =
            pickup_dock_command_sent_ && pickup_dock_waypoint_index_ == waypoint_index;
        active_dropoff_high_layer_ = false;
        active_manip_cmd_ = pickup_cmd_;
        if (!pickup_command_already_sent) {
            suction_failure_count_ = 0;
            suction_failure_received_ = false;
            suction_detected_ = false;
            suction_retry_active_ = false;
            enter_manip_phase(ManipPhase::STOP_NAV);
        } else {
            enter_manip_phase(ManipPhase::WAIT_ARM);
        }
        publish_logistics_pickup_reached(true);
        switch_state(State::TASK_PICKUP_BLOCK);
    } else if (event_type == kLogisticsDropoffReached) {
        const int dropoff_idx = action_id >= 0 ? action_id : waypoint_index;
        const bool use_high_layer = dropoff_idx >= dropoff_high_layer_min_index_;
        active_dropoff_high_layer_ = use_high_layer;
        active_manip_cmd_ = use_high_layer ? place_high_cmd_ : place_low_cmd_;
        RCLCPP_INFO(this->get_logger(),
                    "Logistics waypoint %d reached: DROPOFF dropoff_idx=%d -> %s (%s) command %d.",
                    waypoint_index, dropoff_idx,
                    use_high_layer ? "PLACE_HIGH" : "PLACE_LOW",
                    use_high_layer ? "stand mode" : "walk mode", active_manip_cmd_);
        enter_manip_phase(ManipPhase::STOP_NAV);
        switch_state(State::TASK_DROPOFF_BLOCK);
    } else if (event_type == kLogisticsFinished) {
        RCLCPP_INFO(this->get_logger(), "Logistics navigation finished.");
        switch_state(State::LOGISTICS_DONE);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown logistics event type: %d", event_type);
    }
}

/**
 * @brief 调试话题回调
 */
void TaskStateMachine::debug_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "BRIDGE") {
        switch_state(State::TASK_BRIDGE_CROSS);
    } else if (msg->data == "IDLE") {
        switch_state(State::IDLE);
    }
}

void TaskStateMachine::publish_manipulator_cmd(uint8_t command, const char * context) {
    if (!manipulation_enabled_) {
        RCLCPP_INFO(this->get_logger(),
                    "Manipulation disabled: skip %s command (%u)",
                    context, static_cast<unsigned>(command));
        return;
    }

    std_msgs::msg::UInt8 cmd;
    cmd.data = command;
    manipulator_cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
                "Manipulator command published once: %s (%u)",
                context, static_cast<unsigned>(command));
}

void TaskStateMachine::publish_logistics_pickup_reached(bool reached) {
    if (!logistics_pickup_reached_pub_) {
        return;
    }
    std_msgs::msg::Bool msg;
    msg.data = reached;
    logistics_pickup_reached_pub_->publish(msg);
    if (reached) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Logistics pickup reached flag holding: true");
    } else {
        RCLCPP_INFO(this->get_logger(), "Logistics pickup reached flag published: false");
    }
}

/**
 * @brief 任务赛开始时发布一次机械臂初始化命令
 */
void TaskStateMachine::publish_manipulator_init_cmd() {
    if (!manip_init_on_logistics_start_) {
        return;
    }
    if (!manipulation_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Manipulation disabled: skip init command");
        return;
    }

    std_msgs::msg::UInt8 cmd;
    cmd.data = static_cast<uint8_t>(manip_init_cmd_);
    manipulator_cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
                "Manipulator init command published once: %d",
                manip_init_cmd_);
}

/**
 * @brief 状态指令回调
 */
void TaskStateMachine::state_cmd_callback(const quad::msg::StateCommand::SharedPtr msg) {
    using Cmd = quad::msg::StateCommand;
    switch (msg->command_id) {
        case Cmd::CMD_IDLE:
            switch_state(State::IDLE);
            break;
        case Cmd::CMD_AUTO_NAV:
            if (is_logistics_mode_) {
                active_manip_cmd_ = 0;
                logistics_init_phase_ = LogisticsInitPhase::INIT_CMD;
                logistics_init_phase_start_time_ = this->now();
                logistics_lift_cmd_sent_ = false;
                switch_state(State::LOGISTICS_INIT_MANIP);
                break;
            }
            switch_state(State::AUTO_NAV);
            break;
        case Cmd::CMD_MANUAL_NAV:
            switch_state(State::MANUAL_NAV);
            break;
        case Cmd::CMD_QR_RECOGNITION:
            switch_state(State::QR_RECOGNITION);
            break;
        case Cmd::CMD_ALLOW_LOGISTICS_NAV:
            if (current_state_ == State::LOGISTICS_WAIT_REMOTE_START) {
                RCLCPP_INFO(this->get_logger(),
                            "Remote start confirmed. Continue logistics pre-scan flow.");
                switch_state(State::LOGISTICS_PRE_SCAN);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Ignore ALLOW_LOGISTICS_NAV while current state is %s.",
                            state_to_string(current_state_).c_str());
            }
            break;
        default:
            break;
    }
}

/**
 * @brief 导航状态回调
 */
void TaskStateMachine::nav_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (current_state_ == State::AUTO_NAV && msg->data) {
        if (obstacle_sequence_enabled_) {
            const auto* slot = active_obstacle_slot();
            if (!slot) {
                switch_state(State::IDLE);
                return;
            }
            if (!slot->is_obstacle) {
                RCLCPP_INFO(this->get_logger(), "Navigation-only slot '%s' reached; advance immediately.",
                            slot->name.c_str());
                advance_obstacle_slot();
                if (obstacle_sequence_finished()) {
                    switch_state(State::IDLE);
                }
                return;
            }
            if (!slot->require_visual_servo) {
                RCLCPP_INFO(this->get_logger(),
                            "Radar-classified obstacle slot '%s' reached; execute action '%s' without QR.",
                            slot->name.c_str(), slot->action.c_str());
                switch_state_from_obstacle_action(*slot);
                return;
            }
        }
        switch_state(State::QR_RECOGNITION);
    }
}

/**
 * @brief 障碍点序列全部导航完成回调
 */
void TaskStateMachine::nav_finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (current_state_ == State::AUTO_NAV && msg->data) {
        RCLCPP_INFO(this->get_logger(), "Obstacle navigation finished. Switching to IDLE.");
        switch_state(State::IDLE);
    }
}

/**
 * @brief 发送底层指令并预测下一底层状态
 *
 * 输入：event、lower_state_、tx_map
 * 输出：cmd_evt 话题、expected_lower_state_、last_sent_lower_code_
 * 处理：去重后查表预测并 publish Int8 事件码
 */
void TaskStateMachine::send_lower_command(LowerEvent event) {
    static std::map<int, std::map<int, int>> tx_map = {
        {0, { {0, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0}}},
        {1, { {0, 1}, {1, 0}, {2, 2}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {9, 1}, {10, 1}, {11, 1}, {12, 1}}},
        {2, { {0, 2}, {1, 2}, {2, 3}, {3, 1}, {4, 2}, {5, 7}, {6, 8}, {7, 6}, {9, 2}, {10, 2}, {11, 2}, {12, 2}}},
        {3, { {0, 3}, {1, 3}, {2, 2}, {3, 1}, {4, 5}, {5, 3}, {6, 3}, {7, 3}, {9, 3}, {10, 3}, {11, 3}, {12, 3}}},
        {5, { {0, 5}, {1, 5}, {2, 3}, {3, 1}, {4, 5}, {5, 5}, {6, 5}, {7, 5}, {9, 5}, {10, 5}, {11, 5}, {12, 5}}},
        {6, { {0, 6}, {1, 6}, {2, 2}, {3, 1}, {4, 6}, {5, 6}, {6, 6}, {7, 6}, {9, 6}, {10, 6}, {11, 6}, {12, 6}}},
        {7, { {0, 7}, {1, 7}, {2, 2}, {3, 1}, {4, 7}, {5, 7}, {6, 7}, {7, 7}, {9, 7}, {10, 7}, {11, 7}, {12, 7}}},
        {8, { {0, 8}, {1, 8}, {2, 2}, {3, 1}, {4, 8}, {5, 8}, {6, 8}, {7, 8}, {9, 8}, {10, 8}, {11, 8}, {12, 8}}},
    };

    int event_code = static_cast<int>(event);
    if (last_sent_lower_code_ == event) return;

    if (tx_map.count(lower_state_) && tx_map[lower_state_].count(event_code)) {
        expected_lower_state_ = tx_map[lower_state_][event_code];
    } else {
        expected_lower_state_ = -1; 
    }

    std_msgs::msg::Int8 cmd;
    cmd.data = event_code;
    lower_cmd_pub_->publish(cmd);
    
    last_sent_lower_code_ = event;
    last_cmd_time_ = this->now();
}

/**
 * @brief 确保底层状态到位（软看门狗）
 *
 * 输入：expected_lower_state_、lower_state_、last_cmd_time_
 * 输出：超时则清除 expected 并打 ERROR
 * 处理：到位则清 expected；1.5s 未到位报超时
 */
void TaskStateMachine::makesure_lowerstate() {
    if (expected_lower_state_ == -1 || lower_state_ == expected_lower_state_) {
        if (lower_state_ == expected_lower_state_) expected_lower_state_ = -1; 
        return;
    }

    double elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > 1.5) { 
        RCLCPP_ERROR(this->get_logger(), 
            "LOWER FSM TIMEOUT! Expected: %d, Actual: %d.", 
            expected_lower_state_, lower_state_);
        expected_lower_state_ = -1; 
    }
}

/**
 * @brief 状态切换保护函数
 *
 * 输入：new_state、current_state_
 * 输出：current_state_、sub_step_=0，非匍匐/楼梯相关时 crawl_phase_=0
 * 处理：相同状态不切换；打日志后更新
 */
void TaskStateMachine::switch_state(State new_state) {
    if (current_state_ == new_state) return; 
    
    RCLCPP_INFO(this->get_logger(), "State Switch: %s -> %s", 
                state_to_string(current_state_).c_str(), state_to_string(new_state).c_str());

    current_state_ = new_state;
    sub_step_ = 0;      
    policy_confirm_seen_ = false;
    if (new_state == State::TASK_PATH_POLICY_PREP ||
        new_state == State::TASK_PATH_POLICY_RESTORE) {
        kneel_policy_phase_ = KneelPolicyPhase::TO_FIXED_STAND;
    }
    if (new_state != State::TASK_CRAWL_MOVING && new_state != State::TASK_CRAWL_CROSS &&
        new_state != State::TASK_STAIR_MOVING && new_state != State::TASK_STAIR_UP) {
        crawl_phase_ = 0;   
    }
}

/**
 * @brief 状态枚举转字符串
 */
std::string TaskStateMachine::state_to_string(State state) {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::AUTO_NAV: return "AUTO_NAV";
        case State::MANUAL_NAV: return "MANUAL_NAV";
        case State::QR_RECOGNITION: return "QR_RECOGNITION";
        case State::TASK_WALL_CROSS: return "TASK_WALL_CROSS";
        case State::TASK_WALL_POLICY_RESTORE: return "TASK_WALL_POLICY_RESTORE";
        case State::TASK_SAND_INOUT: return "TASK_SAND_INOUT";
        case State::TASK_BRIDGE_CROSS: return "TASK_BRIDGE_CROSS";
        case State::TASK_CRAWL_CROSS: return "TASK_CRAWL_CROSS";
        case State::TASK_CRAWL_MOVING: return "TASK_CRAWL_MOVING";
        case State::TASK_STAIR_UP: return "TASK_STAIR_UP";
        case State::TASK_STAIR_MOVING: return "TASK_STAIR_MOVING";
        case State::TASK_STAIR_DOWN: return "TASK_STAIR_DOWN";
        case State::TASK_PATH_POLICY_PREP: return "TASK_PATH_POLICY_PREP";
        case State::TASK_PATH_POLICY_RESTORE: return "TASK_PATH_POLICY_RESTORE";
        case State::TASK_POLE_AROUND: return "TASK_POLE_AROUND";
        case State::TASK_SLOPE_CROSS: return "TASK_SLOPE_CROSS";
        case State::LOGISTICS_INIT_MANIP: return "LOGISTICS_INIT_MANIP";
        case State::LOGISTICS_PRE_SCAN: return "LOGISTICS_PRE_SCAN";
        case State::LOGISTICS_LIFT_ARM: return "LOGISTICS_LIFT_ARM";
        case State::LOGISTICS_WAIT_REMOTE_START: return "LOGISTICS_WAIT_REMOTE_START";
        case State::LOGISTICS_NAV: return "LOGISTICS_NAV";
        case State::TASK_PICKUP_BLOCK: return "TASK_PICKUP_BLOCK";
        case State::TASK_DROPOFF_BLOCK: return "TASK_DROPOFF_BLOCK";
        case State::LOGISTICS_DONE: return "LOGISTICS_DONE";
        case State::TASK_OTHER: return "TASK_OTHER";
        default: return "UNKNOWN";
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskStateMachine>()); 
    rclcpp::shutdown();
    return 0;
}
