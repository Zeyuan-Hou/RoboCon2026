#include "pos_control_node.h"
#include <cmath>

static const std::vector<double> POS_STAND = {
    0.0,  -1, 1.4,  
    0.0,  -1, 1.4,  
    0.0,  -0.8, 1.4,  
    0.0,  -0.8, 1.4 
};


static const std::vector<double> POS_DOWN = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

static const std::vector<double> POS_CREEP = {
    0.3, -0.4, 0.6,
    0.3, -0.4, 0.6,
    0.3, -0.4, 0.6,
    0.3, -0.4, 0.6,
};

static const std::vector<double> POS_KNEEL_PREPARE = {
    -0.1, -1.1, 0.7,
    -0.1, -1.1, 0.7,
    -0.1, -1.1, 0.7,
    -0.1, -1.1, 0.7,
};

static const std::vector<double> POS_KNEEL_CRAWL_DEFAULT = {
    -0.0, -1.2, 0.7,
    -0.0, -1.2, 0.7,
    -0.0, -1.2, 0.7,
    -0.0, -1.2, 0.7,
};

PosControlNode::PosControlNode() : Node("pos_control_node") {
    joint_cmd_pub_ =
        this->create_publisher<quad::msg::JointCmd>("/quad/joint_cmd", 10);

    encoder_sub_ = this->create_subscription<quad::msg::Encoder>(
        "/quad/encoder", 10,
        std::bind(&PosControlNode::encoder_callback, this,
                  std::placeholders::_1));

    high_cmd_sub_ = this->create_subscription<quad::msg::HighCommands>(
        "/quad/high_command", 10,
        std::bind(&PosControlNode::high_command_callback, this,
                  std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/quad/cmd_vel", 10,
        std::bind(&PosControlNode::cmd_vel_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.005),
        std::bind(&PosControlNode::update_pos_cmd, this));

    action_start_time_ = this->now();
    last_time_ = this->now();
    for (int i = 0; i < 12; i++) {
        pos_cur_[i] = 0.0;
    }

    current_state_ = STOP;

    // PD参数
    this->declare_parameter<double>("kp_hip", 0.03);
    this->declare_parameter<double>("kp_thigh", 0.03);
    this->declare_parameter<double>("kp_calf", 0.03);
    this->declare_parameter<double>("kd_hip", 3.0);
    this->declare_parameter<double>("kd_thigh", 3.0);
    this->declare_parameter<double>("kd_calf", 3.0);
    pd_param_default_[0] = this->get_parameter("kp_hip").as_double();
    pd_param_default_[1] = this->get_parameter("kp_thigh").as_double();
    pd_param_default_[2] = this->get_parameter("kp_calf").as_double();
    pd_param_default_[3] = this->get_parameter("kd_hip").as_double();
    pd_param_default_[4] = this->get_parameter("kd_thigh").as_double();
    pd_param_default_[5] = this->get_parameter("kd_calf").as_double();
    set_pd_default(pd_param_default_);

    // 关节级 PD 覆盖参数（-1.0 表示使用 phase 默认值）
    this->declare_parameter<std::vector<double>>("joint_pd_override_kp", std::vector<double>(12, -1.0));
    this->declare_parameter<std::vector<double>>("joint_pd_override_kd", std::vector<double>(12, -1.0));
    auto kp_ov = this->get_parameter("joint_pd_override_kp").as_double_array();
    auto kd_ov = this->get_parameter("joint_pd_override_kd").as_double_array();
    std::array<double,12> kp_override{}, kd_override{};
    for (int i = 0; i < 12; ++i) {
        kp_override[i] = kp_ov[i];
        kd_override[i] = kd_ov[i];
    }
    action_controller_.set_joint_overrides(kp_override, kd_override);
    
    // 姿态关节位置参数
    this->declare_parameter("pos_stand", POS_STAND);
    this->declare_parameter("pos_down", POS_DOWN);
    this->declare_parameter("pos_creep", POS_CREEP);
    this->declare_parameter("pos_kneel_prepare", POS_KNEEL_PREPARE);
    this->declare_parameter("pos_kneel_crawl", POS_KNEEL_CRAWL_DEFAULT);
    this->declare_parameter<double>("kneel_prepare_duration", 1.0);
    this->declare_parameter<double>("kneel_gait_period", 1.5);
    this->declare_parameter<double>("kneel_thigh_amplitude", 0.2);
    this->declare_parameter<double>("kneel_trigger_vel_x", 0.05);
    this->declare_parameter<double>("kneel_rear_thigh_amp_scale", 1.35);
    this->declare_parameter<double>("kneel_rear_calf_lift_offset", -0.08);
    this->declare_parameter<double>("kneel_stop_blend_duration", 1.0);
    pos_stand_ = this->get_parameter("pos_stand").as_double_array();
    pos_down_  = this->get_parameter("pos_down").as_double_array();
    pos_creep_  = this->get_parameter("pos_creep").as_double_array();
    pos_kneel_prepare_ = this->get_parameter("pos_kneel_prepare").as_double_array();
    pos_kneel_crawl_ = this->get_parameter("pos_kneel_crawl").as_double_array();
    kneel_prepare_duration_ = this->get_parameter("kneel_prepare_duration").as_double();
    kneel_gait_period_ = this->get_parameter("kneel_gait_period").as_double();
    kneel_thigh_amp_ = this->get_parameter("kneel_thigh_amplitude").as_double();
    kneel_trigger_vel_x_ = this->get_parameter("kneel_trigger_vel_x").as_double();
    kneel_rear_thigh_amp_scale_ = this->get_parameter("kneel_rear_thigh_amp_scale").as_double();
    kneel_rear_calf_lift_offset_ = this->get_parameter("kneel_rear_calf_lift_offset").as_double();
    kneel_stop_blend_duration_ = this->get_parameter("kneel_stop_blend_duration").as_double();
    kneel_gait_start_time_ = this->now();

}   

void PosControlNode::encoder_callback(
    const quad::msg::Encoder::SharedPtr msg) {
    for (int i = 0; i < 12; i++) {
        pos_cur_[i] = msg->qpos[i];
    }
}

void PosControlNode::high_command_callback(
    const quad::msg::HighCommands::SharedPtr msg) {
    bool need_transition = false;
    State_e parsed_state = current_state_;
    if (try_parse_state(msg->command, parsed_state) && parsed_state != current_state_) {
        current_state_ = parsed_state;
        need_transition = true;
    }

    Policy_e parsed_policy = static_cast<Policy_e>(current_policy_);
    if (try_parse_policy(msg->policy_switch, parsed_policy) &&
        parsed_policy != static_cast<Policy_e>(current_policy_)) {
        current_policy_ = parsed_policy;
        need_transition = true;
    }

    if (need_transition) {
        reset_action_transition();
    }
}

void PosControlNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_velocity_ = *msg;
}

void PosControlNode::update_pos_cmd() {
    std::lock_guard<std::mutex> lk(set_pos_mutex);

    rclcpp::Time t = this->now();  // 当前时间
    double time_from_start = (t - action_start_time_).seconds();
    // RCLCPP_INFO(this->get_logger(),"current state:%d",current_state_);

    // 每帧从参数服务器读取最新 kneel 参数
    pos_kneel_prepare_ = this->get_parameter("pos_kneel_prepare").as_double_array();

    // 设置轨迹
    quad::msg::JointCmd msg;
    bool is_rl_mode=false;
    const std::vector<double>* target_pos = nullptr;

    switch (current_state_) {
        case STOP:
            msg.ctrl_mode = DISABLE_MODE;
            break;
        case PASSIVE:
            msg.ctrl_mode = DAMPING_MODE;
            for(int i=0;i<12;i++)
                pos_des_cur_[i] = pos_cur_[i];
            set_pd_default(pd_param_default_);
            break;
        case FIXED_DOWN:
            msg.ctrl_mode = ENABLE_MODE;
            target_pos = &pos_down_;
            set_pd_default(pd_param_default_);
            // update_jointcmd_smoothly(pos_down_, time_from_start, pos_change_duration_);
            break;
        case FIXED_STAND:
            msg.ctrl_mode = ENABLE_MODE;
            if(current_policy_ == TROT){
                target_pos = &pos_stand_;
            }else if(current_policy_ == CREEP){
                target_pos = &pos_creep_;
            }else if(current_policy_ == UPSTAIR){
                target_pos = &pos_stand_;
            }else if(current_policy_ == KNEEL_CRAWL_POLICY){
                target_pos = &pos_kneel_crawl_;
            }
            set_pd_default(pd_param_default_);
            // update_jointcmd_smoothly(pos_stand_, time_from_start, pos_change_duration_);
            break;
        case FREE_STAND:
            msg.ctrl_mode = ENABLE_MODE;
            set_pd_default(pd_param_default_);
            break;
        case RL_MOVE:
            is_rl_mode = true;
            for(int i=0;i<12;i++)
                pos_des_cur_[i] = pos_cur_[i]; 
            break;
        case JUMP:{
            msg.ctrl_mode = ENABLE_MODE;
            auto pos_des_ = action_controller_.compute_action_cmd(
                ActionType::JUMP_ADD, pos_start_arr, time_from_start);
            for (int i = 0; i < 12; ++i){
                pos_des_cur_[i] = pos_des_[i];
            }

            std::array<double,12> kp_arr, kd_arr;
            action_controller_.get_current_pd(ActionType::JUMP_ADD, kp_arr, kd_arr);
            set_pd_arrays(kp_arr, kd_arr);

            break;
            }
        case STRIDE:{
            msg.ctrl_mode = ENABLE_MODE;
            auto pos_des_ = action_controller_.compute_action_cmd(
                ActionType::STRIDE_WALKING, pos_start_arr, time_from_start);
            for (int i = 0; i < 12; ++i){
                pos_des_cur_[i] = pos_des_[i];
            }

            std::array<double,12> kp_arr, kd_arr;
            action_controller_.get_current_pd(ActionType::STRIDE_WALKING, kp_arr, kd_arr);
            set_pd_arrays(kp_arr, kd_arr);

            break;
            }
        case SMALL_JUMP:{
            msg.ctrl_mode = ENABLE_MODE;
            auto pos_des_ = action_controller_.compute_action_cmd(
                ActionType::SMALL_JUMP, pos_start_arr, time_from_start);
            for (int i = 0; i < 12; ++i){
                pos_des_cur_[i] = pos_des_[i];
            }

            std::array<double,12> kp_arr, kd_arr;
            action_controller_.get_current_pd(ActionType::SMALL_JUMP, kp_arr, kd_arr);
            set_pd_arrays(kp_arr, kd_arr);

            break;
            }
        default:
            break;
    }
    if(target_pos != nullptr){
        update_jointcmd_smoothly(*target_pos, time_from_start, pos_change_duration_);
    }

    if (!is_rl_mode) {
        for (int i = 0; i < 12; ++i) {
            msg.pos_des[i] = pos_des_cur_[i];
            msg.kp[i] = kp_current_[i];
            msg.kd[i] = kd_current_[i];
        }
        joint_cmd_pub_->publish(msg);
    }
}

void PosControlNode::update_jointcmd_smoothly(const std::vector<double>& target,
    double time_from_start, double duration) {
    for (int i = 0; i < 12; i++) {
        pos_des_cur_[i] = cosineInterpolate(
            pos_start_[i], target[i], duration, time_from_start);
    }
}

void PosControlNode::set_pd_default(const double pd[6])
{
    for (int l = 0; l < 4; ++l) {
        kp_current_[3*l + 0] = pd[0];  // hip kp
        kp_current_[3*l + 1] = pd[1];  // thigh kp
        kp_current_[3*l + 2] = pd[2];  // calf kp
        kd_current_[3*l + 0] = pd[3];  // hip kd
        kd_current_[3*l + 1] = pd[4];  // thigh kd
        kd_current_[3*l + 2] = pd[5];  // calf kd
    }
}

void PosControlNode::set_pd_arrays(const std::array<double,12>& kp, const std::array<double,12>& kd)
{
    kp_current_ = kp;
    kd_current_ = kd;
}

bool PosControlNode::try_parse_state(int raw_state, State_e& parsed_state) const {
    if (raw_state < STOP || raw_state > SMALL_JUMP) {
        RCLCPP_WARN(this->get_logger(), "Ignore invalid state command: %d", raw_state);
        return false;
    }
    parsed_state = static_cast<State_e>(raw_state);
    return true;
}

bool PosControlNode::try_parse_policy(int raw_policy, Policy_e& parsed_policy) const {
    if (raw_policy < TROT || raw_policy > KNEEL_CRAWL_POLICY) {
        RCLCPP_WARN(this->get_logger(), "Ignore invalid policy command: %d", raw_policy);
        return false;
    }
    parsed_policy = static_cast<Policy_e>(raw_policy);
    return true;
}

void PosControlNode::reset_action_transition() {
    action_start_time_ = this->now();
    for (int i = 0; i < 12; i++) {
        pos_start_[i] = pos_des_cur_[i];
        pos_start_arr[i] = pos_start_[i];
    }
    // 重置所有动作状态
    action_controller_.reset_action(ActionType::JUMP_ADD);
    action_controller_.reset_action(ActionType::STRIDE_WALKING);
    action_controller_.reset_action(ActionType::SMALL_JUMP);
    kneel_crawl_active_ = false;
    kneel_stop_blending_ = false;
    kneel_gait_start_time_ = this->now();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
