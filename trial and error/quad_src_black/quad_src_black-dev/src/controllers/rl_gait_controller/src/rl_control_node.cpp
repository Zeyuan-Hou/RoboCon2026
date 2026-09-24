#include "rl_control_node.h"

#include <algorithm>
#include <iomanip>

// 初始化静态常量
static const std::vector<double> DEFAULT_ANGLES_TROT = {
    0.1, -0.8, 1.4, 0.1, -0.8, 1.4, 0.1, -0.8, 1.4, 0.1, -0.8, 1.4};

static const std::vector<double> DEFAULT_ANGLES_CREEP = {
    0.3, -0.4, 0.6, 0.3, -0.4, 0.6, 0.3, -0.4, 0.6, 0.3, -0.4, 0.6,
};

static const std::vector<double> DEFAULT_ANGLES_UPSTAIR = {
    0.1, -0.8, 1.4, 0.1, -0.8, 1.4, 0.1, -0.8, 1.4, 0.1, -0.8, 1.4,
};

static const std::vector<double> DEFAULT_ANGLES_KNEEL_CRAWL = {
    -0.0, -1.1, 0.8, -0.0, -1.1, 0.8, -0.0, -1.1, 0.8, -0.0, -1.1, 0.8,
};

RLControlNode::RLControlNode() : Node("rl_interface_node") {
    // 订阅者
    state_sub_ = this->create_subscription<quad::msg::State>(
        "/quad/test_estm_state", 10,
        std::bind(&RLControlNode::state_callback, this, std::placeholders::_1));
    encoder_sub_ = this->create_subscription<quad::msg::Encoder>(
        "/quad/encoder", 10,
        std::bind(&RLControlNode::encoder_callback, this,
                  std::placeholders::_1));

    high_cmd_sub_ = this->create_subscription<quad::msg::HighCommands>(
        "/quad/high_command", 10,
        std::bind(&RLControlNode::high_command_callback, this,
                  std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/quad/quad_output", 10,
        std::bind(&RLControlNode::rl_action_callback, this,
                  std::placeholders::_1));

    // 发布者
    obs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/quad/obs_data", 10);
    joint_cmd_pub_ =
        this->create_publisher<quad::msg::JointCmd>("/quad/joint_cmd", 10);
    policy_select_pub_ =
        this->create_publisher<std_msgs::msg::Int8>("/quad/policy_select", 10);

    // 定时器 (50Hz)
    timer_ =
        this->create_wall_timer(std::chrono::duration<double>(0.02),
                                std::bind(&RLControlNode::update_rl_obs, this));

    std::fill(last_action_.begin(), last_action_.end(), 0.0);
    current_state_ = STOP;

    this->declare_parameter<double>("kp_hip", 0.01);
    this->declare_parameter<double>("kp_thigh", 0.01);
    this->declare_parameter<double>("kp_calf", 0.02);
    this->declare_parameter<double>("kd_hip", 3.0);
    this->declare_parameter<double>("kd_thigh", 3.0);
    this->declare_parameter<double>("kd_calf", 3.0);

    this->declare_parameter("default_angles_trot", DEFAULT_ANGLES_TROT);
    this->declare_parameter("default_angles_creep", DEFAULT_ANGLES_CREEP);
    this->declare_parameter("default_angles_upstair", DEFAULT_ANGLES_UPSTAIR);
    this->declare_parameter("default_angles_kneel_crawl",
                            DEFAULT_ANGLES_KNEEL_CRAWL);
    default_angles_trot_ =
        this->get_parameter("default_angles_trot").as_double_array();
    default_angles_creep_ =
        this->get_parameter("default_angles_creep").as_double_array();
    default_angles_upstair_ =
        this->get_parameter("default_angles_upstair").as_double_array();
    default_angles_kneel_crawl_ =
        this->get_parameter("default_angles_kneel_crawl").as_double_array();

    default_dof_angles_ = default_angles_trot_;  // 默认为trot

    obs_queue_.assign(obs_history_length_,
                      std::array<double, 45>{});  // 初始化观测队列长度,防止
}

void RLControlNode::state_callback(const quad::msg::State::SharedPtr msg) {
    sensor_.imu = msg->imu_world;
}

void RLControlNode::encoder_callback(const quad::msg::Encoder::SharedPtr msg) {
    sensor_.encoder = *msg;
}

void RLControlNode::high_command_callback(
    const quad::msg::HighCommands::SharedPtr msg) {
    current_state_ = static_cast<State_e>(msg->command);
    if (msg->policy_switch != current_policy_) {
        current_policy_ = static_cast<Policy_e>(msg->policy_switch);
        // 切换用于计算观测值的默认角度
        if (current_policy_ == TROT) {
            default_dof_angles_ = default_angles_trot_;
            // RCLCPP_INFO(this->get_logger(), "Switched to CREEP default
            // angles");
        } else if (current_policy_ == CREEP) {
            default_dof_angles_ = default_angles_creep_;
            // RCLCPP_INFO(this->get_logger(), "Switched to TROT default
            // angles");
        } else if (current_policy_ == UPSTAIR) {
            default_dof_angles_ = default_angles_upstair_;
            // RCLCPP_INFO(this->get_logger(), "Switched to TROT default
            // angles");
        } else if (current_policy_ == KNEEL_CRAWL_POLICY) {
            default_dof_angles_ = default_angles_kneel_crawl_;
        }

        // 发布目标策略
        std_msgs::msg::Int8 policy_msg;
        policy_msg.data = static_cast<int8_t>(current_policy_);
        policy_select_pub_->publish(policy_msg);
    }
    joy_.lin_x = msg->lin_x;
    joy_.lin_y = msg->lin_y;
    joy_.ang_yaw = msg->ang_yaw;
}

void RLControlNode::rl_action_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(set_pos_mutex);
    if (current_state_ != RL_MOVE) return;

    quad::msg::JointCmd pub_msg;
    pub_msg.ctrl_mode = ENABLE_MODE;
    std::array<double, 12> current_action;

    // PD参数
    double Kp_hip = this->get_parameter("kp_hip").as_double();
    double Kp_thigh = this->get_parameter("kp_thigh").as_double();
    double Kp_calf = this->get_parameter("kp_calf").as_double();
    double Kd_hip = this->get_parameter("kd_hip").as_double();
    double Kd_thigh = this->get_parameter("kd_thigh").as_double();
    double Kd_calf = this->get_parameter("kd_calf").as_double();
    for (int leg = 0; leg < 4; leg++) {
        pub_msg.kp[0 + leg * 3] = Kp_hip;
        pub_msg.kp[1 + leg * 3] = Kp_thigh;
        pub_msg.kp[2 + leg * 3] = Kp_calf;
        pub_msg.kd[0 + leg * 3] = Kd_hip;
        pub_msg.kd[1 + leg * 3] = Kd_thigh;
        pub_msg.kd[2 + leg * 3] = Kd_calf;
    }

    // 获取当前Action并限幅
    for (int i = 0; i < 12; ++i) {
        current_action[i] =
            std::clamp((double)msg->data[i], -normalization_.clip_actions,
                       normalization_.clip_actions);
        last_action_[i] = current_action[i];  // 更新历史 action
    }
    // 处理Action
    for (int i = 0; i < 12; ++i) {
        double action_scaled = last_action_[i] * control_.action_scale;
        // hip_reduction
        if (i % 3 == 0) {  // 髋关节索引[0,3,6,9]
            action_scaled *= control_.hip_reduction;
        }
        pub_msg.pos_des[i] = action_scaled + default_dof_angles_[i];
    }

    joint_cmd_pub_->publish(pub_msg);
}

void RLControlNode::save_obs_to_file() {
    std::ofstream ofs(obs_filename_,
                      std::ios::out | std::ios::trunc);  // 覆盖写入文件

    if (!ofs.is_open()) {
        std::cerr << "Error: Could not open file " << obs_filename_
                  << " for writing." << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(8);

    // 遍历队列中的每一帧观测值
    for (const auto& obs_frame : obs_queue_) {
        // 遍历观测值数组中的每一个元素
        for (size_t i = 0; i < obs_frame.size(); ++i) {
            // RKNN 节点需要浮点数作为输入，这里以高精度 double 写入
            ofs << obs_frame[i];

            // 元素之间用空格分隔，确保 load_input_data 可以正确读取
            if (i < obs_frame.size() - 1) {
                ofs << ", ";
            }
        }
        // 每一帧观测值占一行，或者在帧之间换行
        ofs << ",\n";
    }

    ofs.close();
    // std::cout << "\n✅ Observation data (Total " << obs_queue_.size() * 45 <<
    // " values) saved to " << obs_filename_ << std::endl;
}

Vec3 RLControlNode::quatRotateInverse(const Vec3& v, float x, float y, float z,
                                      float w) {
    Vec3 r{-x, -y, -z};
    Vec3 t;
    t.x = 2.0f * (r.y * v.z - r.z * v.y);
    t.y = 2.0f * (r.z * v.x - r.x * v.z);
    t.z = 2.0f * (r.x * v.y - r.y * v.x);

    Vec3 out;
    out.x = v.x + w * t.x + (r.y * t.z - r.z * t.y);
    out.y = v.y + w * t.y + (r.z * t.x - r.x * t.z);
    out.z = v.z + w * t.z + (r.x * t.y - r.y * t.x);
    return out;
}

void RLControlNode::update_rl_obs() {
    std::array<double, 45> obs;

    // 归一化指令映射：直接使用接收到的值（已在state_machine中完成限幅和符号反转）
    cmd_.x = joy_.lin_x;
    cmd_.y = joy_.lin_y;
    cmd_.dyaw = joy_.ang_yaw;

    // 0-2: 控制指令
    obs[0] = cmd_.x * normalization_.obs_scales.lin_vel;
    obs[1] = cmd_.y * normalization_.obs_scales.lin_vel;
    obs[2] = cmd_.dyaw * normalization_.obs_scales.ang_vel;

    obs[3] = sensor_.imu.ang_vel[0] * normalization_.obs_scales.ang_vel;
    obs[4] = sensor_.imu.ang_vel[1] * normalization_.obs_scales.ang_vel;
    obs[5] = sensor_.imu.ang_vel[2] * normalization_.obs_scales.ang_vel;

    // 6-8: 重力在机体坐标系投影
    Vec3 g_world = {0.0f, 0.0f, -1.0f};
    Vec3 g_body = quatRotateInverse(g_world, sensor_.imu.x, sensor_.imu.y,
                                    sensor_.imu.z, sensor_.imu.w);
    obs[6] = g_body.x;
    obs[7] = g_body.y;
    obs[8] = g_body.z;

    // 9-20: 相对默认角度的关节位置
    for (int i = 0; i < 12; i++) {
        obs[9 + i] = (sensor_.encoder.qpos[i] - default_dof_angles_[i]) *
                     normalization_.obs_scales.dof_pos;
    }
    // 21-32: 关节速度
    for (int i = 0; i < 12; i++) {
        obs[21 + i] =
            sensor_.encoder.qvel[i] * normalization_.obs_scales.dof_vel;
    }
    // 33-44: 上一步 Action
    std::copy(last_action_.begin(), last_action_.end(), obs.begin() + 33);

    // 维护历史观测队列 (最新在前,最旧在后)
    obs_queue_.push_front(obs);
    if (obs_queue_.size() > (size_t)obs_history_length_) {
        obs_queue_.pop_back();
    }

    // 构造并发布 MultiArray (45 * 6 = 270)
    if (obs_queue_.size() == (size_t)obs_history_length_) {
        std_msgs::msg::Float32MultiArray obs_msg;
        obs_msg.data.reserve(270);
        for (const auto& frame : obs_queue_) {
            for (double val : frame) {
                obs_msg.data.push_back(static_cast<float>(val));
            }
        }
        obs_pub_->publish(obs_msg);
    }

    // or save as file
    static bool first = true;
    if (first) {
        save_obs_to_file();
        first = false;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLControlNode>());
    rclcpp::shutdown();
    return 0;
}