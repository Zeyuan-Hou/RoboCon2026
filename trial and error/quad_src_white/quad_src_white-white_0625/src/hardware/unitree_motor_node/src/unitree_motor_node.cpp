#include "unitree_motor_node.h"

UnitreeMotorNode::UnitreeMotorNode() : Node("MotorNode"), running_(true) {
    // 1. 初始化 ROS 接口 (维持原有的 Topic 名称不变)
    encoder_pub_ =
        this->create_publisher<quad::msg::Encoder>("/quad/encoder", 10);
    joint_cmd_sub_ = this->create_subscription<quad::msg::JointCmd>(
        "/quad/joint_cmd", 10,
        std::bind(&UnitreeMotorNode::jointcmd_callback, this,
                  std::placeholders::_1));

    // thermal_load_pub_ =
    // this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_debug/thermal_load",
    // 10); safe_pos_pub_     =
    // this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_debug/safe_pos",
    // 10);

    // thermal_load_array_.fill(0.0f);
    // safe_pos_array_.fill(0.0f);

    // 初始化标定参数
    param_init();

    // 2. 初始化底层目标指令缓存（先设置为 BRAKE 模式确保安全）
    for (int i = 0; i < 12; ++i) {
        target_cmds_[i].motorType = MotorType::GO_M8010_6;
        target_cmds_[i].mode =
            queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        target_cmds_[i].id = LOGICAL_TO_UNITREE_MAP[i].motor_id;
        target_cmds_[i].q = 0.0;
        target_cmds_[i].dq = 0.0;
        target_cmds_[i].tau = 0.0;
        target_cmds_[i].kp = 0.0;
        target_cmds_[i].kd = 0.0;
    }

    // 3. 打开 4 个硬件串口
    try {
        serial_[0] = std::make_shared<SerialPort>("/dev/ttyUSB1");
        serial_[1] = std::make_shared<SerialPort>("/dev/ttyUSB2");
        serial_[2] = std::make_shared<SerialPort>("/dev/ttyUSB3");
        serial_[3] = std::make_shared<SerialPort>("/dev/ttyUSB4");
        RCLCPP_INFO(this->get_logger(),
                    "All 4 USB Serial Ports initialized successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial ports: %s",
                     e.what());
    }

    // 4. 启动 4 个独立的底层通讯线程
    for (int i = 0; i < 4; ++i) {
        leg_threads_[i] =
            std::thread(&UnitreeMotorNode::legControlLoop, this, i);
    }

    // 5. 启动 ROS 2 定时器，负责对外发布机器人当前状态 (200Hz - 5ms)
    update_data_timer_ = this->create_wall_timer(
        2ms, std::bind(&UnitreeMotorNode::publishState, this));
}

UnitreeMotorNode::~UnitreeMotorNode() {
    running_ = false;

    // 等待所有底层线程结束
    for (auto& t : leg_threads_) {
        if (t.joinable()) t.join();
    }

    // 失能所有电机，发送 BRAKE
    MotorCmd stop_cmd;
    MotorData dummy_data;
    stop_cmd.motorType = MotorType::GO_M8010_6;
    stop_cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    stop_cmd.q = 0;
    stop_cmd.dq = 0;
    stop_cmd.tau = 0;
    stop_cmd.kp = 0;
    stop_cmd.kd = 0;

    for (int i = 0; i < 4; i++) {
        for (int m = 0; m < 3; m++) {
            stop_cmd.id = m;
            if (serial_[i]) serial_[i]->sendRecv(&stop_cmd, &dummy_data);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Motors braked and Node shutdown safely.");
}

void UnitreeMotorNode::param_init() {
    for (int i = 0; i < 12; i++) {
        motor_to_joint_scale_.at(i) = 1 / MOTOR_REDUCTION_RATIO.at(i) /
                                      EXTERNAL_REDUCTION_RATIO.at(i) *
                                      LEG_MIRROR_COE.at(i);
        zero_offset_.at(i) = JOINT_INIT_POS.at(i) - REAL_INIT_POS.at(i);
#ifdef CALIBRATION
        zero_offset_.at(i) = 0.0;
#endif
    }
}

// 接收高层控制指令
void UnitreeMotorNode::jointcmd_callback(
    const quad::msg::JointCmd::SharedPtr msg) {
    // 遍历 12 个逻辑关节
    for (int i = 0; i < 12; ++i) {
        int serial_idx = LOGICAL_TO_UNITREE_MAP[i].serial_idx;

        MotorCmd cmd;
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id = LOGICAL_TO_UNITREE_MAP[i].motor_id;

        if (msg->ctrl_mode == DISABLE_MODE) {
            cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
            cmd.q = 0.0;
            cmd.dq = 0.0;
            cmd.tau = 0.0;
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        } else if (msg->ctrl_mode == DAMPING_MODE) {
            cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
            cmd.q = 0.0;
            cmd.dq = 0.0;
            cmd.tau = 0.0;
            cmd.kp = 0.0;
            cmd.kd = 0.2;
        } else if (msg->ctrl_mode == ENABLE_MODE) {
            cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
            cmd.q =
                (msg->pos_des[i] - zero_offset_[i]) / motor_to_joint_scale_[i];
            cmd.dq = 0.0;
            cmd.kp = msg->kp[i];
            cmd.kd = msg->kd[i];

        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d",
                         msg->ctrl_mode);
        }

        // 存入共享内存（针对这一条腿加锁）
        {
            std::lock_guard<std::mutex> lock(leg_mutex_[serial_idx]);
            target_cmds_[i] = cmd;
        }
    }
}

// 并行底层通讯线程 (目标频率 500Hz ~ 1000Hz)
void UnitreeMotorNode::legControlLoop(int leg_id) {
    // 每个线程负责 1 条腿的 3 个电机
    MotorCmd local_cmds[3];
    MotorData local_data[3];

    while (running_ && rclcpp::ok()) {
        // 1. 从共享内存中取出当前腿的 3 个目标指令
        {
            std::lock_guard<std::mutex> lock(leg_mutex_[leg_id]);
            for (int i = 0; i < 12; ++i) {
                if (LOGICAL_TO_UNITREE_MAP[i].serial_idx == leg_id) {
                    local_cmds[LOGICAL_TO_UNITREE_MAP[i].motor_id] =
                        target_cmds_[i];
                }
            }
        }

        // 2. 串口收发：依次向这条腿的 3
        // 个电机发送指令（该过程独立，不锁主线程）
        for (int m = 0; m < 3; ++m) {
            // 注意：Unitree SDK 要求发送和接收的 motorType 对齐
            local_data[m].motorType = MotorType::GO_M8010_6;
            if (serial_[leg_id]) {
                serial_[leg_id]->sendRecv(&local_cmds[m], &local_data[m]);
            }
        }

        // 3. 将收到的 3 个电机状态写回共享内存
        {
            std::lock_guard<std::mutex> lock(leg_mutex_[leg_id]);
            for (int i = 0; i < 12; ++i) {
                if (LOGICAL_TO_UNITREE_MAP[i].serial_idx == leg_id) {
                    current_states_[i] =
                        local_data[LOGICAL_TO_UNITREE_MAP[i].motor_id];
                }
            }
        }

        // 4. 睡眠保证通讯频率：设置 2ms (500Hz) 对于 3 个串联电机足够且稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// 主线程状态发布 (200Hz)
void UnitreeMotorNode::publishState() {
    for (int i = 0; i < 12; ++i) {
        int serial_idx = LOGICAL_TO_UNITREE_MAP[i].serial_idx;
        MotorData data;

        // 安全读取当前状态
        {
            std::lock_guard<std::mutex> lock(leg_mutex_[serial_idx]);
            data = current_states_[i];
        }

        // 【坐标系转换：电机空间 -> 关节空间】
        encoder_msg_.qpos[i] =
            data.q * motor_to_joint_scale_[i] + zero_offset_[i];
        encoder_msg_.qvel[i] = data.dq * motor_to_joint_scale_[i];
        // encoder_msg_.tau[i] = data.tau * LEG_MIRROR_COE[i];

        // // 提取温度作为热负载调试参数
        // thermal_load_array_[i] = static_cast<float>(data.temp);
    }

    // 发布编码器话题给上层里程计 / 状态估计模块
    encoder_pub_->publish(encoder_msg_);

    // // 发布 Debug 信息
    // std_msgs::msg::Float32MultiArray thermal_msg;
    // thermal_msg.data = std::vector<float>(thermal_load_array_.begin(),
    // thermal_load_array_.end()); thermal_load_pub_->publish(thermal_msg);

    // std_msgs::msg::Float32MultiArray safe_pos_msg;
    // safe_pos_msg.data = std::vector<float>(safe_pos_array_.begin(),
    // safe_pos_array_.end()); safe_pos_pub_->publish(safe_pos_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeMotorNode>());
    rclcpp::shutdown();
    return 0;
}