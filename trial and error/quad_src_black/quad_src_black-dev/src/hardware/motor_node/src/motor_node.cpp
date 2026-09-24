#include "motor_node.h"

/**
 * @brief send motor command to HexapodCommBoard through UDP
 */
void MotorNode::sendMotorCommand(void) {
    udp_comm_.setSendData(udp_send_data_);
    udp_comm_.send();
}

/**
 * @brief update motor command to HexapodCommBoard through UDP
 */
void MotorNode::updateMotorCommand(void) {
    /* 设置电机参数并UDP发送给主控 */
    for (int i = 0; i < 12; i++) {
        // 【关键修改】获取逻辑关节 i 对应的物理电机索引 hw_idx
        // i 是上层算法的顺序 (FL, FR...)
        // hw_idx 是实际插在板子上的顺序
        int hw_idx = JOINT_TO_MOTOR_MAP[i];
        
        // 设置电机控制参数
        // 位置模式
        udp_send_data_.state = state_;
        
        // 注意：kp_, kd_, pos_des_ 使用逻辑索引 i (来自上层)
        // udp_send_data_.udp_motor_send 使用物理索引 hw_idx (发给底层)
        udp_send_data_.udp_motor_send[hw_idx].torque = 0.0;
        udp_send_data_.udp_motor_send[hw_idx].vel = 0.0;
        
    #ifdef POSITION_ERROR_LIMIT 
        safe_pos_des_.at(i) = limitJointPosition(i);
        last_pos_des_.at(i) = safe_pos_des_.at(i);

        // TODO 使用safe_pos_des_
        udp_send_data_.udp_motor_send[hw_idx].pos =
            (safe_pos_des_.at(i) - zero_offset_.at(i)) / motor_to_joint_scale_.at(i);
    #else
    udp_send_data_.udp_motor_send[hw_idx].pos =
                (pos_des_.at(i) - zero_offset_.at(i)) / motor_to_joint_scale_.at(i);
    #endif
    udp_send_data_.udp_motor_send[hw_idx].kp = kp_.at(i);
    udp_send_data_.udp_motor_send[hw_idx].kd = kd_.at(i);

    }
    // 发布所有关节的热负载数据
    auto thermal_msg = std_msgs::msg::Float32MultiArray();
    thermal_msg.data.assign(thermal_load_array_.begin(), thermal_load_array_.end());
    thermal_load_pub_->publish(thermal_msg);
    
    // 发布所有关节的安全位置数据
    auto pos_msg = std_msgs::msg::Float32MultiArray();
    pos_msg.data.assign(safe_pos_array_.begin(), safe_pos_array_.end());
    safe_pos_pub_->publish(pos_msg);

    // // 力矩模式
    //   for (int i = 0; i < 12; i++) {
    //     double torque_temp = fmax(
    //         fmin(torque_msg_.joint_torques[i] +
    //         wbc_torque_msg_.joint_torques[i],
    //              max_torque),
    //         -max_torque);

    //     udp_send_data_.udp_motor_send[i].torque =
    //         torque_temp / MOTOR_REDUCTION_RATIO.at(i);
    //   }
    //   udp_send_data_.udp_motor_send[0].torque = 0.01;
}

/**
 * @brief receive motor feedback from HexapodCommBoard through UDP
 */
void MotorNode::receiveFeedback(void) {
    // non-blocking
    // udp_comm_.send();
    if (udp_comm_.receive(10))  // 10ms
    {
        // if (!udp_ready_flag_)
        //   udp_ready_flag_ = true;
        udp_receive_data_ = udp_comm_.getReceiveData();
    }
    // blocking
    // udp_comm_.receive();
}

void MotorNode::updateMotorFeedback(void) {
    // 1. 计算相对时间
    static auto start_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(current_time - start_time).count();

    bool error_detected_in_this_frame = false;

    for (int i = 0; i < 12; i++) {
        int hw_idx = JOINT_TO_MOTOR_MAP[i];
        auto& motor_data = udp_receive_data_.udp_motor_receive[hw_idx];

        // 计算实际关节位置 (放入 encoder_msg_ 供发布)
        encoder_msg_.qpos.at(i) = motor_data.pos * motor_to_joint_scale_.at(i) + zero_offset_.at(i);
        encoder_msg_.qvel.at(i) = motor_data.vel * motor_to_joint_scale_.at(i);

        // 错误检测与打印（带频率限制）
        if(motor_data.error > 0.1) {
            error_detected_in_this_frame = true;
            auto now = this->now();
            double elapsed_error = (now - last_error_print_time_.at(i)).seconds();
            if(elapsed_error >= ERROR_PRINT_INTERVAL) {
                RCLCPP_INFO(this->get_logger(),"[Joint:%d|HW:%d] error:%f", i, hw_idx, motor_data.error);
                last_error_print_time_.at(i) = now;
            }
        }
        
        if(motor_data.communication_rate < 300) {
            auto now = this->now();
            double elapsed_comm = (now - last_comm_print_time_.at(i)).seconds();
            if(elapsed_comm >= ERROR_PRINT_INTERVAL) {
                RCLCPP_INFO(this->get_logger(),"[Joint:%d|HW:%d] comm_rate:%d", i, hw_idx, motor_data.communication_rate);
                last_comm_print_time_.at(i) = now;
            }
        }
        
        // ==========================================
        // 【修改】写入详细位置信息
        // ==========================================
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(5) << elapsed_sec << "," 
                      << i << "," 
                      << hw_idx << "," 
                      << (int)motor_data.error << "," 
                      << (int)motor_data.temp << ","   
                      << std::setprecision(4) << motor_data.torque << "," 
                      << std::setprecision(4) << motor_data.vel << ","
                      
                      // 1. Pos_Raw: 电机原始反馈 (便于排查零点漂移)
                      << std::setprecision(4) << motor_data.pos << ","
                      
                      // 2. Pos_Real: 换算后的关节实际弧度 (用于和目标值画在一张图上)
                      << std::setprecision(4) << encoder_msg_.qpos.at(i) << ","
                      
                      // 3. Pos_Des:  目标位置 (查看跟踪滞后情况)
                      << std::setprecision(4) << pos_des_.at(i) << "\n";
        }
    }

    // 刷新文件流
    static int flush_count = 0;
    if (log_file_.is_open()) {
        if (error_detected_in_this_frame || (++flush_count > 50)) {
            log_file_.flush();
            flush_count = 0;
        }
    }
}

/**
 * @brief read feedback in elspider mini interface
 */
void MotorNode::read(void) {
    //   if (!feedback_ready_flag_ && udp_ready_flag_)
    //     feedback_ready_flag_ = true;
    receiveFeedback();
    updateMotorFeedback();
    // updateImu();
}

/**
 * @brief write commands in elspider mini interface
 */
void MotorNode::write(void) {
    updateMotorCommand();
    sendMotorCommand();
}

void MotorNode::update_data(void) {
    write();
    read();
    encoder_pub_->publish(encoder_msg_);
}

double MotorNode::limitJointPosition(int joint_idx){
    /* 参数设置 */
    float base_error_limit = 0.7;    // 最大error, 计算: tau_limit / 40
    float filter_alpha = 0.2;  // 低通滤波系数
    // 力矩累积保护         
    float thermal_tau = 8.0;         // 热时间常数 (秒)
    float dt = 0.005;                // 控制周期 (秒), 200Hz
    
    /*获取电机参数*/ 
    auto& motor_data = udp_receive_data_.udp_motor_receive[JOINT_TO_MOTOR_MAP[joint_idx]];
    
    double q_des = pos_des_.at(joint_idx);      // 目标位置
    double q = encoder_msg_.qpos.at(joint_idx); // 当前位置
    double temp = motor_data.temp;           // 温度
    double tau_estimated = motor_data.torque;   // 估算力矩 (使用反馈力矩)

    // 热负载模型：指数衰减积分
    float decay = exp(-dt / thermal_tau);
    thermal_load_.at(joint_idx) = thermal_load_.at(joint_idx) * decay + tau_estimated * tau_estimated * dt;
    
    float L =  thermal_load_.at(joint_idx);
    float thermal_scale = 1.0;
    
    // 记录热负载值
    // RCLCPP_INFO(this->get_logger(), "Joint %d - Thermal Load: %.2f", joint_idx, L);
    
    if(L < 10.0) 
        thermal_scale = 1.0;
    else if(L < 20.0) {
        thermal_scale = 0.8;
        auto now = this->now();
        double elapsed = (now - last_thermal_warn_time_.at(joint_idx)).seconds();
        if(elapsed >= ERROR_PRINT_INTERVAL) {
            RCLCPP_WARN(this->get_logger(), "Joint %d - Thermal Load High! Scale reduced to 0.8", joint_idx);
            last_thermal_warn_time_.at(joint_idx) = now;
        }
    }
    else if(L < 30.0) {
        thermal_scale = 0.6;
        auto now = this->now();
        double elapsed = (now - last_thermal_warn_time_.at(joint_idx)).seconds();
        if(elapsed >= ERROR_PRINT_INTERVAL) {
            RCLCPP_WARN(this->get_logger(), "Joint %d - Thermal Load Very High! Scale reduced to 0.6", joint_idx);
            last_thermal_warn_time_.at(joint_idx) = now;
        }
    }
    else {
        thermal_scale = 0.4;
        auto now = this->now();
        double elapsed = (now - last_thermal_warn_time_.at(joint_idx)).seconds();
        if(elapsed >= ERROR_PRINT_INTERVAL) {
            RCLCPP_ERROR(this->get_logger(), "Joint %d - Thermal Load Critical! Scale reduced to 0.4", joint_idx);
            last_thermal_warn_time_.at(joint_idx) = now;
        }
    }
    
    // 计算位置误差
    float err = q_des - q;
    // 应用热负载缩放后的误差限制
    float scaled_error_limit = base_error_limit * thermal_scale;
    
    // RCLCPP_INFO(this->get_logger(), "Joint %d - Error Limit: %.3f (base: %.3f, scale: %.2f)", 
    //             joint_idx, scaled_error_limit, base_error_limit, thermal_scale);
    
    err = std::clamp(err, -scaled_error_limit, scaled_error_limit);
    
    // 计算安全目标位置
    float safe_q_des = q + err;
    
    // 低通滤波
    safe_q_des = filter_alpha * safe_q_des + (1 - filter_alpha) * last_pos_des_.at(joint_idx);
    
    // RCLCPP_INFO(this->get_logger(), "Joint %d - Safe Position: %.4f (Original Desired: %.4f, Current Position: %.4f)", 
    //             joint_idx, safe_q_des, q_des, q);

    // 存储数据用于发布
    thermal_load_array_[joint_idx] = L;
    safe_pos_array_[joint_idx] = safe_q_des;

    return safe_q_des;  
}


  
void MotorNode::param_init(void) {
    udp_send_data_.state = 0x00;  // 控制状态

    kp_.fill(0.0);
    kd_.fill(0.0);
    pos_des_.fill(0.0);

    last_pos_des_.fill(0.0);

    // 初始化错误打印时间戳
    auto init_time = this->now();
    for (int i = 0; i < 12; i++) {
        last_error_print_time_.at(i) = init_time;
        last_comm_print_time_.at(i) = init_time;
        last_thermal_warn_time_.at(i) = init_time;
    }

    for (int i = 0; i < 12; i++) {
        motor_to_joint_scale_.at(i) = 1 / MOTOR_REDUCTION_RATIO.at(i) /
                                      EXTERNAL_REDUCTION_RATIO.at(i) *
                                      LEG_MIRROR_COE.at(i);
        zero_offset_.at(i) = JOINT_INIT_POS.at(i) - REAL_INIT_POS.at(i);
#ifdef CALIBRATION
        zero_offset_.at(i) = 0.0;
#endif
    }
    // ==========================================
    // 【新增】日志文件初始化 (指定绝对路径)
    // ==========================================
    std::string log_path = "/home/cat/hitcrt_quad_ws/src/hardware/motor_node/log/motor_log.csv";

    log_file_.open(log_path, std::ios::out | std::ios::trunc);

    if (log_file_.is_open())
    {
        // 写入表头: 
        // Pos_Raw: 电机端原始数据 (多圈角度)
        // Pos_Real: 换算后的关节实际角度 (弧度)
        // Pos_Des:  上位机下发的目标角度 (弧度)
        log_file_ << "Time_Sec,Logic_ID,HW_ID,Error_Code,Temp,Torque,Vel,Pos_Raw,Pos_Real,Pos_Des\n";
        
        RCLCPP_INFO(this->get_logger(), "Log file opened successfully at: %s", log_path.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "FAILED to open log file at: %s", log_path.c_str());
    }
}

void MotorNode::jointcmd_callback(const quad::msg::JointCmd::SharedPtr msg) {
    state_ = msg->ctrl_mode;
    for (int i = 0; i < 12; i++) {
        kp_.at(i) = msg->kp.at(i);
        kd_.at(i) = msg->kd.at(i);
        pos_des_.at(i) = msg->pos_des.at(i);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<MotorNode>(kLocalPort, kTargetIp, kTargetPort));
    rclcpp::shutdown();
    return 0;
}