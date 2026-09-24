#include <rclcpp/rclcpp.hpp>

#include "JointPdController.hpp"
#include "quad/msg/encoder.hpp"
#include "quad/msg/torque.hpp"
#include "quad/msg/joint_cmd.hpp"
#include "FSM.h"

// double _Kp[12]={50.0, 50.0, 50.0, 50.0, 50.0, 50.0,
//                 50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
// double _Kd[12]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
//                 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; 

/**
 * @brief 控制节点类
 */
class JointControlNode : public rclcpp::Node {
   public:
    // 构造函数
    JointControlNode() : Node("joint_control_node") {
        // // 声明参数
        // this->declare_parameter<double>("Kp", 40.0);
        // this->declare_parameter<double>("Kd", 1.0);
        
        current_ctrl_mode_ = DISABLE_MODE;
        for (int i = 0; i < 12; i++) {
            feedback_pos_[i] = 0.0;
            target_pos_[i] = 0.0;
            pd_[i].setPD(0.0, 0.0);  // 设置PD参数
        }
        
        // 创建反馈订阅者
        joint_cmd_sub_ = this->create_subscription<quad::msg::JointCmd>(
            "/quad/joint_cmd", 10,
            std::bind(&JointControlNode::jointcmd_callback, this,
                      std::placeholders::_1));


        // 创建反馈订阅者
        encoder_sub_ = this->create_subscription<quad::msg::Encoder>(
            "/quad/encoder", 10,
            std::bind(&JointControlNode::encoder_callback, this,
                      std::placeholders::_1));

        // 创建控制量发布者
        torque_pub_ = this->create_publisher<quad::msg::Torque>("/quad/joint_torques", 10);

        // 创建定时器，周期为5ms
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.005),
            std::bind(&JointControlNode::control_loop, this));

        last_time_ = this->now();  // 初始化上次时间
        
    }

   private:
    void jointcmd_callback(const quad::msg::JointCmd::SharedPtr msg) {
        current_ctrl_mode_ = static_cast<Control_Mode_e>(msg->ctrl_mode);  // 获取控制模式
        for (int i = 0; i < 12; i++) {
            Kp_[i] = msg->kp[i]*2000;  // 获取Kp参数
            Kd_[i] = msg->kd[i]/3;  // 获取Kd参数
            target_pos_[i] = msg->pos_des[i];  // 获取目标位置
        }
    }

    // 接收状态反馈回调函数
    void encoder_callback(const quad::msg::Encoder::SharedPtr msg) {
        for (int i = 0; i < 12; i++) {
            feedback_pos_[i] = msg->qpos[i];  // 获取反馈位置
            feedback_vel_[i] = msg->qvel[i];  // 获取反馈速度
        }
    }
    // 定时器中断回调函数(控制循环)
    void control_loop() {
        // double Kp = this->get_parameter("Kp").as_double();
        // double Kd = this->get_parameter("Kd").as_double();
        for (int i = 0; i < 12; i++) {
            pd_[i].setPD(Kp_[i], Kd_[i]);  // 更新PID参数
        }

        // auto now = this->now();
        // double dt = (now - last_time_).seconds();
        // last_time_ = now;

        // if (dt <= 0.0 || dt > 0.3) {  // dt异常
        //     dt = 0.001;
        // }

        /* 发布控制量 */
        quad::msg::Torque torque_msg;
        switch (current_ctrl_mode_) {
            case DISABLE_MODE:
                for (int i = 0; i < 12; i++) {
                    torque_msg.joint_torques[i] = 0.0;
                }
                break;
            case ENABLE_MODE:
                for (int i = 0; i < 12; i++) {
                    // torque_msg.joint_torques[i] =
                    //     pd_[i].calcPD(target_pos_[i], feedback_pos_[i], dt);
                    torque_msg.joint_torques[i] =
                        pd_[i].calcPD(target_pos_[i], feedback_pos_[i], feedback_vel_[i]);
                }
                break;
            case DAMPING_MODE:
                for (int i = 0; i < 12; i++) {
                    torque_msg.joint_torques[i] = -3.0*feedback_vel_[i];
                }
                break;
            default:
                for (int i = 0; i < 12; i++) {
                    torque_msg.joint_torques[i] = 0.0;
                }
                break;
        }
        torque_pub_->publish(torque_msg);
    }

    /* 声明 */
    rclcpp::TimerBase::SharedPtr control_timer_;  // 定时器指针
    rclcpp::Publisher<quad::msg::Torque>::SharedPtr
        torque_pub_;  // 话题发布者指针
    rclcpp::Subscription<quad::msg::Encoder>::SharedPtr
        encoder_sub_;  // 话题订阅者指针
    rclcpp::Subscription<quad::msg::JointCmd>::SharedPtr
        joint_cmd_sub_;  // 话题订阅者指针
    Control_Mode_e current_ctrl_mode_; // 当前控制模式
    JointPdController pd_[12];    // PID 控制器对象
    double Kp_[12],Kd_[12];  // PD参数
    double feedback_pos_[12];   // 反馈位置
    double feedback_vel_[12];   // 反馈位置
    double target_pos_[12];     // 目标位置
    rclcpp::Time last_time_;  // 上次时间
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针*/
    auto node = std::make_shared<JointControlNode>();
    /*运行节点*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}