#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>

enum class PumpCommand : int8_t
{
    STOP = 0,
    GRAB = 1,
    RELEASE = 2
};

enum class PumpState
{
    IDLE,
    HOLDING,
    RELEASING,
    STOPPED
};

class PumpNode : public rclcpp::Node
{
public:
    PumpNode() : Node("pump_controller_node"), pump_line_(nullptr), valve_line_(nullptr)
    {
        this->declare_parameter("pump_chip", "gpiochip1");
        this->declare_parameter("pump_line", 14);
        this->declare_parameter("valve_chip", "gpiochip1");
        this->declare_parameter("valve_line", 15);
        this->declare_parameter("cmd_interval_ms", 500);  // 最小指令间隔(ms)

        std::string pump_chip = this->get_parameter("pump_chip").as_string();
        unsigned int pump_line_num = this->get_parameter("pump_line").as_int();
        std::string valve_chip = this->get_parameter("valve_chip").as_string();
        unsigned int valve_line_num = this->get_parameter("valve_line").as_int();

        if (!init_gpio(pump_chip, pump_line_num, valve_chip, valve_line_num)) {
            RCLCPP_FATAL(this->get_logger(), "GPIO初始化失败");
            return;
        }

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/quad/pump_cmd", 10,
            std::bind(&PumpNode::cmd_callback, this, std::placeholders::_1));

        // 状态反馈发布器
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/pump/state", 10);
        // 动作完成标志发布器
        action_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pump/action_done", 10);

        setPump(false);
        setValve(false);
        current_state_ = PumpState::IDLE;
        publishState(PumpState::IDLE);
        publishActionDone(true);  // 初始状态为完成

        // 初始化指令频率限制
        last_cmd_time_ = this->now();
        cmd_interval_ms_ = this->get_parameter("cmd_interval_ms").as_int();

        RCLCPP_INFO(this->get_logger(), "气泵控制器启动 - 气泵:%s/%u, 电磁阀:%s/%u, 指令间隔:%ldms",
                    pump_chip.c_str(), pump_line_num, valve_chip.c_str(), valve_line_num, cmd_interval_ms_);
    }

    ~PumpNode()
    {
        if (pump_line_) {
            gpiod_line_set_value(pump_line_, 0);
            gpiod_line_release(pump_line_);
        }
        if (valve_line_) {
            gpiod_line_set_value(valve_line_, 0);
            gpiod_line_release(valve_line_);
        }
        if (pump_chip_) {
            gpiod_chip_close(pump_chip_);
        }
        if (valve_chip_) {
            gpiod_chip_close(valve_chip_);
        }
    }

private:
    gpiod_chip *pump_chip_ = nullptr;
    gpiod_chip *valve_chip_ = nullptr;
    gpiod_line *pump_line_ = nullptr;
    gpiod_line *valve_line_ = nullptr;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr action_done_pub_;
    rclcpp::TimerBase::SharedPtr release_timer_;

    PumpState current_state_;
    rclcpp::Time last_cmd_time_;  // 上次接收指令时间
    int64_t cmd_interval_ms_;     // 最小指令间隔

    bool init_gpio(const std::string &pump_chip_name, unsigned int pump_gpio,
                   const std::string &valve_chip_name, unsigned int valve_gpio)
    {
        pump_chip_ = gpiod_chip_open_by_name(pump_chip_name.c_str());
        if (!pump_chip_) {
            RCLCPP_ERROR(this->get_logger(), "无法打开气泵GPIO芯片: %s", pump_chip_name.c_str());
            return false;
        }

        valve_chip_ = gpiod_chip_open_by_name(valve_chip_name.c_str());
        if (!valve_chip_) {
            RCLCPP_ERROR(this->get_logger(), "无法打开电磁阀GPIO芯片: %s", valve_chip_name.c_str());
            return false;
        }

        pump_line_ = gpiod_chip_get_line(pump_chip_, pump_gpio);
        if (!pump_line_) {
            RCLCPP_ERROR(this->get_logger(), "无法获取气泵GPIO线: %u", pump_gpio);
            return false;
        }

        valve_line_ = gpiod_chip_get_line(valve_chip_, valve_gpio);
        if (!valve_line_) {
            RCLCPP_ERROR(this->get_logger(), "无法获取电磁阀GPIO线: %u", valve_gpio);
            return false;
        }

        if (gpiod_line_request_output(pump_line_, "pump_control", 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置气泵GPIO为输出模式");
            return false;
        }

        if (gpiod_line_request_output(valve_line_, "valve_control", 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置电磁阀GPIO为输出模式");
            return false;
        }

        return true;
    }

    void setPump(bool on)
    {
        gpiod_line_set_value(pump_line_, on ? 1 : 0);
    }

    void setValve(bool on)
    {
        gpiod_line_set_value(valve_line_, on ? 1 : 0);
    }

    void publishState(PumpState state)
    {
        current_state_ = state;
        std_msgs::msg::String msg;
        switch (state) {
            case PumpState::IDLE:
                msg.data = "IDLE";
                break;
            case PumpState::HOLDING:
                msg.data = "HOLDING";
                break;
            case PumpState::RELEASING:
                msg.data = "RELEASING";
                break;
            case PumpState::STOPPED:
                msg.data = "STOPPED";
                break;
        }
        state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "状态: %s", msg.data.c_str());
    }

    void publishActionDone(bool done)
    {
        std_msgs::msg::Bool msg;
        msg.data = done;
        action_done_pub_->publish(msg);
    }

    void cmd_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        // 检查指令频率
        auto now = this->now();
        auto elapsed_ms = (now - last_cmd_time_).nanoseconds() / 1000000;
        if (elapsed_ms < cmd_interval_ms_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "指令频率过高，忽略该指令 (间隔:%ldms < %ldms)",
                                 elapsed_ms, cmd_interval_ms_);
            return;
        }
        last_cmd_time_ = now;

        PumpCommand cmd = static_cast<PumpCommand>(msg->data);

        switch (cmd) {
            case PumpCommand::GRAB:
                handleGrab();
                break;
            case PumpCommand::RELEASE:
                handleRelease();
                break;
            case PumpCommand::STOP:
                handleStop();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "未知命令: %d", msg->data);
                break;
        }
    }

    void handleGrab()
    {
        release_timer_.reset();
        publishActionDone(false);  // 动作开始
        RCLCPP_INFO(this->get_logger(), "执行GRAB");
        setValve(false);
        setPump(true);
        publishState(PumpState::HOLDING);

        // 延时后标记动作完成
        release_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),
            [this]() {
                publishActionDone(true);  // 吸合完成
                release_timer_.reset();
            });
    }

    void handleRelease()
    {
        release_timer_.reset();
        publishActionDone(false);  // 动作开始
        RCLCPP_INFO(this->get_logger(), "执行RELEASE");
        publishState(PumpState::RELEASING);

        setPump(false);

        release_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                setValve(true);
                release_timer_->cancel();

                release_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(200),
                    [this]() {
                        setValve(false);
                        publishState(PumpState::IDLE);
                        publishActionDone(true);  // 释放完成
                        release_timer_.reset();
                    });
            });
    }

    void handleStop()
    {
        release_timer_.reset();
        publishActionDone(false);  // 动作开始
        RCLCPP_INFO(this->get_logger(), "执行STOP");
        setPump(false);
        setValve(false);
        publishState(PumpState::STOPPED);
        publishActionDone(true);  // 停止完成
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PumpNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
