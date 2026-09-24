#include "task_manager/task_executor_node.hpp"
#include "task_manager/lidar_pose_guard.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>

/**
 * @brief 构造任务执行器节点
 *
 * 输入：ROS 参数（load_parameters）
 * 输出：初始化成员与 ROS 接口
 * 处理：加载参数后 setup_ros_interfaces
 */
TaskExecutorNode::TaskExecutorNode()
    : Node("task_executor_node"),
      current_mode_(ExecutorMode::IDLE),
      x_(0.0), y_(0.0), yaw_(0.0), got_tf_(false)
{
    load_parameters();
    setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "TaskExecutorNode Initialized.");
}

/**
 * @brief 创建订阅、发布与控制定时器
 *
 * 输入：tf_topic_ 等成员
 * 输出：各 sub/pub 与 50Hz timer_
 * 处理：与原构造中接口注册一致
 */
void TaskExecutorNode::setup_ros_interfaces() {
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, 100, std::bind(&TaskExecutorNode::tf_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "current_state", 10, std::bind(&TaskExecutorNode::state_callback, this, std::placeholders::_1));

    qr_sub_ = this->create_subscription<quad::msg::QrResult>(
        "qr_detection_result", 10, std::bind(&TaskExecutorNode::qr_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    task_done_pub_ = this->create_publisher<std_msgs::msg::Int32>("task_done", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("visual_servoing/target", 10);
    error_debug_pub_ = this->create_publisher<geometry_msgs::msg::Point>("visual_servoing/debug_errors", 10);
    lidar_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>(lidar_pose_topic_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TaskExecutorNode::control_loop, this));
}

/**
 * @brief 加载各类任务所需的配置参数
 *
 * 输入：节点参数服务器
 * 输出：过桥/楼梯/匍匐/视觉/绕杆相关成员
 * 处理：declare 与 get 顺序与原实现一致
 */
void TaskExecutorNode::load_parameters() {
    this->declare_parameter("target_frame", "camera_init");
    this->declare_parameter("child_frame", "aft_mapped");
    this->declare_parameter("tf_topic", "/tf");
    this->declare_parameter("lidar_pose_topic", "lidar_pose_xyyaw");
    this->declare_parameter("straight_target_yaw", 0.0);
    this->declare_parameter("straight_target_dist", 4.0);
    this->declare_parameter("straight_max_speed", 0.5);
    this->declare_parameter("straight_kp_y", 2.0);
    this->declare_parameter("kp_xy", 0.5);
    this->declare_parameter("kp_yaw", 1.0);
    this->declare_parameter("max_vy", 0.5);
    this->declare_parameter("max_dyaw", 0.5);

    target_frame_ = this->get_parameter("target_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    lidar_pose_topic_ = this->get_parameter("lidar_pose_topic").as_string();
    straight_target_yaw_ = this->get_parameter("straight_target_yaw").as_double();
    straight_target_dist_ = this->get_parameter("straight_target_dist").as_double();
    straight_max_speed_ = this->get_parameter("straight_max_speed").as_double();
    straight_kp_y_ = this->get_parameter("straight_kp_y").as_double();
    kp_xy_ = this->get_parameter("kp_xy").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_dyaw_ = this->get_parameter("max_dyaw").as_double();

    this->declare_parameter("tasks.stair_duration", 4.0);
    this->declare_parameter("tasks.stair_target_yaw", 0.0);
    this->declare_parameter("tasks.stair_use_current_yaw", true);
    this->declare_parameter("tasks.stair_forward_speed", 0.25);
    this->declare_parameter("tasks.stair_kp_y", 2.0);
    this->declare_parameter("tasks.stair_kp_yaw", 1.0);
    this->declare_parameter("tasks.stair_max_vy", 0.25);
    this->declare_parameter("tasks.stair_max_dyaw", 0.4);
    this->declare_parameter("tasks.stair_max_distance", 4.0);

    stair_duration_ = this->get_parameter("tasks.stair_duration").as_double();
    stair_target_yaw_ = this->get_parameter("tasks.stair_target_yaw").as_double();
    stair_use_current_yaw_ = this->get_parameter("tasks.stair_use_current_yaw").as_bool();
    stair_forward_speed_ = this->get_parameter("tasks.stair_forward_speed").as_double();
    stair_kp_y_ = this->get_parameter("tasks.stair_kp_y").as_double();
    stair_kp_yaw_ = this->get_parameter("tasks.stair_kp_yaw").as_double();
    stair_max_vy_ = this->get_parameter("tasks.stair_max_vy").as_double();
    stair_max_dyaw_ = this->get_parameter("tasks.stair_max_dyaw").as_double();
    stair_max_distance_ = this->get_parameter("tasks.stair_max_distance").as_double();

    this->declare_parameter("tasks.crawl_target_yaw", 0.0);
    this->declare_parameter("tasks.crawl_use_current_yaw", true);
    this->declare_parameter("tasks.crawl_forward_speed", 0.25);
    this->declare_parameter("tasks.crawl_kp_y", 1.0);
    this->declare_parameter("tasks.crawl_kp_yaw", 1.0);
    this->declare_parameter("tasks.crawl_max_vy", 0.2);
    this->declare_parameter("tasks.crawl_max_dyaw", 0.3);
    this->declare_parameter("tasks.crawl_target_dist", 1.5);

    crawl_target_yaw_ = this->get_parameter("tasks.crawl_target_yaw").as_double();
    crawl_use_current_yaw_ = this->get_parameter("tasks.crawl_use_current_yaw").as_bool();
    crawl_forward_speed_ = this->get_parameter("tasks.crawl_forward_speed").as_double();
    crawl_kp_y_ = this->get_parameter("tasks.crawl_kp_y").as_double();
    crawl_kp_yaw_ = this->get_parameter("tasks.crawl_kp_yaw").as_double();
    crawl_max_vy_ = this->get_parameter("tasks.crawl_max_vy").as_double();
    crawl_max_dyaw_ = this->get_parameter("tasks.crawl_max_dyaw").as_double();
    crawl_target_dist_ = this->get_parameter("tasks.crawl_target_dist").as_double();

    this->declare_parameter("visual_servoing.default_offset", std::vector<double>{0.0, 700.0, 0.0});
    this->declare_parameter("visual_servoing.tolerance_x_m", 0.06);
    this->declare_parameter("visual_servoing.tolerance_y_m", 0.06);
    this->declare_parameter("visual_servoing.tolerance_yaw_rad", 0.08);
    this->declare_parameter("visual_servoing.kp_x", 2.5);
    this->declare_parameter("visual_servoing.kp_y", 2.5);
    this->declare_parameter("visual_servoing.kp_yaw", 0.75);
    this->declare_parameter("visual_servoing.max_vx", 0.4);
    this->declare_parameter("visual_servoing.max_vy", 0.4);
    this->declare_parameter("visual_servoing.max_wz", 0.5);
    this->declare_parameter("visual_servoing.yaw_error_sign", 1.0);

    auto def_vec = this->get_parameter("visual_servoing.default_offset").as_double_array();
    if (def_vec.size() == 3) {
        default_offset_ = {def_vec[0], def_vec[1], def_vec[2]};
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "visual_servoing.default_offset must contain exactly 3 values; falling back to [0, 700, 0].");
        default_offset_ = {0.0, 700.0, 0.0};
    }
    visual_tolerance_x_m_ = this->get_parameter("visual_servoing.tolerance_x_m").as_double();
    visual_tolerance_y_m_ = this->get_parameter("visual_servoing.tolerance_y_m").as_double();
    visual_tolerance_yaw_rad_ = this->get_parameter("visual_servoing.tolerance_yaw_rad").as_double();
    visual_kp_x_ = this->get_parameter("visual_servoing.kp_x").as_double();
    visual_kp_y_ = this->get_parameter("visual_servoing.kp_y").as_double();
    visual_kp_yaw_ = this->get_parameter("visual_servoing.kp_yaw").as_double();
    visual_max_vx_ = this->get_parameter("visual_servoing.max_vx").as_double();
    visual_max_vy_ = this->get_parameter("visual_servoing.max_vy").as_double();
    visual_max_wz_ = this->get_parameter("visual_servoing.max_wz").as_double();
    visual_yaw_error_sign_ = this->get_parameter("visual_servoing.yaw_error_sign").as_double();

    for (int id : {1, 2, 3, 4, 5, 6}) {
        std::string param_name = "visual_servoing.task_" + std::to_string(id) + "_offset";
        this->declare_parameter(param_name, std::vector<double>{});
        auto param = this->get_parameter(param_name);
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            continue;
        }
        auto val = param.as_double_array();
        if (val.size() == 3) {
            task_config_map_[id] = {val[0], val[1], val[2]};
        } else if (!val.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Ignoring %s because it must contain exactly 3 values: [x_mm, y_mm, yaw_deg]",
                        param_name.c_str());
        }
    }

    this->declare_parameter("tasks.pole_trajectory_file", "");
    this->declare_parameter("tasks.pole_kp_x", 0.8);
    this->declare_parameter("tasks.pole_kp_y", 0.8);
    this->declare_parameter("tasks.pole_kp_yaw", 1.0);
    this->declare_parameter("tasks.pole_kd_x", 0.0);
    this->declare_parameter("tasks.pole_kd_y", 0.0);
    this->declare_parameter("tasks.pole_kd_yaw", 0.0);
    this->declare_parameter("tasks.pole_lin_vel_creep_min", 0.0);
    this->declare_parameter("tasks.pole_lidar_offset_x", 0.0);
    this->declare_parameter("tasks.pole_lidar_offset_y", 0.0);
    this->declare_parameter("tasks.pole_max_vx", 0.35);
    this->declare_parameter("tasks.pole_max_vy", 0.35);
    this->declare_parameter("tasks.pole_max_dyaw", 0.5);

    pole_trajectory_file_ = this->get_parameter("tasks.pole_trajectory_file").as_string();
    pole_kp_x_ = this->get_parameter("tasks.pole_kp_x").as_double();
    pole_kp_y_ = this->get_parameter("tasks.pole_kp_y").as_double();
    pole_kp_yaw_ = this->get_parameter("tasks.pole_kp_yaw").as_double();
    pole_kd_x_ = this->get_parameter("tasks.pole_kd_x").as_double();
    pole_kd_y_ = this->get_parameter("tasks.pole_kd_y").as_double();
    pole_kd_yaw_ = this->get_parameter("tasks.pole_kd_yaw").as_double();
    pole_lin_vel_creep_min_ = this->get_parameter("tasks.pole_lin_vel_creep_min").as_double();
    pole_lidar_offset_x_ = this->get_parameter("tasks.pole_lidar_offset_x").as_double();
    pole_lidar_offset_y_ = this->get_parameter("tasks.pole_lidar_offset_y").as_double();
    pole_max_vx_ = this->get_parameter("tasks.pole_max_vx").as_double();
    pole_max_vy_ = this->get_parameter("tasks.pole_max_vy").as_double();
    pole_max_dyaw_ = this->get_parameter("tasks.pole_max_dyaw").as_double();
}

/**
 * @brief 根据上层状态字符串切换执行器模式
 *
 * 输入：state_str（与 task_state_machine current_state 一致）
 * 输出：current_mode_ 及各任务段标志位
 * 处理：与原 if-else 链相同，仅在模式变化时重置标志
 */
void TaskExecutorNode::apply_executor_mode_from_state_string(const std::string& state_str) {
    if (!completed_state_latch_.empty()) {
        if (state_str == completed_state_latch_) {
            return;
        }
        completed_state_latch_.clear();
    }

    if (state_str == "QR_RECOGNITION") {
        if (current_mode_ != ExecutorMode::VISUAL_SERVOING) {
            current_mode_ = ExecutorMode::VISUAL_SERVOING;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: VISUAL_SERVOING");
        }
    } else if (state_str == "TASK_CRAWL_MOVING") {
        if (current_mode_ != ExecutorMode::LIDAR_CRAWL) {
            current_mode_ = ExecutorMode::LIDAR_CRAWL;
            crawl_started_ = false;
            crawl_finished_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: LIDAR_CRAWL");
        }
    } else if (state_str == "TASK_BRIDGE_CROSS") {
        if (current_mode_ != ExecutorMode::BRIDGE_CROSS) {
            current_mode_ = ExecutorMode::BRIDGE_CROSS;
            straight_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: BRIDGE_CROSS");
        }
    } else if (state_str == "TASK_STAIR_MOVING") {
        if (current_mode_ != ExecutorMode::STAIR_UP_MOVING) {
            current_mode_ = ExecutorMode::STAIR_UP_MOVING;
            stair_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: STAIR_UP_MOVING");
        }
    } else if (state_str == "TASK_POLE_AROUND") {
        if (current_mode_ != ExecutorMode::POLE_AROUND) {
            current_mode_ = ExecutorMode::POLE_AROUND;
            pole_replay_started_ = false;
            pole_replay_done_ = false;
            pole_pd_initialized_ = false;
            pole_trajectory_loaded_ = load_pole_trajectory();
            RCLCPP_INFO(this->get_logger(), "Executor Mode: POLE_AROUND");
        }
    } else {
        if (current_mode_ != ExecutorMode::IDLE) {
            current_mode_ = ExecutorMode::IDLE;
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Executor Mode: IDLE (Released Control)");
        }
    }
}

/**
 * @brief 顶层状态回调入口
 *
 * 输入：current_state 话题字符串
 * 输出：委托 apply_executor_mode_from_state_string
 * 处理：解析 msg->data 后切换模式
 */
void TaskExecutorNode::state_callback(const std_msgs::msg::String::SharedPtr msg) {
    apply_executor_mode_from_state_string(msg->data);
}

/**
 * @brief 二维码检测结果回调
 *
 * 输入：QrResult 消息
 * 输出：latest_qr_data_、last_qr_time_、qr_data_received_
 * 处理：缓存最新视觉观测
 */
void TaskExecutorNode::qr_callback(const quad::msg::QrResult::SharedPtr msg) {
    latest_qr_data_ = *msg;
    last_qr_time_ = this->now();
    qr_data_received_ = true;
}

/**
 * @brief 从 TF 消息更新位姿（匹配 target/child 帧）
 *
 * 输入：TFMessage、target_frame_/child_frame_
 * 输出：x_/y_/yaw_、got_tf_、lidar_pose_topic_
 * 处理：遍历 transforms 取第一个匹配项，平移直接使用，四元数只转换 yaw 后发布
 */
void TaskExecutorNode::update_pose_from_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& transform : msg->transforms) {
        if (transform.header.frame_id == target_frame_ && transform.child_frame_id == child_frame_) {
            x_ = transform.transform.translation.x;
            y_ = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw_);
            got_tf_ = true;

            geometry_msgs::msg::Point pose_msg;
            pose_msg.x = x_;
            pose_msg.y = y_;
            pose_msg.z = yaw_;
            lidar_pose_pub_->publish(pose_msg);
            break;
        }
    }
}

/**
 * @brief TF 回调
 *
 * 输入：TF 消息
 * 输出：更新并发布通用雷达位姿；任务模式可复用 x_/y_/yaw_
 * 处理：只做平移读取和四元数到 yaw 的转换，不做坐标变换
 */
void TaskExecutorNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    update_pose_from_tf(msg);
}

/**
 * @brief 定时控制循环：按 ExecutorMode 分发任务算法
 *
 * 输入：current_mode_
 * 输出：各 execute_* 发布的 cmd_vel / task_done
 * 处理：switch 分发，IDLE 不动作
 */
void TaskExecutorNode::control_loop() {
    switch (current_mode_) {
        case ExecutorMode::VISUAL_SERVOING:
            execute_visual_servoing();
            break;
        case ExecutorMode::LIDAR_CRAWL:
            execute_lidar_crawl();
            break;
        case ExecutorMode::BRIDGE_CROSS:
            execute_bridge_cross();
            break;
        case ExecutorMode::STAIR_UP_MOVING:
            execute_stair_up();
            break;
        case ExecutorMode::POLE_AROUND:
            execute_pole_around();
            break;
        case ExecutorMode::IDLE:
        default:
            break;
    }
}

/**
 * @brief 判断 QR 数据是否超时
 *
 * 输入：last_qr_time_、DATA_TIMEOUT_THRESHOLD
 * 输出：是否应停车并停止伺服
 * 处理：与当前时间差超过阈值则为 stale
 */
bool TaskExecutorNode::is_qr_data_stale() {
    return (this->now() - last_qr_time_).seconds() > DATA_TIMEOUT_THRESHOLD;
}

/**
 * @brief 获取当前 task_id 对应的视觉目标偏置
 *
 * 输入：task_id、task_config_map_、default_offset_
 * 输出：TaskOffset
 * 处理：map 有则用 per-task，否则 default
 */
TaskOffset TaskExecutorNode::get_active_task_offset(int task_id) const {
    if (task_config_map_.count(task_id)) {
        return task_config_map_.at(task_id);
    }
    return default_offset_;
}

/**
 * @brief QR 视觉系误差转换到机体系
 *
 * 输入：config、latest_qr_data_、visual_yaw_error_sign_
 * 输出：VisualBodyErrors
 * 处理：mm/deg 差分后按 body.x=qr.y, body.y=-qr.x 变换
 */
VisualBodyErrors TaskExecutorNode::compute_visual_body_errors(const TaskOffset& config) const {
    const double err_qr_x_mm = latest_qr_data_.x - config.target_qr_x;
    const double err_qr_y_mm = latest_qr_data_.y - config.target_qr_y;
    const double err_qr_yaw_deg = latest_qr_data_.yaw - config.target_qr_yaw;
    VisualBodyErrors errors{};
    errors.err_x_m = err_qr_y_mm / 1000.0;
    errors.err_y_m = -err_qr_x_mm / 1000.0;
    errors.err_yaw_rad = visual_yaw_error_sign_ * err_qr_yaw_deg * (M_PI / 180.0);
    errors.err_qr_yaw_deg = err_qr_yaw_deg;
    return errors;
}

/**
 * @brief 发布视觉伺服调试误差
 *
 * 输入：机体系误差
 * 输出：visual_servoing/debug_errors
 * 处理：填入 Point.x/y/z 发布
 */
void TaskExecutorNode::publish_visual_debug_errors(const VisualBodyErrors& errors) const {
    geometry_msgs::msg::Point debug_msg;
    debug_msg.x = errors.err_x_m;
    debug_msg.y = errors.err_y_m;
    debug_msg.z = errors.err_yaw_rad;
    error_debug_pub_->publish(debug_msg);
}

/**
 * @brief 判断三轴视觉对准是否完成
 *
 * 输入：errors、visual_tolerance_* 
 * 输出：是否全部在容差内
 * 处理：分别比较 |err| 与 tolerance
 */
bool TaskExecutorNode::is_visual_aligned(const VisualBodyErrors& errors) const {
    return std::abs(errors.err_x_m) < visual_tolerance_x_m_ &&
           std::abs(errors.err_y_m) < visual_tolerance_y_m_ &&
           std::abs(errors.err_yaw_rad) < visual_tolerance_yaw_rad_;
}

/**
 * @brief 发布视觉 P 控制速度（分轴到位则该轴速度为 0）
 *
 * 输入：errors、task_id
 * 输出：cmd_vel
 * 处理：clamp 后 publish，并 throttle 日志
 */
void TaskExecutorNode::publish_visual_velocity_cmd(const VisualBodyErrors& errors, int task_id) {
    const bool x_aligned = std::abs(errors.err_x_m) < visual_tolerance_x_m_;
    const bool y_aligned = std::abs(errors.err_y_m) < visual_tolerance_y_m_;
    const bool yaw_aligned = std::abs(errors.err_yaw_rad) < visual_tolerance_yaw_rad_;

    geometry_msgs::msg::Twist vel;
    vel.linear.x = x_aligned ? 0.0 : std::clamp(visual_kp_x_ * errors.err_x_m, -visual_max_vx_, visual_max_vx_);
    vel.linear.y = y_aligned ? 0.0 : std::clamp(visual_kp_y_ * errors.err_y_m, -visual_max_vy_, visual_max_vy_);
    vel.angular.z = yaw_aligned ? 0.0 : std::clamp(visual_kp_yaw_ * errors.err_yaw_rad, -visual_max_wz_, visual_max_wz_);
    cmd_vel_pub_->publish(vel);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "VISUAL_SERVO: task=%d err=(%.3f, %.3f, %.3f rad raw_yaw=%.2f deg sign=%.1f) cmd=(%.2f, %.2f, %.2f)",
        task_id, errors.err_x_m, errors.err_y_m, errors.err_yaw_rad, errors.err_qr_yaw_deg, visual_yaw_error_sign_,
        vel.linear.x, vel.linear.y, vel.angular.z);
}

/**
 * @brief 视觉闭环伺服对准主流程
 *
 * 输入：qr 数据、容差与增益参数
 * 输出：cmd_vel 或 task_done(task_id)
 * 处理：超时停车→算误差→对齐则 finish_task→否则发速度
 */
void TaskExecutorNode::execute_visual_servoing() {
    if (!qr_data_received_) {
        return;
    }

    if (is_qr_data_stale()) {
        stop_robot();
        return;
    }

    const int task_id = latest_qr_data_.task_type;
    const TaskOffset config = get_active_task_offset(task_id);
    const VisualBodyErrors errors = compute_visual_body_errors(config);

    publish_visual_debug_errors(errors);

    if (is_visual_aligned(errors)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "Visual Servoing Aligned. Task ID: %d", task_id);
        finish_task(task_id);
        return;
    }

    publish_visual_velocity_cmd(errors, task_id);
}

/**
 * @brief 记录匍匐段起始状态
 *
 * 输入：x_/y_/yaw_
 * 输出：crawl_start_*、crawl_active_target_yaw_、crawl_started_
 * 处理：首次进入时记录
 */
void TaskExecutorNode::ensure_crawl_segment_started() {
    if (crawl_started_) {
        return;
    }
    crawl_start_x_ = x_;
    crawl_start_y_ = y_;
    crawl_active_target_yaw_ = crawl_use_current_yaw_ ? yaw_ : crawl_target_yaw_;
    crawl_started_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Lidar Crawl Started at (%.2f, %.2f), yaw current %.2f, target %.2f%s",
                x_, y_, yaw_, crawl_active_target_yaw_,
                crawl_use_current_yaw_ ? " (current yaw)" : "");
}

/**
 * @brief 计算匍匐前进与横向误差
 *
 * 输入：位姿、crawl_active_target_yaw_
 * 输出：CrawlProgress
 * 处理：世界系位移投影到目标直线系
 */
CrawlProgress TaskExecutorNode::compute_crawl_progress() const {
    CrawlProgress progress{};
    const double dx_world = x_ - crawl_start_x_;
    const double dy_world = y_ - crawl_start_y_;
    const double cos_target = std::cos(crawl_active_target_yaw_);
    const double sin_target = std::sin(crawl_active_target_yaw_);
    progress.forward_progress = cos_target * dy_world + sin_target * dx_world;
    progress.lateral_error = -cos_target * dx_world + sin_target * dy_world;
    progress.err_yaw = normalize_angle(crawl_active_target_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断匍匐段是否应结束
 *
 * 输入：progress、crawl_target_dist_
 * 输出：是否结束
 * 处理：按前进距离完成，yaw 偏差过大时不完成
 */
bool TaskExecutorNode::is_crawl_finished(const CrawlProgress& progress) const {
    if (std::abs(progress.err_yaw) > 0.25) {
        return false;
    }
    return crawl_target_dist_ > 0.0 && progress.forward_progress >= crawl_target_dist_;
}

/**
 * @brief 计算匍匐雷达闭环 cmd_vel
 *
 * 输入：progress
 * 输出：Twist
 * 处理：|err_yaw|>0.25 只转；否则定速前进+横向纠偏
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_crawl_twist(const CrawlProgress& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.25) {
        cmd.angular.z = std::clamp(crawl_kp_yaw_ * progress.err_yaw, -crawl_max_dyaw_, crawl_max_dyaw_);
    } else {
        cmd.linear.x = crawl_forward_speed_;
        cmd.linear.y = std::clamp(crawl_kp_y_ * (-progress.lateral_error), -crawl_max_vy_, crawl_max_vy_);
        cmd.angular.z = std::clamp(crawl_kp_yaw_ * progress.err_yaw, -crawl_max_dyaw_, crawl_max_dyaw_);
    }
    return cmd;
}

/**
 * @brief 匍匐雷达闭环直线段
 *
 * 输入：TF、匍匐闭环参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算进度→结束或发速
 */
void TaskExecutorNode::execute_lidar_crawl() {
    if (crawl_finished_) {
        return;
    }
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "LidarCrawl: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("LidarCrawl")) {
        return;
    }

    ensure_crawl_segment_started();
    const CrawlProgress progress = compute_crawl_progress();

    if (is_crawl_finished(progress)) {
        stop_robot();
        crawl_finished_ = true;
        RCLCPP_INFO(this->get_logger(), "Lidar Crawl Finished. progress=%.2f/%.2f",
                    progress.forward_progress, crawl_target_dist_);
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_crawl_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "LIDAR_CRAWL: progress=%.2f/%.2f, lateral=%.2f, yaw=%.2f, vx=%.2f, vy=%.2f, wz=%.2f",
        progress.forward_progress, crawl_target_dist_, progress.lateral_error, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 记录过桥起始位姿（首次进入时）
 *
 * 输入：x_/y_/yaw_
 * 输出：straight_start_*、straight_started_
 * 处理：仅 straight_started_ 为 false 时记录
 */
void TaskExecutorNode::ensure_bridge_segment_started() {
    if (straight_started_) {
        return;
    }
    straight_start_x_ = x_;
    straight_start_y_ = y_;
    straight_start_yaw_ = yaw_;
    straight_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Bridge Cross Started at (%.2f, %.2f), locked yaw=%.2f", x_, y_, yaw_);
}

/**
 * @brief 计算过桥理想直线系进度与剩余距离
 *
 * 输入：位姿与锁存起始 yaw、straight_target_dist_
 * 输出：BridgeProgressErrors
 * 处理：map 位移投影到理想雷达系
 */
BridgeProgressErrors TaskExecutorNode::compute_bridge_progress_errors() const {
    const double dx_map = x_ - straight_start_x_;
    const double dy_map = y_ - straight_start_y_;
    const double cos_ideal = std::cos(straight_start_yaw_);
    const double sin_ideal = std::sin(straight_start_yaw_);

    BridgeProgressErrors progress{};
    progress.dx_ideal_lidar = cos_ideal * dy_map + sin_ideal * dx_map;
    progress.dy_ideal_lidar = -sin_ideal * dy_map + cos_ideal * dx_map;
    progress.remain_dist = straight_target_dist_ - progress.dx_ideal_lidar;
    progress.err_yaw = normalize_angle(straight_start_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断过桥是否完成
 *
 * 输入：remain_dist、err_yaw
 * 输出：是否完成
 * 处理：remain_dist<0.05 且 |err_yaw|<0.1
 */
bool TaskExecutorNode::is_bridge_finished(const BridgeProgressErrors& progress) const {
    return progress.remain_dist < 0.05 && std::abs(progress.err_yaw) < 0.1;
}

/**
 * @brief 根据过桥误差计算 cmd_vel
 *
 * 输入：progress、PID/限速参数
 * 输出：Twist（先对航向再对位置）
 * 处理：|err_yaw|>0.2 只转；否则 vx/vy/wz
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_bridge_twist(const BridgeProgressErrors& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.2) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * progress.err_yaw, max_dyaw_));
    } else {
        cmd.linear.x = std::max(-straight_max_speed_, std::min(kp_xy_ * progress.remain_dist, straight_max_speed_));
        cmd.linear.y = std::max(-max_vy_, std::min(straight_kp_y_ * (-progress.dy_ideal_lidar), max_vy_));
        cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * progress.err_yaw, max_dyaw_));
    }
    return cmd;
}

/**
 * @brief 雷达辅助直线过桥
 *
 * 输入：TF、直线目标参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算误差→完成或发速
 */
void TaskExecutorNode::execute_bridge_cross() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "BridgeCross: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("BridgeCross")) {
        return;
    }

    ensure_bridge_segment_started();
    const BridgeProgressErrors progress = compute_bridge_progress_errors();

    if (is_bridge_finished(progress)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "Bridge Cross Finished.");
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_bridge_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "BRIDGE CROSS: rem=%.2f, cross=%.2f, err_yaw=%.2f, vx=%.2f, vy=%.2f, wz=%.2f",
        progress.remain_dist, progress.dy_ideal_lidar, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 记录上楼梯段起始状态
 *
 * 输入：x_/y_
 * 输出：stair_start_*、stair_started_、stair_start_time_
 * 处理：首次进入时记录
 */
void TaskExecutorNode::ensure_stair_segment_started() {
    if (stair_started_) {
        return;
    }
    stair_start_time_ = this->now();
    stair_start_x_ = x_;
    stair_start_y_ = y_;
    stair_active_target_yaw_ = stair_use_current_yaw_ ? yaw_ : stair_target_yaw_;
    stair_started_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Stair Up Started at (%.2f, %.2f), yaw current %.2f, target %.2f%s",
                x_, y_, yaw_, stair_active_target_yaw_,
                stair_use_current_yaw_ ? " (current yaw)" : "");
}

/**
 * @brief 计算上楼梯前进与横向误差
 *
 * 输入：位姿、stair_active_target_yaw_
 * 输出：StairProgress
 * 处理：世界系位移投影到目标直线系
 */
StairProgress TaskExecutorNode::compute_stair_progress() const {
    StairProgress progress{};
    progress.elapsed = (this->now() - stair_start_time_).seconds();
    const double dx_world = x_ - stair_start_x_;
    const double dy_world = y_ - stair_start_y_;
    const double cos_target = std::cos(stair_active_target_yaw_);
    const double sin_target = std::sin(stair_active_target_yaw_);
    progress.forward_progress = cos_target * dy_world + sin_target * dx_world;
    progress.lateral_error = -cos_target * dx_world + sin_target * dy_world;
    progress.err_yaw = normalize_angle(stair_active_target_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断上楼梯段是否应结束
 *
 * 输入：progress、stair_max_distance_
 * 输出：是否结束
 * 处理：只按前进距离完成；yaw 未对齐时不完成，避免原地转误判成功
 */
bool TaskExecutorNode::is_stair_finished(const StairProgress& progress) const {
    if (std::abs(progress.err_yaw) > 0.25) {
        return false;
    }
    return stair_max_distance_ > 0.0 && progress.forward_progress >= stair_max_distance_;
}

/**
 * @brief 计算上楼梯 cmd_vel
 *
 * 输入：progress
 * 输出：Twist
 * 处理：|err_yaw|>0.25 只转；否则定速前进+横向纠偏
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_stair_twist(const StairProgress& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.25) {
        cmd.angular.z = std::clamp(stair_kp_yaw_ * progress.err_yaw, -stair_max_dyaw_, stair_max_dyaw_);
    } else {
        cmd.linear.x = stair_forward_speed_;
        cmd.linear.y = std::clamp(stair_kp_y_ * (-progress.lateral_error), -stair_max_vy_, stair_max_vy_);
        cmd.angular.z = std::clamp(stair_kp_yaw_ * progress.err_yaw, -stair_max_dyaw_, stair_max_dyaw_);
    }
    return cmd;
}

/**
 * @brief 雷达辅助上楼梯闭环
 *
 * 输入：TF、楼梯参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算进度→结束或发速
 */
void TaskExecutorNode::execute_stair_up() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "StairUp: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("StairUp")) {
        return;
    }

    ensure_stair_segment_started();
    const StairProgress progress = compute_stair_progress();

    if (is_stair_finished(progress)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "Stair Up Finished. progress=%.2f/%.2f, elapsed=%.2f",
                    progress.forward_progress, stair_max_distance_, progress.elapsed);
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_stair_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "STAIR_UP: t=%.2f, progress=%.2f/%.2f, lateral=%.2f, yaw=%.2f, vx=%.2f, vy=%.2f, wz=%.2f",
        progress.elapsed, progress.forward_progress, stair_max_distance_, progress.lateral_error, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 启动绕杆轨迹回放计时
 *
 * 输入：pole_trajectory_
 * 输出：pole_replay_t0_、pole_replay_started_ 等
 * 处理：首次进入时重置 PD 并打日志
 */
void TaskExecutorNode::ensure_pole_replay_started() {
    if (pole_replay_started_) {
        return;
    }
    pole_replay_t0_ = this->now();
    pole_replay_started_ = true;
    pole_replay_done_ = false;
    pole_pd_initialized_ = false;
    RCLCPP_INFO(this->get_logger(),
                "PoleAround: replay started, duration %.3f s, %zu points.",
                pole_trajectory_.back().t, pole_trajectory_.size());
}

/**
 * @brief 计算绕杆参考点与机体的 map→body 误差
 *
 * 输入：参考位姿 ref、当前 x_/y_/yaw_、pole_lidar_offset_*
 * 输出：PoleBodyErrors
 * 处理：与 lidar_nav_auto_parku 一致的坐标变换
 */
PoleBodyErrors TaskExecutorNode::compute_pole_body_errors(double ref_x, double ref_y, double ref_yaw) const {
    const double err_x_map = ref_x - x_;
    const double err_y_map = ref_y - y_;
    const double err_yaw = normalize_angle(ref_yaw - yaw_);
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double err_x_lidar = cos_yaw * err_y_map - sin_yaw * err_x_map;
    const double err_y_lidar = -sin_yaw * err_y_map - cos_yaw * err_x_map;

    PoleBodyErrors errors{};
    errors.err_x_body = err_x_lidar + pole_lidar_offset_x_;
    errors.err_y_body = err_y_lidar - pole_lidar_offset_y_;
    errors.err_yaw = err_yaw;
    return errors;
}

/**
 * @brief 绕杆 PD 速度控制（含 creep 最小线速度）
 *
 * 输入：PoleBodyErrors
 * 输出：Twist；更新 pole PD 历史
 * 处理：微分限幅 dt，creep 后 clamp 发布
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_pole_pd_twist(const PoleBodyErrors& errors) {
    const auto ctrl_now = this->now();
    if (!pole_pd_initialized_) {
        pole_prev_err_x_body_ = errors.err_x_body;
        pole_prev_err_y_body_ = errors.err_y_body;
        pole_prev_err_yaw_ = errors.err_yaw;
        pole_prev_ctrl_time_ = ctrl_now;
        pole_pd_initialized_ = true;
    }
    double dt = (ctrl_now - pole_prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    const double dex_dt = (errors.err_x_body - pole_prev_err_x_body_) / dt;
    const double dey_dt = (errors.err_y_body - pole_prev_err_y_body_) / dt;
    const double deyaw_dt = normalize_angle(errors.err_yaw - pole_prev_err_yaw_) / dt;

    pole_prev_err_x_body_ = errors.err_x_body;
    pole_prev_err_y_body_ = errors.err_y_body;
    pole_prev_err_yaw_ = errors.err_yaw;
    pole_prev_ctrl_time_ = ctrl_now;

    double vx = pole_kp_x_ * errors.err_x_body + pole_kd_x_ * dex_dt;
    double vy = pole_kp_y_ * errors.err_y_body + pole_kd_y_ * dey_dt;
    if (pole_lin_vel_creep_min_ > 0.0) {
        constexpr double eps = 5e-3;
        if (std::abs(errors.err_x_body) > eps && std::abs(vx) < pole_lin_vel_creep_min_) {
            vx = std::copysign(pole_lin_vel_creep_min_, errors.err_x_body);
        }
        if (std::abs(errors.err_y_body) > eps && std::abs(vy) < pole_lin_vel_creep_min_) {
            vy = std::copysign(pole_lin_vel_creep_min_, errors.err_y_body);
        }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx, -pole_max_vx_, pole_max_vx_);
    cmd.linear.y = std::clamp(vy, -pole_max_vy_, pole_max_vy_);
    cmd.angular.z = std::clamp(
        pole_kp_yaw_ * errors.err_yaw + pole_kd_yaw_ * deyaw_dt,
        -pole_max_dyaw_, pole_max_dyaw_);
    return cmd;
}

/**
 * @brief 判断绕杆时间轴回放是否结束
 *
 * 输入：elapsed、t_end
 * 输出：是否应结束；结束时置 pole_replay_done_ 并 finish_task
 * 处理：elapsed>=t_end 时停车、打日志、finish_task(-1)
 */
bool TaskExecutorNode::check_pole_replay_finished(double elapsed, double t_end) {
    if (elapsed < t_end) {
        return false;
    }
    pole_replay_done_ = true;
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Pole Around Finished. replay duration %.3f s.", t_end);
    finish_task(-1);
    return true;
}

/**
 * @brief 绕杆轨迹时间轴回放与闭环纠偏
 *
 * 输入：pole_trajectory_、TF、PD 参数
 * 输出：cmd_vel、target 调试、task_done(-1)
 * 处理：采样参考→算 body 误差→PD→检查时间结束
 */
void TaskExecutorNode::execute_pole_around() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PoleAround: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("PoleAround")) {
        return;
    }
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        stop_robot();
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "PoleAround: no valid trajectory loaded.");
        return;
    }

    ensure_pole_replay_started();

    if (pole_replay_done_) {
        stop_robot();
        finish_task(-1);
        return;
    }

    const double elapsed = (this->now() - pole_replay_t0_).seconds();
    const double t_end = pole_trajectory_.back().t;

    double ref_x = 0.0;
    double ref_y = 0.0;
    double ref_yaw = 0.0;
    if (!sample_pole_reference(elapsed, ref_x, ref_y, ref_yaw)) {
        return;
    }

    geometry_msgs::msg::Point target_msg;
    target_msg.x = ref_x;
    target_msg.y = ref_y;
    target_msg.z = ref_yaw;
    target_pub_->publish(target_msg);

    const PoleBodyErrors errors = compute_pole_body_errors(ref_x, ref_y, ref_yaw);
    const geometry_msgs::msg::Twist cmd = compute_pole_pd_twist(errors);
    cmd_vel_pub_->publish(cmd);

    if (check_pole_replay_finished(elapsed, t_end)) {
        return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "POLE_REPLAY: t=%.2f/%.2f pos(%.2f,%.2f,%.2f) ref(%.2f,%.2f,%.2f) err_body(%.2f,%.2f,%.2f) cmd(%.2f,%.2f,%.2f)",
        elapsed, t_end, x_, y_, yaw_, ref_x, ref_y, ref_yaw,
        errors.err_x_body, errors.err_y_body, errors.err_yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 雷达位姿越界时停车并告警
 *
 * 输入：context（日志前缀）、x_/y_
 * 输出：true 表示已越界并已 stop_robot
 * 处理：x∉[-5,5] 或 y∉[0,10] 时 throttle 提示「雷达飘了」
 */
bool TaskExecutorNode::stop_on_lidar_drift(const char* context) {
    if (!task_manager::lidar_pose_guard::is_out_of_bounds(x_, y_)) {
        return false;
    }
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "%s: 雷达飘了! pose(%.2f, %.2f) out of bounds [x:%.0f,%.0f] [y:%.0f,%.0f]",
        context, x_, y_,
        task_manager::lidar_pose_guard::kMinX, task_manager::lidar_pose_guard::kMaxX,
        task_manager::lidar_pose_guard::kMinY, task_manager::lidar_pose_guard::kMaxY);
    stop_robot();
    return true;
}

std::string TaskExecutorNode::executor_mode_state_name() const {
    switch (current_mode_) {
        case ExecutorMode::VISUAL_SERVOING:
            return "QR_RECOGNITION";
        case ExecutorMode::LIDAR_CRAWL:
            return "TASK_CRAWL_MOVING";
        case ExecutorMode::BRIDGE_CROSS:
            return "TASK_BRIDGE_CROSS";
        case ExecutorMode::STAIR_UP_MOVING:
            return "TASK_STAIR_MOVING";
        case ExecutorMode::POLE_AROUND:
            return "TASK_POLE_AROUND";
        case ExecutorMode::IDLE:
        default:
            return "";
    }
}

/**
 * @brief 规范化角度到 [-pi, pi]
 *
 * 输入：angle（弧度）
 * 输出：归一化角度
 * 处理：循环加减 2*pi
 */
double TaskExecutorNode::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 发布零速度停车
 *
 * 输入：cmd_vel_pub_
 * 输出：零 Twist
 * 处理：publish 默认构造 Twist
 */
void TaskExecutorNode::stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}

/**
 * @brief 发布任务完成并进入 IDLE 模式
 *
 * 输入：task_id（QR 为正 ID，常规完成为 -1）
 * 输出：task_done 话题；current_mode_=IDLE
 * 处理：publish Int32 后切换执行器模式
 */
void TaskExecutorNode::finish_task(int task_id) {
    completed_state_latch_ = executor_mode_state_name();
    std_msgs::msg::Int32 done_msg;
    done_msg.data = task_id;
    task_done_pub_->publish(done_msg);
    current_mode_ = ExecutorMode::IDLE;
}

/**
 * @brief 从文本文件加载绕杆录制轨迹
 *
 * 输入：pole_trajectory_file_
 * 输出：pole_trajectory_；返回值是否成功
 * 处理：解析 time x y yaw，校验单调时间并归零 t0
 */
bool TaskExecutorNode::load_pole_trajectory() {
    pole_trajectory_.clear();

    if (pole_trajectory_file_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Pole trajectory file is empty.");
        return false;
    }

    std::ifstream file(pole_trajectory_file_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open pole trajectory file: %s",
                     pole_trajectory_file_.c_str());
        return false;
    }

    std::string line;
    std::size_t line_no = 0;
    while (std::getline(file, line)) {
        line_no++;
        const auto first_non_space = line.find_first_not_of(" \t\r\n");
        if (first_non_space == std::string::npos || line[first_non_space] == '#') {
            continue;
        }

        std::istringstream iss(line);
        TrajectoryPoint point{};
        if (!(iss >> point.t >> point.x >> point.y >> point.yaw)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid pole trajectory format at line %zu: %s. Expected: time_sec x y yaw",
                         line_no, line.c_str());
            pole_trajectory_.clear();
            return false;
        }
        pole_trajectory_.push_back(point);
    }

    if (pole_trajectory_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Pole trajectory file needs at least 2 valid points: %s",
                     pole_trajectory_file_.c_str());
        return false;
    }
    for (std::size_t i = 1; i < pole_trajectory_.size(); ++i) {
        if (pole_trajectory_[i].t <= pole_trajectory_[i - 1].t) {
            RCLCPP_ERROR(this->get_logger(),
                         "Pole trajectory time must be strictly increasing at index %zu", i);
            pole_trajectory_.clear();
            return false;
        }
    }

    const double t0 = pole_trajectory_.front().t;
    for (auto& point : pole_trajectory_) {
        point.t -= t0;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu pole replay points from %s, duration %.3f s",
                pole_trajectory_.size(), pole_trajectory_file_.c_str(), pole_trajectory_.back().t);
    return true;
}

/**
 * @brief 按回放时间插值参考位姿
 *
 * 输入：elapsed、pole_trajectory_
 * 输出：x/y/yaw 参考；返回值是否有效
 * 处理：首尾夹持或段内线性插值（yaw 经 normalize）
 */
bool TaskExecutorNode::sample_pole_reference(double elapsed, double& x, double& y, double& yaw) const {
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        return false;
    }

    if (elapsed <= pole_trajectory_.front().t) {
        x = pole_trajectory_.front().x;
        y = pole_trajectory_.front().y;
        yaw = pole_trajectory_.front().yaw;
        return true;
    }

    if (elapsed >= pole_trajectory_.back().t) {
        x = pole_trajectory_.back().x;
        y = pole_trajectory_.back().y;
        yaw = pole_trajectory_.back().yaw;
        return true;
    }

    for (std::size_t i = 0; i + 1 < pole_trajectory_.size(); ++i) {
        const auto& a = pole_trajectory_[i];
        const auto& b = pole_trajectory_[i + 1];
        if (elapsed >= a.t && elapsed <= b.t) {
            const double dt_seg = b.t - a.t;
            const double alpha = (dt_seg > 1e-9) ? (elapsed - a.t) / dt_seg : 0.0;
            x = a.x + alpha * (b.x - a.x);
            y = a.y + alpha * (b.y - a.y);
            const double dyaw = normalize_angle(b.yaw - a.yaw);
            yaw = normalize_angle(a.yaw + alpha * dyaw);
            return true;
        }
    }

    x = pole_trajectory_.back().x;
    y = pole_trajectory_.back().y;
    yaw = pole_trajectory_.back().yaw;
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
