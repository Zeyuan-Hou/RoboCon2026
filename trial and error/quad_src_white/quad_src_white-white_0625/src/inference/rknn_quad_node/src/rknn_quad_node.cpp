#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "rknn_api.h"
#include "quad/msg/high_commands.hpp"
#include "FSM.h"

#include <vector>
#include <chrono>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <mutex>

// 定义维度
#define INPUT_SIZE 270
#define OUTPUT_SIZE 12

class RKNNQuadNode : public rclcpp::Node
{
public:
    RKNNQuadNode() : Node("rknn_quad_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("trot_model_path", "/home/cat/hitcrt_quad2026_ws/src/inference/rknn_quad_node/models/white_0509.rknn");
        this->declare_parameter<std::string>("creep_model_path", "/home/cat/hitcrt_quad2026_ws/src/inference/rknn_quad_node/models/creep.rknn");
        this->declare_parameter<std::string>("upstair_model_path", "/home/cat/hitcrt_quad2026_ws/src/inference/rknn_quad_node/models/white_0424_upstair.rknn");
        this->declare_parameter<std::string>("kneel_crawl_model_path", "/home/cat/hitcrt_quad2026_ws/src/inference/rknn_quad_node/models/white_kneel_crawl.rknn");

        std::string trot_model_path = this->get_parameter("trot_model_path").as_string();
        std::string creep_model_path = this->get_parameter("creep_model_path").as_string();
        std::string upstair_model_path = this->get_parameter("upstair_model_path").as_string();
        std::string kneel_crawl_model_path = this->get_parameter("kneel_crawl_model_path").as_string();

        std::string default_model_path = trot_model_path;
        // 初始化模型列表
        // 第一个模型（索引0）- 默认模型，RL_MOVE 时使用
        // 第二个模型（索引1）- RL_LOW 时使用
        // 第三个模型（索引2）- RL_HIGH 时使用
        model_list_ = {
            trot_model_path,  // 索引0: TROT
            creep_model_path,  // 索引1: CREEP 
            upstair_model_path,  // 索引2: UPSTAIR
            kneel_crawl_model_path,  // 索引3: KNEEL_CRAWL 过Gap
        };

        // 查找初始模型在列表中的位置
        current_model_index_ = 0;
        for (size_t i = 0; i < model_list_.size(); ++i) {
            if (model_list_[i] == default_model_path) {
                current_model_index_ = i;
                break;
            }
        }

        // 1. 初始化标准 RKNN
        if (!init_rknn_standard(default_model_path)) {
            rclcpp::shutdown();
            return;
        }

        // 2. 创建发布者
        output_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quad_output", 10);
        inference_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("inference_time", 10);

        // 3. 创建订阅者
        // 当收到 270 维的观测数据时，触发回调进行推理
        observation_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/quad/obs_data", 
            10, 
            std::bind(&RKNNQuadNode::observation_callback, this, std::placeholders::_1)
        );
        // // 订阅遥控器命令，用于切换模型
        // high_cmd_sub_ = this->create_subscription<quad::msg::HighCommands>(
        //     "/quad/high_command", 
        //     10, 
        //     std::bind(&RKNNQuadNode::high_command_callback, this, std::placeholders::_1)
        // );
        // 订阅策略切换命令，用于切换模型
        policy_select_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/quad/policy_select", 
            10, 
            std::bind(&RKNNQuadNode::policy_select_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "RKNN 节点已启动 (Topic Subscription Mode)");
        RCLCPP_INFO(this->get_logger(), "当前模型: %s (索引: %zu)", 
            model_list_[current_model_index_].c_str(), current_model_index_);
        RCLCPP_INFO(this->get_logger(), "模型切换规则: RL_MOVE->模型0, RL_LOW->模型1, RL_HIGH->模型2");
    }

    ~RKNNQuadNode()
    {
        std::lock_guard<std::mutex> lk(model_switch_mutex_);
        if (ctx_ > 0) rknn_destroy(ctx_);
    }

private:
    std::vector<std::string> model_list_;
    size_t current_model_index_ = 0;
    std::mutex model_switch_mutex_;
    int last_command_ = -1;  // 记录上一次的命令，避免重复切换

    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr inference_time_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr observation_sub_;
    // rclcpp::Subscription<quad::msg::HighCommands>::SharedPtr high_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr policy_select_sub_;


    // // 遥控器命令回调
    // void high_command_callback(const quad::msg::HighCommands::SharedPtr msg)
    // {
    //     int cmd = msg->command;
        
    //     // 如果命令没有变化，不进行切换
    //     if (cmd == last_command_) {
    //         return;
    //     }
        
    //     last_command_ = cmd;
    //     size_t target_index = current_model_index_;

    //     // 根据命令切换模型
    //     if (cmd == RL_MOVE) {
    //         target_index = 0;  // 第一个模型
    //     } else if (cmd == RL_LOW) {
    //         target_index = 1;  // 第二个模型
    //     } else if (cmd == RL_HIGH) {
    //         target_index = 2;  // 第三个模型
    //     } else {
    //         // 其他命令不切换模型
    //         return;
    //     }

    //     // 如果目标模型与当前模型不同，进行切换
    //     if (target_index != current_model_index_) {
    //         switch_model(target_index);
    //     }
    // }
    void policy_select_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        int policy_cmd = msg->data;
        size_t target_index = current_model_index_;  
        // 根据命令切换模型
        if (policy_cmd == TROT) {
            target_index = 0;  // 第一个模型
        } else if (policy_cmd == CREEP) {
            target_index = 1;  // 第二个模型
        } else if(policy_cmd == UPSTAIR){
            target_index = 2;  // 第三个模型
        }else if(policy_cmd == KNEEL_CRAWL_POLICY){
            target_index = 3;  // 第四个模型
        }else{
            // 其他命令不切换模型
            return;
        }  
        // 如果目标模型与当前模型不同，进行切换 
        if (target_index != current_model_index_) {
            switch_model(target_index);
        }             
    }

    // 切换模型
    void switch_model(size_t target_index)
    {
        if (target_index >= model_list_.size()) {
            RCLCPP_ERROR(this->get_logger(), "模型索引超出范围: %zu", target_index);
            return;
        }

        std::lock_guard<std::mutex> lk(model_switch_mutex_);
        
        size_t old_index = current_model_index_;
        std::string new_model_path = model_list_[target_index];
        
        RCLCPP_INFO(this->get_logger(), "正在切换模型: %s -> %s", 
                    model_list_[old_index].c_str(), new_model_path.c_str());

        // 销毁旧模型
        if (ctx_ > 0) {
            rknn_destroy(ctx_);
            ctx_ = 0;
        }

        // 加载新模型
        if (!init_rknn_standard(new_model_path)) {
            RCLCPP_ERROR(this->get_logger(), "模型切换失败，恢复到原模型");
            if (!init_rknn_standard(model_list_[old_index])) {
                RCLCPP_FATAL(this->get_logger(), "无法恢复原模型，节点将关闭");
                rclcpp::shutdown();
            }
            return;
        }

        // 更新当前模型索引
        current_model_index_ = target_index;
        
        RCLCPP_INFO(this->get_logger(), "模型切换成功！当前模型: %s (索引: %zu)", 
                    model_list_[current_model_index_].c_str(), current_model_index_);
    }

    // --- 初始化 (标准流程) ---
    bool init_rknn_standard(const std::string& model_path)
    {
        FILE* fp = fopen(model_path.c_str(), "rb");
        if (!fp) {
            RCLCPP_ERROR(this->get_logger(), "无法打开模型: %s", model_path.c_str());
            return false;
        }
        fseek(fp, 0, SEEK_END);
        size_t model_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        void* model_data = malloc(model_size);
        fread(model_data, 1, model_size, fp);
        fclose(fp);

        int ret = rknn_init(&ctx_, model_data, model_size, 0, nullptr);
        free(model_data);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_init 失败: %d", ret);
            return false;
        }

        ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret < 0) return false;

        RCLCPP_INFO(this->get_logger(), "Model Loaded. Input Num: %d, Output Num: %d", io_num_.n_input, io_num_.n_output);
        
        // rknn_tensor_attr out_attr;
        // memset(&out_attr, 0, sizeof(out_attr));
        // out_attr.index = 0;

        // rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));

        // RCLCPP_INFO(this->get_logger(),
        //     "Output attr: type=%d, qnt_type=%d, scale=%f",
        //     out_attr.type,
        //     out_attr.qnt_type,
        //     out_attr.scale);
        return true;

    }

    // --- 订阅回调 & 推理函数 ---
    void observation_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 0. 数据校验
        if (msg->data.size() != INPUT_SIZE) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "接收到的观测数据维度错误! Expected: %d, Got: %zu", INPUT_SIZE, msg->data.size());
            return;
        }

        auto start = std::chrono::high_resolution_clock::now();

        // 1. 设置输入
        // 这里的 msg->data 是 std::vector<float>，我们可以直接获取指针
        // 注意：const_cast 是因为 rknn 接口没有声明 const void*，但通常不会修改输入 buffer
        void* input_data_ptr = (void*)msg->data.data();

        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        
        inputs[0].index = 0;
        // 这里非常重要：我们告诉 RKNN，我们传进去的数据是 FLOAT32
        // RKNN 驱动会自动把它转换成模型内部需要的格式 (比如 FP16)
        inputs[0].type = RKNN_TENSOR_FLOAT32; 
        inputs[0].size = msg->data.size() * sizeof(float);
        inputs[0].fmt = RKNN_TENSOR_NCHW;
        inputs[0].buf = input_data_ptr; // 直接使用消息内存

        int ret = rknn_inputs_set(ctx_, io_num_.n_input, inputs);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_inputs_set fail: %d", ret);
            return;
        }

        // 2. 运行推理
        ret = rknn_run(ctx_, nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_run fail: %d", ret);
            return;
        }

        // 3. 获取输出
        rknn_output outputs[1];
        memset(outputs, 0, sizeof(outputs));
        // 这里非常重要：我们告诉 RKNN，我们需要 FLOAT 格式的输出
        // 驱动会自动把模型内部的 FP16/INT8 转回 FP32
        outputs[0].want_float = 1; 

        ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs, nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_outputs_get fail: %d", ret);
            return;
        }

        auto end = std::chrono::high_resolution_clock::now();

        // 4. 处理并发布数据
        float* output_ptr = (float*)outputs[0].buf;
        
        // 构造输出消息
        auto out_msg = std_msgs::msg::Float32MultiArray();
        out_msg.data.assign(output_ptr, output_ptr + OUTPUT_SIZE); // 只取前12个
        output_pub_->publish(out_msg);

        // 打印第一帧，检查是否回到了 -1.7 左右
        static bool first = true;
        if(first){
            RCLCPP_INFO(this->get_logger(), "第一帧输出:");
            for(int i=0;i<12;i++){
            RCLCPP_INFO(this->get_logger(), "%.4f, ", output_ptr[i]);
            }
            first = false;
            
        }

        // 5. 释放输出内存
        rknn_outputs_release(ctx_, io_num_.n_output, outputs);

        // 6. 发布推理耗时
        std::chrono::duration<double, std::milli> duration = end - start;
        auto time_msg = std_msgs::msg::Float64();
        time_msg.data = duration.count();
        inference_time_pub_->publish(time_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RKNNQuadNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
