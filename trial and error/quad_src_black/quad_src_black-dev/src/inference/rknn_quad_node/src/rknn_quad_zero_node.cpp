#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rknn_api.h"

#include <vector>
#include <chrono>
#include <cstring>
#include <fstream>  // 用于文件读写
#include <iostream>
#include <cmath>
#include <cstdint>
#include <algorithm>

typedef uint16_t hfloat;

// --- 工具函数：FP32 <-> FP16 ---
static hfloat float32_to_float16(float v) {
    uint32_t x = *((uint32_t*)&v);
    uint16_t h = ((x >> 16) & 0x8000) | ((((x & 0x7f800000) - 0x38000000) >> 13) & 0x7c00) | ((x >> 13) & 0x03ff);
    return h;
}

static float float16_to_float32(hfloat v) {
    uint32_t t = ((v & 0x8000) << 16) | (((v & 0x7c00) + 0x1c000) << 13) | ((v & 0x03ff) << 13);
    return *((float*)&t);
}

class RKNNZeroCopyNode : public rclcpp::Node
{
public:
    RKNNZeroCopyNode() : Node("rknn_zerocopy_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("model_path", "/home/cat/dog_ws/src/model/model_6000.rknn");
        this->declare_parameter<std::string>("input_data_path", "/home/cat/dog_ws/rknn_input_270.txt");
        this->declare_parameter<double>("inference_rate", 125.0);
        this->declare_parameter<bool>("enable_logging", true); // 新增：是否保存TXT

        std::string model_path = this->get_parameter("model_path").as_string();
        std::string input_path = this->get_parameter("input_data_path").as_string();
        double inference_rate = this->get_parameter("inference_rate").as_double();
        enable_logging_ = this->get_parameter("enable_logging").as_bool();

        // 1. 加载数据
        if (!load_input_data(input_path)) {
            RCLCPP_ERROR(this->get_logger(), "加载输入数据失败");
            return;
        }

        // 2. 初始化 RKNN
        if (!init_rknn_zerocopy(model_path)) {
             RCLCPP_ERROR(this->get_logger(), "RKNN 初始化失败");
             return;
        }

        // 3. ROS 设定
        output_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quad_output", 10);
        time_pub_ = this->create_publisher<std_msgs::msg::Float64>("inference_time", 10);

        auto period = std::chrono::duration<double>(1.0 / inference_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&RKNNZeroCopyNode::inference_callback, this));

        // 如果开启记录，先清空旧文件
        if (enable_logging_) {
            std::ofstream outfile("rknn_output_log.txt", std::ios::out);
            outfile.close();
            RCLCPP_INFO(this->get_logger(), ">>> 日志已开启，结果将写入 rknn_output_log.txt <<<");
        }
        
        RCLCPP_INFO(this->get_logger(), ">>> 零拷贝节点启动成功 <<<");
    }

    ~RKNNZeroCopyNode() {
        if (ctx_ > 0) {
            if (in_mem_) rknn_destroy_mem(ctx_, in_mem_);
            if (out_mem_) rknn_destroy_mem(ctx_, out_mem_);
            rknn_destroy(ctx_);
        }
    }

private:
    rknn_context ctx_ = 0;
    std::vector<float> input_raw_data_;
    rknn_tensor_mem* in_mem_ = nullptr;
    rknn_tensor_mem* out_mem_ = nullptr;
    rknn_tensor_attr in_attr_;
    rknn_tensor_attr out_attr_;
    bool enable_logging_ = false;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string get_type_string(rknn_tensor_type type) {
        switch(type) {
            case RKNN_TENSOR_FLOAT32: return "FP32";
            case RKNN_TENSOR_FLOAT16: return "FP16";
            case RKNN_TENSOR_INT8:    return "INT8";
            case RKNN_TENSOR_UINT8:   return "UINT8";
            default: return "UNKNOWN";
        }
    }

    bool init_rknn_zerocopy(const std::string& model_path)
    {
        FILE* fp = fopen(model_path.c_str(), "rb");
        if (!fp) return false;
        fseek(fp, 0, SEEK_END);
        size_t size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        void* data = malloc(size);
        fread(data, 1, size, fp);
        fclose(fp);

        int ret = rknn_init(&ctx_, data, size, 0, nullptr);
        free(data);
        if (ret < 0) return false;

        in_attr_.index = 0;
        ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in_attr_, sizeof(in_attr_));
        out_attr_.index = 0;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr_, sizeof(out_attr_));

        RCLCPP_INFO(this->get_logger(), "Input: %s (Scale: %f, ZP: %d)", get_type_string(in_attr_.type).c_str(), in_attr_.scale, in_attr_.zp);
        RCLCPP_INFO(this->get_logger(), "Output: %s (Scale: %f, ZP: %d)", get_type_string(out_attr_.type).c_str(), out_attr_.scale, out_attr_.zp);

        in_mem_ = rknn_create_mem(ctx_, in_attr_.size_with_stride);
        out_mem_ = rknn_create_mem(ctx_, out_attr_.size_with_stride);
        
        rknn_set_io_mem(ctx_, in_mem_, &in_attr_);
        rknn_set_io_mem(ctx_, out_mem_, &out_attr_);

        return true;
    }

    bool load_input_data(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) return false;
        float val;
        input_raw_data_.clear();
        while (file >> val) input_raw_data_.push_back(val);
        if (input_raw_data_.size() != 270) {
            RCLCPP_WARN(this->get_logger(), "Data size %ld != 270, padding...", input_raw_data_.size());
            input_raw_data_.resize(270, 0.0f);
        }
        return true;
    }

    void inference_callback()
    {
        auto start = std::chrono::high_resolution_clock::now();

        // 1. 填充输入 (支持 FP32/FP16/INT8)
        void* dest_ptr = in_mem_->virt_addr;
        int num_elems = input_raw_data_.size();

        if (in_attr_.type == RKNN_TENSOR_INT8) {
            int8_t* dest_i8 = (int8_t*)dest_ptr;
            float scale = in_attr_.scale;
            int32_t zp = in_attr_.zp;
            for (int i = 0; i < num_elems; ++i) {
                int32_t v = (int32_t)(input_raw_data_[i] / scale + 0.5f) + zp;
                v = std::max(-128, std::min(127, v));
                dest_i8[i] = (int8_t)v;
            }
        } 
        else if (in_attr_.type == RKNN_TENSOR_FLOAT16) {
            uint16_t* dest_f16 = (uint16_t*)dest_ptr;
            for (int i = 0; i < num_elems; ++i) dest_f16[i] = float32_to_float16(input_raw_data_[i]);
        }
        else {
            memcpy(dest_ptr, input_raw_data_.data(), num_elems * sizeof(float));
        }

        rknn_mem_sync(ctx_, in_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);

        // 2. 推理
        if (rknn_run(ctx_, nullptr) < 0) return;

        rknn_mem_sync(ctx_, out_mem_, RKNN_MEMORY_SYNC_FROM_DEVICE);

        // 3. 读取输出
        std::vector<float> out_float_vec;
        void* src_ptr = out_mem_->virt_addr;
        int out_elems = 12;

        if (out_attr_.type == RKNN_TENSOR_INT8) {
            int8_t* src_i8 = (int8_t*)src_ptr;
            float scale = out_attr_.scale;
            int32_t zp = out_attr_.zp;
            for (int i = 0; i < out_elems; ++i) {
                out_float_vec.push_back(((int32_t)src_i8[i] - zp) * scale);
            }
        }
        else if (out_attr_.type == RKNN_TENSOR_FLOAT16) {
            uint16_t* src_f16 = (uint16_t*)src_ptr;
            for (int i = 0; i < out_elems; ++i) out_float_vec.push_back(float16_to_float32(src_f16[i]));
        }
        else {
            float* src_f32 = (float*)src_ptr;
            out_float_vec.assign(src_f32, src_f32 + out_elems);
        }

        // 4. [新增] 保存到 TXT 文件
        if (enable_logging_) {
            // 使用 append 模式打开
            std::ofstream outfile("rknn_output_log.txt", std::ios::app);
            if (outfile.is_open()) {
                for (size_t i = 0; i < out_float_vec.size(); ++i) {
                    outfile << out_float_vec[i] << (i == out_float_vec.size() - 1 ? "" : " ");
                }
                outfile << "";
                outfile.close();
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> diff = end - start;

        // 发布 ROS 话题
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = out_float_vec;
        output_pub_->publish(msg);
        
        std_msgs::msg::Float64 time_msg;
        time_msg.data = diff.count();
        time_pub_->publish(time_msg);

        // 控制台打印精简一点
        // printf("Loop %.3f ms | Out[0]: %.4f", diff.count(), out_float_vec[0]);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RKNNZeroCopyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
