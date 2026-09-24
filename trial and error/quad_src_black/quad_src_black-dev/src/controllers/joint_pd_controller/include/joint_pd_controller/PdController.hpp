#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "LowPassFilter.hpp"

/**
 * @brief PD控制器类
 */
class PdController {
   public:
    PdController(double kp=0.0, double kd=0.0, double output_max= 33.5)
        : Kp_(kp),
          //   Ki_(ki),
          Kd_(kd),
          target_(0.0),
          feedback_(0.0),
          error_(0.0),
          prev_error_(0.0),
          derivative_error_(0.0),
          prev_derivative_error_(0.0),
          output_(0.0),
          output_max_(output_max),
        //   error_deadband_(error_deadband),
          dt_(0.0) {}

    double calcPD(double sp, double fb, double dt) {
        target_ = sp;
        feedback_ = fb;
        dt_ = dt;

        error_ = target_ - feedback_;  // 计算误差

        // if (fabs(error_) < error_deadband_) {  // 设置死区
        //     error_ = 0.0;
        // }

        /* 比例环节 */
        double proportion_output = Kp_ * error_;  // 计算比例项输出

        // /* 积分环节 */
        // integral_error_ += error_ * dt_;  // 计算误差积分
        // integral_error_ = std::clamp(integral_error_, -integral_max_,
        //                              integral_max_);     // 积分限幅
        // double integral_output = Ki_ * integral_error_;  // 计算积分项输出

        /* 微分环节 */
        derivative_error_ = (error_ - prev_error_) / dt_;  // 计算误差微分
        derivative_error_ =
            lowPassFilter(derivative_error_, prev_derivative_error_, 0.001,
                          dt_);                              // 一阶低通滤波
        double derivative_output = Kd_ * derivative_error_;  // 计算微分项输出

        /* 计算输出 */
        output_ = proportion_output + derivative_output;           // 计算总输出
        output_ = std::clamp(output_, -output_max_, output_max_);  // 输出限幅

        prev_error_ = error_;                        // 更新上次误差
        prev_derivative_error_ = derivative_error_;  // 更新上次误差微分
        return output_;                              // 返回输出值
    }
    void setPD(double kp, double kd) {
        Kp_ = kp;
        Kd_ = kd;
    }
    void reset() {
        error_ = prev_error_ = 0.0;
        // integral_error_ = 0.0;
        derivative_error_ = prev_derivative_error_ = 0.0;
        output_ = 0.0;
    }

   protected:
    double Kp_, Kd_;             // PID参数
    double target_, feedback_;   // 目标值，反馈值
    double error_, prev_error_;  // 误差，上次误差
    // double integral_error_;                            // 误差积分
    double derivative_error_, prev_derivative_error_;  // 误差微分,上次误差微分
    double output_;                                    // 输出值
    double output_max_;                                // 输出限幅
    // double error_deadband_;                            // 误差死区
    double dt_;                                        // 控制周期

};



#endif  // PD_CONTROLLER_H
