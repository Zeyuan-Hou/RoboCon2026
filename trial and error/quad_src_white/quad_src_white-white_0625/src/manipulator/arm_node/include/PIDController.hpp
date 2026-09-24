#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cmath>
#include <algorithm>
namespace manipulator{
class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double output_max, double integral_max,
                  double error_deadband)
        : Kp_(kp), Ki_(ki), Kd_(kd),
          target_(0.0), feedback_(0.0),
          error_(0.0), prev_error_(0.0),
          integral_error_(0.0),
          derivative_error_(0.0), prev_derivative_error_(0.0),
          output_(0.0),
          output_max_(output_max), integral_max_(integral_max),
          error_deadband_(error_deadband),
          dt_(0.0) {}

    void reset() {
        error_ = 0.0;
        prev_error_ = 0.0;
        integral_error_ = 0.0;
        derivative_error_ = 0.0;
        prev_derivative_error_ = 0.0;
        output_ = 0.0;
    }

    void setParameters(double kp, double ki, double kd) {
        Kp_ = kp;
        Ki_ = ki;
        Kd_ = kd;
    }

    void setOutputLimits(double output_max) {
        output_max_ = output_max;
    }

    void setIntegralLimits(double integral_max) {
        integral_max_ = integral_max;
    }

    void setDeadBand(double error_deadband) {
        error_deadband_ = error_deadband;
    }

    double compute(double setpoint, double measured_value, double dt) {
        target_ = setpoint;
        feedback_ = measured_value;
        dt_ = dt;

        error_ = target_ - feedback_;

        if (std::fabs(error_) < error_deadband_) {
            error_ = 0.0;
        }

        double proportion_output = Kp_ * error_;

        integral_error_ += error_ * dt_;
        integral_error_ = std::clamp(integral_error_, -integral_max_, integral_max_);
        double integral_output = Ki_ * integral_error_;

        derivative_error_ = (error_ - prev_error_) / dt_;
        derivative_error_ = lowPassFilter(derivative_error_, prev_derivative_error_, 2 * dt_, dt_);
        double derivative_output = Kd_ * derivative_error_;

        output_ = proportion_output + integral_output + derivative_output;
        output_ = std::clamp(output_, -output_max_, output_max_);

        prev_error_ = error_;
        prev_derivative_error_ = derivative_error_;
        return output_;
    }

    /**
     * @brief 微分先行
     */
    double computeNoDerivativeKick(double setpoint, double measured_value, double dt) {
        target_ = setpoint;
        feedback_ = measured_value;
        dt_ = dt;

        error_ = target_ - feedback_;

        if (std::fabs(error_) < error_deadband_) {
            error_ = 0.0;
        }

        double proportion_output = Kp_ * error_;

        integral_error_ += error_ * dt_;
        integral_error_ = std::clamp(integral_error_, -integral_max_, integral_max_);
        double integral_output = Ki_ * integral_error_;

        double measured_derivative = (feedback_ - prev_feedback_) / dt_;
        measured_derivative = lowPassFilter(measured_derivative, prev_measured_derivative_, 2 * dt_, dt_);
        double derivative_output = Kd_ * measured_derivative;

        output_ = proportion_output + integral_output + derivative_output;
        output_ = std::clamp(output_, -output_max_, output_max_);

        prev_error_ = error_;
        prev_measured_derivative_ = measured_derivative;
        prev_feedback_ = feedback_;
        return output_;
    }

    /**
     * @brief 使用期望速度与实际速度误差作为微分项
     * @param setpoint 目标位置
     * @param measured_value 实际位置
     * @param desired_velocity 期望速度(TD输出的m_x2)
     * @param actual_velocity 实际速度(电机反馈速度)
     * @param dt 采样周期
     * @return PID输出
     */
    double computeWithVelocityError(double setpoint, double measured_value,
                                    double desired_velocity, double actual_velocity,
                                    double dt) {
        target_ = setpoint;
        feedback_ = measured_value;
        dt_ = dt;

        error_ = target_ - feedback_;

        if (std::fabs(error_) < error_deadband_) {
            error_ = 0.0;
        }

        double proportion_output = Kp_ * error_;

        integral_error_ += error_ * dt_;
        integral_error_ = std::clamp(integral_error_, -integral_max_, integral_max_);
        double integral_output = Ki_ * integral_error_;

        // 使用期望速度与实际速度的误差作为微分项
        double velocity_error = desired_velocity - actual_velocity;
        derivative_error_ = lowPassFilter(velocity_error, prev_derivative_error_, 2 * dt_, dt_);
        double derivative_output = Kd_ * derivative_error_;

        output_ = proportion_output + integral_output + derivative_output;
        output_ = std::clamp(output_, -output_max_, output_max_);

        prev_error_ = error_;
        prev_derivative_error_ = derivative_error_;
        return output_;
    }


private:
    double lowPassFilter(double input, double prev_output, double tau, double dt) const {
        double alpha = tau / (tau + dt);
        return alpha * prev_output + (1.0 - alpha) * input;
    }

    double Kp_, Ki_, Kd_;
    double target_, feedback_;
    double error_, prev_error_;
    double integral_error_;
    double derivative_error_, prev_derivative_error_;
    double measured_derivative_, prev_measured_derivative_;
    double prev_feedback_;
    double output_;
    double output_max_, integral_max_;
    double error_deadband_;
    double dt_;
};
}

#endif
