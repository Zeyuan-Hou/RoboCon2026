/**
 * @file TD.hpp
 * @brief 跟踪微分器 (Tracking Differentiator)
 *
 * 基于韩京清研究员的ADRC理论中的最速控制综合函数fhan
 * 用于平滑跟踪目标信号并提取微分信号
 */

#ifndef TD_HPP_
#define TD_HPP_

#include <cmath>

/**
 * @brief 跟踪微分器类
 *
 * TD通过非线性反馈实现快速无超调的跟踪，同时提取近似微分。
 * 相比传统微分器，TD具有良好的噪声抑制能力。
 *
 * 参数说明：
 * - r: 速度因子，决定跟踪速度，r越大跟踪越快但可能产生超调
 * - h: 滤波因子，决定滤波效果，h越大滤波越好但相位滞后越大
 * - T: 采样周期，应与控制周期一致
 */
class TrackingDifferentiator {
public:
    /**
     * @brief 默认构造函数
     */
    TrackingDifferentiator() = default;

    /**
     * @brief 构造函数
     * @param r 速度因子，r越大跟踪越快
     * @param h 滤波因子，h越大滤波效果越好
     * @param T 采样周期(s)
     */
    TrackingDifferentiator(double r, double h, double T)
        : m_r(r), m_h(h), m_T(T) {}

    /**
     * @brief 初始化参数
     * @param r 速度因子
     * @param h 滤波因子
     * @param T 采样周期
     */
    void init(double r, double h, double T) {
        m_r = r;
        m_h = h;
        m_T = T;
        reset();
    }

    /**
     * @brief 更新参数（不重置状态）
     * @param r 速度因子
     * @param h 滤波因子
     */
    void setParameters(double r, double h) {
        m_r = r;
        m_h = h;
    }

    /**
     * @brief 重置状态
     * @param init_pos 初始位置
     */
    void reset(double init_pos = 0.0) {
            m_x1 = init_pos;    
            m_x2 = 0.0;
            m_x = init_pos - m_aim;
            m_y = 0.0;
            m_a = 0.0;
            m_fhan = 0.0;
    }

    /**
     * @brief 设置目标值
     * @param aim 目标位置
     */
    void setTarget(double aim) {
        m_aim = aim;
    }

    /**
     * @brief 获取当前目标值
     * @return 目标位置
     */
    double getTarget() const {
        return m_aim;
    }

    /**
     * @brief 符号函数
     * @param x 输入值
     * @return 符号（1.0, -1.0, 或 0.0）
     */
    static double sgn(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return 0.0;
    }

    /**
     * @brief 最速控制综合函数fhan计算
     *
     * 执行一步TD更新，计算新的位置、速度和加速度。
     * 应在每个控制周期调用一次。
     */
    void calculate() {
        m_x = m_x1 - m_aim;
        double d = m_r * m_h;
        double d0 = m_h * d;
        double y = m_x + m_h * m_x2;
        double a0 = std::sqrt(d * d + 8.0 * m_r * std::abs(y));

        double a;
        if (std::abs(y) > d0) {
            a = m_x2 + (a0 - d) * sgn(y) / 2.0;
        } else {
            a = m_x2 + y / m_h;
        }

        double fhan;
        if (std::abs(a) > d) {
            fhan = -1.0 * m_r * sgn(a);
        } else {
            fhan = -1.0 * m_r * a / d;
        }

        m_y = y;
        m_a = a;
        m_fhan = fhan;

        m_x1 += m_T * m_x2;
        m_x2 += m_T * fhan;
    }

    /**
     * @brief 获取当前位置
     * @return 位置值
     */
    double getPosition() const {
        return m_x1;
    }

    /**
     * @brief 获取当前速度
     * @return 速度值
     */
    double getVelocity() const {
        return m_x2;
    }

    /**
     * @brief 获取当前加速度（控制量）
     * @return 加速度值
     */
    double getAcceleration() const {
        return m_fhan;
    }

    /**
     * @brief 检查是否到达目标
     * @param pos_threshold 位置误差阈值（默认0.01）
     * @param vel_threshold 速度阈值（默认0.05）
     * @return 如果位置和速度都在阈值内返回true
     */
    bool hasArrived(double pos_threshold = 0.01, double vel_threshold = 0.05) const {
        return std::abs(m_x1 - m_aim) < pos_threshold && std::abs(m_x2) < vel_threshold;
    }

    /**
     * @brief 获取位置误差
     * @return 当前位置与目标的差值
     */
    double getPositionError() const {
        return m_x1 - m_aim;
    }

private:
    double m_x1 = 0.0;      ///< 位置状态
    double m_x2 = 0.0;      ///< 速度状态
    double m_x = 0.0;       ///< 误差
    double m_r = 1.0;       ///< 速度因子
    double m_h = 0.001;     ///< 滤波因子
    double m_T = 0.001;     ///< 采样周期
    double m_aim = 0.0;     ///< 目标值
    double m_y = 0.0;       ///< 中间变量y
    double m_a = 0.0;       ///< 中间变量a
    double m_fhan = 0.0;    ///< 最速控制函数输出（加速度）
};

#endif  // TD_HPP_
