#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

// 一阶低通滤波器函数
inline double lowPassFilter(double input, double prev_output, double tau, double dt) {
    double alpha = tau / (tau + dt);
    return alpha * prev_output + (1.0 - alpha) * input;
}

#endif // LOWPASS_FILTER_H