#ifndef INTER_POLATE_H
#define INTER_POLATE_H
#include <cmath>

/**
 * @brief 
 * 余弦插值
 * @param[in] start_pos       起始值
 * @param[in] target_pos      目标值
 * @param[in] T             过程总时间
 * @param[in] t             经过的时间
 * @return double 
 */
inline double cosineInterpolate(double start_pos, double target_pos, double T, double t) {
    double s;
    if (t >= T) {
        s = 1.0;
    } else {
        s = 0.5 * (1 - cos(M_PI * t / T));
    }
    return start_pos + s * (target_pos - start_pos);
}

#endif // INTER_POLATE_H