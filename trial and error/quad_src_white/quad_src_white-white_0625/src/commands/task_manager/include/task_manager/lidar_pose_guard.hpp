#ifndef TASK_MANAGER_LIDAR_POSE_GUARD_HPP_
#define TASK_MANAGER_LIDAR_POSE_GUARD_HPP_

namespace task_manager {
namespace lidar_pose_guard {

constexpr double kMinX = -10.0;
constexpr double kMaxX = 10.0;
constexpr double kMinY = -10.0;
constexpr double kMaxY = 15.0;

/**
 * @brief 判断雷达系位姿是否超出安全范围
 *
 * 输入：x、y（camera_init / aft_mapped 下平移，单位 m）
 * 输出：true 表示越界（雷达可能飘了）
 * 处理：x∉[-5,5] 或 y∉[0,10] 则判定异常
 */
inline bool is_out_of_bounds(double x, double y) {
    return x < kMinX || x > kMaxX || y < kMinY || y > kMaxY;
}

}  // namespace lidar_pose_guard
}  // namespace task_manager

#endif  // TASK_MANAGER_LIDAR_POSE_GUARD_HPP_
