#ifndef LIDAR_NAV_DEMO_CPP_ISOLATION_BAND_HPP_
#define LIDAR_NAV_DEMO_CPP_ISOLATION_BAND_HPP_

#include "lidar_nav_demo_cpp/lidar_a_star.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace lidar_nav {

constexpr double kTightLateralYMin = 0.8;
constexpr double kTightLateralYMax = 1.3;
constexpr double kIsolationMinDx = 0.2;

inline bool is_in_tight_lateral_y_zone(double y) {
    return y >= kTightLateralYMin - 1e-9 && y <= kTightLateralYMax + 1e-9;
}

inline bool is_corridor_lateral_only_arrival(
    double target_y, bool cross_isolation_band, double corridor_lateral_y_max)
{
    return cross_isolation_band && target_y <= corridor_lateral_y_max + 1e-9;
}

inline bool corridor_segment_lateral_only_arrival(
    bool cross_isolation_band,
    double target_y,
    bool is_segment_first,
    bool is_segment_last,
    double corridor_lateral_y_max)
{
    if (is_segment_first || is_segment_last) {
        return false;
    }
    return is_corridor_lateral_only_arrival(
        target_y, cross_isolation_band, corridor_lateral_y_max);
}

inline bool is_in_precise_lateral_arrival_y_zone(
    double y, double y_min, double y_max)
{
    return y >= y_min - 1e-9 && y <= y_max + 1e-9;
}

inline bool is_in_rear_corridor_yaw_free_zone(double y, double y_min, double y_max) {
    return y >= y_min - 1e-9 && y <= y_max + 1e-9;
}

inline bool is_in_rear_exit_creep_y_zone(double y, double y_min, double y_max) {
    return y >= y_min - 1e-9 && y <= y_max + 1e-9;
}

inline bool in_isolation_y_range(double y, double y_min, double y_max) {
    return y >= y_min - 1e-9 && y <= y_max + 1e-9;
}

inline bool blocked_by_y_x_isolation(
    double x1, double y1, double x2, double y2,
    double y_min, double y_max, double min_dx = kIsolationMinDx)
{
    if (std::abs(x1 - x2) <= min_dx + 1e-9) {
        return false;
    }
    return in_isolation_y_range(y1, y_min, y_max) &&
           in_isolation_y_range(y2, y_min, y_max);
}

class IsolationBandRegion {
public:
    void configure(
        double y_min, double y_max,
        double neighbor_radius, double containment_radius);

    void build_from_add_points(const std::vector<GraphPoint>& nodes);

    bool contains(double x, double y) const;
    bool path_intersects(
        const std::vector<GraphPoint>& path,
        size_t begin, size_t end_exclusive) const;

    bool empty() const { return nodes_.empty(); }
    std::size_t node_count() const { return nodes_.size(); }

private:
    static double euclidean_distance(double x1, double y1, double x2, double y2);

    double y_min_{0.9};
    double y_max_{3.8};
    double neighbor_radius_{1.0};
    double containment_radius_{1.0};
    std::vector<GraphPoint> nodes_;
};

}  // namespace lidar_nav

#endif  // LIDAR_NAV_DEMO_CPP_ISOLATION_BAND_HPP_
