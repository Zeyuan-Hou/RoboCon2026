#ifndef LIDAR_NAV_DEMO_CPP_LIDAR_A_STAR_HPP_
#define LIDAR_NAV_DEMO_CPP_LIDAR_A_STAR_HPP_

#include <limits>
#include <string>
#include <utility>
#include <vector>

struct GraphPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct PathVizPickupSlot {
    double x{0.0};
    double y{0.0};
    int eightboxes_slot{0};
    int cargo_class{0};
    bool priority_first{false};
};

struct PathVizDropoffSlot {
    double x{0.0};
    double y{0.0};
    int dropoff_index{0};
    int cargo_class{0};
    int layer{0};
};

struct AStarConnectivity {
    bool start_connected{false};
    bool goal_connected{false};
    bool graph_path_exists{false};
    double start_nearest_dist{std::numeric_limits<double>::infinity()};
    double goal_nearest_dist{std::numeric_limits<double>::infinity()};
    size_t start_component_size{0};
    size_t goal_component_size{0};
};

class LidarAStar {
public:
    void set_neighbor_radius(double radius);
    void set_isolation_y_range(double y_min, double y_max);
    void load_static_nodes(const std::vector<GraphPoint>& nodes);
    bool load_add_point_file(const std::string& path);

    const std::vector<GraphPoint>& static_nodes() const { return static_nodes_; }
    const std::vector<std::vector<std::pair<size_t, double>>>& adjacency() const { return adj_; }
    double neighbor_radius() const { return neighbor_radius_; }

    AStarConnectivity check_connectivity(
        double start_x, double start_y,
        double goal_x, double goal_y) const;

    std::vector<GraphPoint> find_shortest_path(
        double start_x, double start_y,
        double goal_x, double goal_y) const;

    void set_logistics_waypoints_overlay(const std::vector<GraphPoint>& points);
    void set_path_viz_slots(
        const std::vector<PathVizPickupSlot>& pickups,
        const std::vector<PathVizDropoffSlot>& dropoffs);

    void visualize_segment(
        const std::vector<GraphPoint>& segment_path,
        const std::string& out_svg_path,
        double x_min, double x_max, double y_min, double y_max) const;

    size_t nearest_static_node(double x, double y) const;

private:
    static double euclidean_distance(double x1, double y1, double x2, double y2);
    void rebuild_adjacency();
    double nearest_static_distance(double x, double y) const;
    double yaw_for_virtual_node(double x, double y) const;

    double neighbor_radius_{1.0};
    double isolation_y_min_{0.9};
    double isolation_y_max_{3.8};
    std::vector<GraphPoint> static_nodes_;
    std::vector<std::vector<std::pair<size_t, double>>> adj_;
    std::vector<GraphPoint> logistics_overlay_;
    std::vector<PathVizPickupSlot> path_viz_pickup_slots_;
    std::vector<PathVizDropoffSlot> path_viz_dropoff_slots_;
};

#endif  // LIDAR_NAV_DEMO_CPP_LIDAR_A_STAR_HPP_
