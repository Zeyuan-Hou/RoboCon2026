#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct AStarNode {
    int x, y;
    double g_cost, h_cost;
    int parent_index;

    AStarNode(int x_, int y_, double g, double h, int parent)
        : x(x_), y(y_), g_cost(g), h_cost(h), parent_index(parent) {}

    double f_cost() const { return g_cost + h_cost; }

    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

class GlobalPlannerNode : public rclcpp::Node
{
public:
    GlobalPlannerNode() : Node("global_planner_node")
    {
        // Parameters
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("heuristic_weight", 1.0);

        heuristic_weight_ = this->get_parameter("heuristic_weight").as_double();

        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&GlobalPlannerNode::mapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        RCLCPP_INFO(this->get_logger(), "Global Planner Node has been started.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        has_map_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_map_ || !has_odom_) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or odom");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planning path to goal...");

        // Convert start and goal to map coordinates
        int start_x, start_y, goal_x, goal_y;
        if (!worldToMap(current_pose_.position.x, current_pose_.position.y, start_x, start_y)) {
            RCLCPP_WARN(this->get_logger(), "Start pose is out of map bounds!");
            return;
        }
        if (!worldToMap(msg->pose.position.x, msg->pose.position.y, goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "Goal pose is out of map bounds!");
            return;
        }

        auto path_cells = planAStar(start_x, start_y, goal_x, goal_y);

        if (path_cells.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to find a path!");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;

        for (const auto& cell : path_cells) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            double wx, wy;
            mapToWorld(cell.first, cell.second, wx, wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published global path with %zu points", path_msg.poses.size());
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) const {
        double origin_x = map_.info.origin.position.x;
        double origin_y = map_.info.origin.position.y;
        double res = map_.info.resolution;

        if (wx < origin_x || wy < origin_y) return false;

        mx = (wx - origin_x) / res;
        my = (wy - origin_y) / res;

        if (mx < 0 || mx >= (int)map_.info.width || my < 0 || my >= (int)map_.info.height) {
            return false;
        }
        return true;
    }

    void mapToWorld(int mx, int my, double& wx, double& wy) const {
        double origin_x = map_.info.origin.position.x;
        double origin_y = map_.info.origin.position.y;
        double res = map_.info.resolution;

        wx = origin_x + (mx + 0.5) * res;
        wy = origin_y + (my + 0.5) * res;
    }

    int getIndex(int mx, int my) const {
        return my * map_.info.width + mx;
    }

    bool isFree(int mx, int my) const {
        if (mx < 0 || mx >= (int)map_.info.width || my < 0 || my >= (int)map_.info.height) return false;
        int cost = map_.data[getIndex(mx, my)];
        return cost >= 0 && cost <= 50; // 0=free, 100=obstacle, -1=unknown. We can allow free or unknown.
    }

    double heuristic(int x1, int y1, int x2, int y2) const {
        return std::hypot(x1 - x2, y1 - y2) * heuristic_weight_;
    }

    std::vector<std::pair<int, int>> planAStar(int start_x, int start_y, int goal_x, int goal_y) {
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::vector<bool> closed_set(map_.info.width * map_.info.height, false);
        std::vector<int> parent_map(map_.info.width * map_.info.height, -1);
        std::vector<double> g_cost_map(map_.info.width * map_.info.height, std::numeric_limits<double>::infinity());

        int start_idx = getIndex(start_x, start_y);
        g_cost_map[start_idx] = 0;
        open_set.emplace(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y), -1);

        int dirs[8][2] = {{1,0}, {0,1}, {-1,0}, {0,-1}, {1,1}, {-1,1}, {-1,-1}, {1,-1}};
        double costs[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

        int best_node_x = start_x;
        int best_node_y = start_y;

        bool found = false;

        while (!open_set.empty()) {
            AStarNode current = open_set.top();
            open_set.pop();

            int curr_idx = getIndex(current.x, current.y);
            if (closed_set[curr_idx]) continue;
            closed_set[curr_idx] = true;
            parent_map[curr_idx] = current.parent_index;

            if (current.x == goal_x && current.y == goal_y) {
                best_node_x = current.x;
                best_node_y = current.y;
                found = true;
                break;
            }

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dirs[i][0];
                int ny = current.y + dirs[i][1];

                if (!isFree(nx, ny)) continue;

                int n_idx = getIndex(nx, ny);
                if (closed_set[n_idx]) continue;

                double new_g = current.g_cost + costs[i];
                if (new_g < g_cost_map[n_idx]) {
                    g_cost_map[n_idx] = new_g;
                    open_set.emplace(nx, ny, new_g, heuristic(nx, ny, goal_x, goal_y), curr_idx);
                }
            }
        }

        std::vector<std::pair<int, int>> path;
        if (found) {
            int curr_idx = getIndex(best_node_x, best_node_y);
            while (curr_idx != -1) {
                int cx = curr_idx % map_.info.width;
                int cy = curr_idx / map_.info.width;
                path.push_back({cx, cy});
                curr_idx = parent_map[curr_idx];
            }
            std::reverse(path.begin(), path.end());
        }

        return path;
    }

    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_map_ = false;
    bool has_odom_ = false;
    double heuristic_weight_ = 1.0;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
