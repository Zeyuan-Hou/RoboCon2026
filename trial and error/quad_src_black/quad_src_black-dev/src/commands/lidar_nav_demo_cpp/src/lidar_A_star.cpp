#include "lidar_nav_demo_cpp/lidar_a_star.hpp"
#include "lidar_nav_demo_cpp/isolation_band.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>

namespace {

std::string svg_escape(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        switch (ch) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            default: out += ch; break;
        }
    }
    return out;
}

const char* cargo_class_svg_color(int cargo_class) {
    static const char* kColors[] = {"#2ca02c", "#888888", "#1f77b4", "#d62728"};
    if (cargo_class < 0 || cargo_class > 3) {
        return "#cccccc";
    }
    return kColors[cargo_class];
}

const char* cargo_class_label(int cargo_class) {
    static const char* kNames[] = {"G", "S", "B", "R"};
    if (cargo_class < 0 || cargo_class > 3) {
        return "?";
    }
    return kNames[cargo_class];
}

void append_map_square(
    std::ostringstream& svg,
    const std::function<std::pair<double, double>(double, double)>& to_svg,
    double cx, double cy, double side,
    const char* fill, const char* stroke, double stroke_width)
{
    const double half = side * 0.5;
    const auto nw = to_svg(cx - half, cy + half);
    const auto se = to_svg(cx + half, cy - half);
    const double rect_x = std::min(nw.first, se.first);
    const double rect_y = std::min(nw.second, se.second);
    const double rect_w = std::abs(se.first - nw.first);
    const double rect_h = std::abs(se.second - nw.second);
    svg << "<rect x=\"" << rect_x << "\" y=\"" << rect_y
        << "\" width=\"" << rect_w << "\" height=\"" << rect_h
        << "\" fill=\"" << fill << "\" stroke=\"" << stroke
        << "\" stroke-width=\"" << stroke_width << "\"/>\n";
}

}  // namespace

void LidarAStar::set_neighbor_radius(double radius) {
    neighbor_radius_ = radius;
    rebuild_adjacency();
}

void LidarAStar::set_isolation_y_range(double y_min, double y_max) {
    isolation_y_min_ = y_min;
    isolation_y_max_ = y_max;
    rebuild_adjacency();
}

void LidarAStar::load_static_nodes(const std::vector<GraphPoint>& nodes) {
    static_nodes_ = nodes;
    rebuild_adjacency();
}

bool LidarAStar::load_add_point_file(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::vector<GraphPoint> loaded;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        GraphPoint point;
        if (!(iss >> point.x >> point.y >> point.yaw)) {
            continue;
        }
        loaded.push_back(point);
    }
    static_nodes_ = std::move(loaded);
    rebuild_adjacency();
    return true;
}

double LidarAStar::euclidean_distance(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void LidarAStar::rebuild_adjacency() {
    const size_t n = static_nodes_.size();
    adj_.assign(n, {});
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const double d = euclidean_distance(
                static_nodes_[i].x, static_nodes_[i].y,
                static_nodes_[j].x, static_nodes_[j].y);
            if (d <= neighbor_radius_ + 1e-9 &&
                !lidar_nav::blocked_by_y_x_isolation(
                    static_nodes_[i].x, static_nodes_[i].y,
                    static_nodes_[j].x, static_nodes_[j].y,
                    isolation_y_min_, isolation_y_max_)) {
                adj_[i].emplace_back(j, d);
                adj_[j].emplace_back(i, d);
            }
        }
    }
}

size_t LidarAStar::nearest_static_node(double x, double y) const {
    if (static_nodes_.empty()) {
        return 0;
    }
    size_t best = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < static_nodes_.size(); ++i) {
        const double d = euclidean_distance(x, y, static_nodes_[i].x, static_nodes_[i].y);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

double LidarAStar::nearest_static_distance(double x, double y) const {
    if (static_nodes_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    return euclidean_distance(
        x, y,
        static_nodes_[nearest_static_node(x, y)].x,
        static_nodes_[nearest_static_node(x, y)].y);
}

double LidarAStar::yaw_for_virtual_node(double x, double y) const {
    if (static_nodes_.empty()) {
        return 0.0;
    }
    return static_nodes_[nearest_static_node(x, y)].yaw;
}

AStarConnectivity LidarAStar::check_connectivity(
    double start_x, double start_y,
    double goal_x, double goal_y) const
{
    AStarConnectivity info;
    if (static_nodes_.empty()) {
        return info;
    }

    const size_t n = static_nodes_.size();
    info.start_nearest_dist = nearest_static_distance(start_x, start_y);
    info.goal_nearest_dist = nearest_static_distance(goal_x, goal_y);
    info.start_connected = info.start_nearest_dist <= neighbor_radius_ + 1e-9;
    info.goal_connected = info.goal_nearest_dist <= neighbor_radius_ + 1e-9;

    std::vector<size_t> component(n, n);
    size_t comp_id = 0;
    for (size_t i = 0; i < n; ++i) {
        if (component[i] != n) {
            continue;
        }
        std::queue<size_t> q;
        q.push(i);
        component[i] = comp_id;
        size_t comp_size = 0;
        while (!q.empty()) {
            const size_t u = q.front();
            q.pop();
            comp_size++;
            for (const auto& edge : adj_[u]) {
                const size_t v = edge.first;
                if (component[v] == n) {
                    component[v] = comp_id;
                    q.push(v);
                }
            }
        }
        if (info.start_connected) {
            const size_t start_anchor = nearest_static_node(start_x, start_y);
            if (component[start_anchor] == comp_id) {
                info.start_component_size = comp_size;
            }
        }
        if (info.goal_connected) {
            const size_t goal_anchor = nearest_static_node(goal_x, goal_y);
            if (component[goal_anchor] == comp_id) {
                info.goal_component_size = comp_size;
            }
        }
        comp_id++;
    }

    if (!info.start_connected || !info.goal_connected) {
        return info;
    }

    const size_t start_id = n;
    const size_t goal_id = n + 1;
    const size_t total = n + 2;
    std::vector<std::vector<size_t>> query_adj(total);
    for (size_t i = 0; i < n; ++i) {
        for (const auto& edge : adj_[i]) {
            query_adj[i].push_back(edge.first);
            query_adj[edge.first].push_back(i);
        }
    }
    for (size_t i = 0; i < n; ++i) {
        const double ds = euclidean_distance(start_x, start_y, static_nodes_[i].x, static_nodes_[i].y);
        if (ds <= neighbor_radius_ + 1e-9) {
            query_adj[start_id].push_back(i);
            query_adj[i].push_back(start_id);
        }
        const double dg = euclidean_distance(goal_x, goal_y, static_nodes_[i].x, static_nodes_[i].y);
        if (dg <= neighbor_radius_ + 1e-9) {
            query_adj[goal_id].push_back(i);
            query_adj[i].push_back(goal_id);
        }
    }

    std::vector<bool> visited(total, false);
    std::queue<size_t> q;
    q.push(start_id);
    visited[start_id] = true;
    while (!q.empty()) {
        const size_t u = q.front();
        q.pop();
        if (u == goal_id) {
            info.graph_path_exists = true;
            break;
        }
        for (size_t v : query_adj[u]) {
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }
    return info;
}

std::vector<GraphPoint> LidarAStar::find_shortest_path(
    double start_x, double start_y,
    double goal_x, double goal_y) const
{
    std::vector<GraphPoint> empty;
    if (static_nodes_.empty()) {
        return empty;
    }

    const auto connectivity = check_connectivity(start_x, start_y, goal_x, goal_y);
    if (!connectivity.start_connected || !connectivity.goal_connected) {
        return empty;
    }

    const size_t n = static_nodes_.size();
    const size_t start_id = n;
    const size_t goal_id = n + 1;
    const size_t total = n + 2;

    std::vector<std::vector<std::pair<size_t, double>>> query_adj(total);
    for (size_t i = 0; i < n; ++i) {
        query_adj[i] = adj_[i];
    }

    for (size_t i = 0; i < n; ++i) {
        const double ds = euclidean_distance(start_x, start_y, static_nodes_[i].x, static_nodes_[i].y);
        if (ds <= neighbor_radius_ + 1e-9) {
            query_adj[start_id].emplace_back(i, ds);
            query_adj[i].emplace_back(start_id, ds);
        }
        const double dg = euclidean_distance(goal_x, goal_y, static_nodes_[i].x, static_nodes_[i].y);
        if (dg <= neighbor_radius_ + 1e-9) {
            query_adj[goal_id].emplace_back(i, dg);
            query_adj[i].emplace_back(goal_id, dg);
        }
    }

    const double start_yaw = yaw_for_virtual_node(start_x, start_y);
    const double goal_yaw = yaw_for_virtual_node(goal_x, goal_y);

    auto heuristic = [&](size_t node_id) -> double {
        if (node_id == start_id) {
            return euclidean_distance(start_x, start_y, goal_x, goal_y);
        }
        if (node_id == goal_id) {
            return 0.0;
        }
        return euclidean_distance(static_nodes_[node_id].x, static_nodes_[node_id].y, goal_x, goal_y);
    };

    std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
    std::vector<size_t> came_from(total, total);
    using QueueItem = std::pair<double, size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open_set;

    g_score[start_id] = 0.0;
    open_set.emplace(heuristic(start_id), start_id);

    while (!open_set.empty()) {
        const size_t current = open_set.top().second;
        open_set.pop();

        if (current == goal_id) {
            break;
        }

        for (const auto& edge : query_adj[current]) {
            const size_t neighbor = edge.first;
            const double tentative = g_score[current] + edge.second;
            if (tentative < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative;
                open_set.emplace(tentative + heuristic(neighbor), neighbor);
            }
        }
    }

    if (!std::isfinite(g_score[goal_id])) {
        return empty;
    }

    std::vector<size_t> node_chain;
    for (size_t node = goal_id; node != start_id; node = came_from[node]) {
        if (node >= total || came_from[node] == total) {
            return empty;
        }
        node_chain.push_back(node);
    }
    node_chain.push_back(start_id);
    std::reverse(node_chain.begin(), node_chain.end());

    std::vector<GraphPoint> path;
    path.reserve(node_chain.size());
    for (size_t node_id : node_chain) {
        if (node_id < n) {
            path.push_back(static_nodes_[node_id]);
        } else if (node_id == start_id) {
            path.push_back({start_x, start_y, start_yaw});
        } else {
            path.push_back({goal_x, goal_y, goal_yaw});
        }
    }
    return path;
}

void LidarAStar::set_logistics_waypoints_overlay(const std::vector<GraphPoint>& points) {
    logistics_overlay_ = points;
}

void LidarAStar::set_path_viz_slots(
    const std::vector<PathVizPickupSlot>& pickups,
    const std::vector<PathVizDropoffSlot>& dropoffs)
{
    path_viz_pickup_slots_ = pickups;
    path_viz_dropoff_slots_ = dropoffs;
}

void LidarAStar::visualize_segment(
    const std::vector<GraphPoint>& segment_path,
    const std::string& out_svg_path,
    double x_min, double x_max, double y_min, double y_max) const
{
    constexpr int kWidth = 800;
    constexpr int kHeight = 720;
    constexpr int kMargin = 40;

    const double x_span = std::max(x_max - x_min, 1e-6);
    const double y_span = std::max(y_max - y_min, 1e-6);

    auto to_svg = [&](double x, double y) -> std::pair<double, double> {
        const double sx = kMargin + (x - x_min) / x_span * (kWidth - 2 * kMargin);
        const double sy = kHeight - kMargin - (y - y_min) / y_span * (kHeight - 2 * kMargin);
        return {sx, sy};
    };

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << kWidth
        << "\" height=\"" << kHeight << "\" viewBox=\"0 0 " << kWidth << " " << kHeight << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#f8f8f8\"/>\n";
    svg << "<text x=\"" << kMargin << "\" y=\"24\" font-size=\"14\" fill=\"#333\">"
        << svg_escape(out_svg_path) << "</text>\n";

    constexpr double kGridStep = 0.5;
    const double grid_x_start = std::floor(x_min / kGridStep) * kGridStep;
    const double grid_y_start = std::floor(y_min / kGridStep) * kGridStep;
    svg << std::fixed << std::setprecision(1);
    for (double gx = grid_x_start; gx <= x_max + 1e-9; gx += kGridStep) {
        const auto p1 = to_svg(gx, y_min);
        const auto p2 = to_svg(gx, y_max);
        svg << "<line x1=\"" << p1.first << "\" y1=\"" << p1.second
            << "\" x2=\"" << p2.first << "\" y2=\"" << p2.second
            << "\" stroke=\"#d0d0d0\" stroke-width=\"1\"/>\n";
        const auto label_p = to_svg(gx, y_min);
        svg << "<text x=\"" << label_p.first << "\" y=\"" << (kHeight - 8)
            << "\" font-size=\"10\" fill=\"#999\" text-anchor=\"middle\">"
            << gx << "</text>\n";
    }
    for (double gy = grid_y_start; gy <= y_max + 1e-9; gy += kGridStep) {
        const auto p1 = to_svg(x_min, gy);
        const auto p2 = to_svg(x_max, gy);
        svg << "<line x1=\"" << p1.first << "\" y1=\"" << p1.second
            << "\" x2=\"" << p2.first << "\" y2=\"" << p2.second
            << "\" stroke=\"#d0d0d0\" stroke-width=\"1\"/>\n";
        const auto label_p = to_svg(x_min, gy);
        svg << "<text x=\"8\" y=\"" << (label_p.second + 3)
            << "\" font-size=\"10\" fill=\"#999\" text-anchor=\"start\">"
            << gy << "</text>\n";
    }

    for (size_t i = 0; i < static_nodes_.size(); ++i) {
        for (const auto& edge : adj_[i]) {
            if (edge.first <= i) {
                continue;
            }
            const auto p1 = to_svg(static_nodes_[i].x, static_nodes_[i].y);
            const auto p2 = to_svg(static_nodes_[edge.first].x, static_nodes_[edge.first].y);
            svg << "<line x1=\"" << p1.first << "\" y1=\"" << p1.second
                << "\" x2=\"" << p2.first << "\" y2=\"" << p2.second
                << "\" stroke=\"#bbbbbb\" stroke-width=\"1\"/>\n";
        }
    }

    for (const auto& node : static_nodes_) {
        const auto p = to_svg(node.x, node.y);
        svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
            << "\" r=\"4\" fill=\"#3366cc\"/>\n";
    }

    constexpr double kPickupSquareSide = 0.25;
    constexpr double kDropoffSquareSide = 0.22;
    constexpr double kPriorityOutlineSide = 0.32;

    if (!path_viz_dropoff_slots_.empty()) {
        for (const auto& slot : path_viz_dropoff_slots_) {
            append_map_square(
                svg, to_svg, slot.x, slot.y, kDropoffSquareSide,
                cargo_class_svg_color(slot.cargo_class), "#444444", 1.0);
            const auto p = to_svg(slot.x, slot.y);
            svg << "<text x=\"" << p.first << "\" y=\"" << (p.second + 4)
                << "\" font-size=\"11\" fill=\"#111\" text-anchor=\"middle\" font-weight=\"bold\">"
                << "D" << slot.dropoff_index
                << cargo_class_label(slot.cargo_class)
                << (slot.layer > 0 ? "L2" : "")
                << "</text>\n";
        }
    }

    if (!path_viz_pickup_slots_.empty()) {
        for (const auto& slot : path_viz_pickup_slots_) {
            if (slot.priority_first) {
                append_map_square(
                    svg, to_svg, slot.x, slot.y, kPriorityOutlineSide,
                    "none", "#ffd700", 3.5);
            }
            append_map_square(
                svg, to_svg, slot.x, slot.y, kPickupSquareSide,
                cargo_class_svg_color(slot.cargo_class), "#222222", 1.2);
            const auto p = to_svg(slot.x, slot.y);
            svg << "<text x=\"" << p.first << "\" y=\"" << (p.second + 4)
                << "\" font-size=\"12\" fill=\"#111\" text-anchor=\"middle\" font-weight=\"bold\">"
                << slot.eightboxes_slot << "</text>\n";
        }
    } else {
        for (const auto& wp : logistics_overlay_) {
            const auto p = to_svg(wp.x, wp.y);
            svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
                << "\" r=\"5\" fill=\"#ff69b4\" stroke=\"#c71585\" stroke-width=\"1.5\"/>\n";
        }
    }

    if (!path_viz_pickup_slots_.empty() || !path_viz_dropoff_slots_.empty()) {
        svg << "<rect x=\"520\" y=\"8\" width=\"270\" height=\"92\" fill=\"#ffffffcc\" stroke=\"#999\" rx=\"4\"/>\n";
        svg << "<text x=\"528\" y=\"24\" font-size=\"11\" fill=\"#333\">"
            << "Pickup: eightboxes slot / color</text>\n";
        svg << "<text x=\"528\" y=\"40\" font-size=\"11\" fill=\"#333\">"
            << "Dropoff: Dindex + G/Y/B/R (+L2)</text>\n";
        svg << "<rect x=\"528\" y=\"48\" width=\"12\" height=\"12\" fill=\"#2ca02c\" stroke=\"#222\"/>\n";
        svg << "<text x=\"544\" y=\"58\" font-size=\"10\" fill=\"#333\">0 green</text>\n";
        svg << "<rect x=\"600\" y=\"48\" width=\"12\" height=\"12\" fill=\"#888888\" stroke=\"#222\"/>\n";
        svg << "<text x=\"616\" y=\"58\" font-size=\"10\" fill=\"#333\">1 gray</text>\n";
        svg << "<rect x=\"660\" y=\"48\" width=\"12\" height=\"12\" fill=\"#1f77b4\" stroke=\"#222\"/>\n";
        svg << "<text x=\"676\" y=\"58\" font-size=\"10\" fill=\"#333\">2 blue</text>\n";
        svg << "<rect x=\"720\" y=\"48\" width=\"12\" height=\"12\" fill=\"#d62728\" stroke=\"#222\"/>\n";
        svg << "<text x=\"736\" y=\"58\" font-size=\"10\" fill=\"#333\">3 red</text>\n";
        svg << "<rect x=\"528\" y=\"68\" width=\"14\" height=\"14\" fill=\"none\" stroke=\"#ffd700\" stroke-width=\"3\"/>\n";
        svg << "<text x=\"548\" y=\"80\" font-size=\"10\" fill=\"#333\">priority 1st pickup</text>\n";
    }

    if (!segment_path.empty()) {
        const auto start_p = to_svg(segment_path.front().x, segment_path.front().y);
        const auto goal_p = to_svg(segment_path.back().x, segment_path.back().y);
        svg << "<circle cx=\"" << start_p.first << "\" cy=\"" << start_p.second
            << "\" r=\"6\" fill=\"#2ca02c\"/>\n";
        svg << "<circle cx=\"" << goal_p.first << "\" cy=\"" << goal_p.second
            << "\" r=\"6\" fill=\"#2ca02c\"/>\n";

        svg << "<polyline fill=\"none\" stroke=\"#d62728\" stroke-width=\"3\" points=\"";
        for (const auto& point : segment_path) {
            const auto p = to_svg(point.x, point.y);
            svg << p.first << "," << p.second << " ";
        }
        svg << "\"/>\n";
    }

    svg << "</svg>\n";

    std::ofstream out(out_svg_path);
    if (out.is_open()) {
        out << svg.str();
    }
}
