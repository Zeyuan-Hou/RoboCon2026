#include "lidar_nav_demo_cpp/isolation_band.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace lidar_nav {

void IsolationBandRegion::configure(
    double y_min, double y_max,
    double neighbor_radius, double containment_radius)
{
    y_min_ = y_min;
    y_max_ = y_max;
    neighbor_radius_ = neighbor_radius;
    containment_radius_ = containment_radius > 0.0 ? containment_radius : neighbor_radius;
    nodes_.clear();
}

double IsolationBandRegion::euclidean_distance(
    double x1, double y1, double x2, double y2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void IsolationBandRegion::build_from_add_points(const std::vector<GraphPoint>& nodes) {
    nodes_.clear();
    if (nodes.empty()) {
        return;
    }

    std::vector<GraphPoint> filtered;
    filtered.reserve(nodes.size());
    for (const auto& node : nodes) {
        if (in_isolation_y_range(node.y, y_min_, y_max_)) {
            filtered.push_back(node);
        }
    }
    if (filtered.empty()) {
        return;
    }

    const size_t n = filtered.size();
    std::vector<std::vector<size_t>> adj(n);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const double d = euclidean_distance(
                filtered[i].x, filtered[i].y,
                filtered[j].x, filtered[j].y);
            if (d <= neighbor_radius_ + 1e-9) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    std::vector<int> component(n, -1);
    int best_component = -1;
    size_t best_size = 0;

    for (size_t start = 0; start < n; ++start) {
        if (component[start] >= 0) {
            continue;
        }
        const int comp_id = static_cast<int>(start);
        std::queue<size_t> q;
        q.push(start);
        component[start] = comp_id;
        size_t comp_size = 0;

        while (!q.empty()) {
            const size_t u = q.front();
            q.pop();
            ++comp_size;
            for (size_t v : adj[u]) {
                if (component[v] >= 0) {
                    continue;
                }
                component[v] = comp_id;
                q.push(v);
            }
        }

        if (comp_size > best_size) {
            best_size = comp_size;
            best_component = comp_id;
        }
    }

    nodes_.reserve(best_size);
    for (size_t i = 0; i < n; ++i) {
        if (component[i] == best_component) {
            nodes_.push_back(filtered[i]);
        }
    }
}

bool IsolationBandRegion::contains(double x, double y) const {
    if (nodes_.empty()) {
        return false;
    }
    for (const auto& node : nodes_) {
        if (euclidean_distance(x, y, node.x, node.y) <= containment_radius_ + 1e-9) {
            return true;
        }
    }
    return false;
}

bool IsolationBandRegion::path_intersects(
    const std::vector<GraphPoint>& path,
    size_t begin, size_t end_exclusive) const
{
    if (nodes_.empty() || begin >= end_exclusive || end_exclusive > path.size()) {
        return false;
    }
    for (size_t i = begin; i < end_exclusive; ++i) {
        if (contains(path[i].x, path[i].y)) {
            return true;
        }
    }
    return false;
}

}  // namespace lidar_nav
