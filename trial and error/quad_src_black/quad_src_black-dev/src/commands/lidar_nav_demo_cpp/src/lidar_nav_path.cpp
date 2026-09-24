#include "lidar_nav_demo_cpp/isolation_band.hpp"
#include "lidar_nav_demo_cpp/lidar_nav_demo_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <limits>
#include <numeric>
#include <set>
#include <utility>
#include <sstream>
#include <string>
#include <vector>

bool LidarNavControl::validate_dropoff_color_config() const {
    if (dropoff_color_order_.size() != 4) {
        return false;
    }
    std::set<int> seen;
    for (int cls : dropoff_color_order_) {
        if (cls < 0 || cls > 3 || !seen.insert(cls).second) {
            return false;
        }
    }
    for (int slot : class_to_layer1_slot_) {
        if (slot < 0 || slot > 3) {
            return false;
        }
    }
    const size_t num_dropoff = dropoff_flat_.size() / 3;
    if (dropoff_layer_stride_ <= 0 ||
        static_cast<size_t>(dropoff_layer_stride_) * 2 > num_dropoff) {
        return false;
    }
    return true;
}

bool LidarNavControl::validate_eightboxes_pickup_index_map() const {
    if (eightboxes_pickup_index_map_.size() != 8) {
        return false;
    }
    std::set<int> seen;
    for (int idx : eightboxes_pickup_index_map_) {
        if (idx < 0 || idx > 7 || !seen.insert(idx).second) {
            return false;
        }
    }
    return true;
}

size_t LidarNavControl::pickup_index_for_eightboxes_slot(size_t string_idx) const {
    if (string_idx >= eightboxes_pickup_index_map_.size()) {
        return string_idx;
    }
    return static_cast<size_t>(eightboxes_pickup_index_map_[string_idx]);
}

size_t LidarNavControl::eightboxes_slot_for_pickup_index(size_t pickup_idx) const {
    for (size_t string_idx = 0; string_idx < eightboxes_pickup_index_map_.size(); ++string_idx) {
        if (eightboxes_pickup_index_map_[string_idx] == static_cast<int>(pickup_idx)) {
            return string_idx;
        }
    }
    return eightboxes_pickup_index_map_.size();
}

std::string LidarNavControl::cargo_color_name(int cargo_class) const {
    if (cargo_class >= 0 &&
        static_cast<size_t>(cargo_class) < cargo_color_names_.size()) {
        return cargo_color_names_[static_cast<size_t>(cargo_class)];
    }
    return "class_" + std::to_string(cargo_class);
}

void LidarNavControl::play_path_ready_audio() const {
    const int audio_id = ocr_result_locked_ ? ocr_result_ : 0;
    const std::string audio_path =
        "/home/cat/hitcrt_quad_ws/src/commands/lidar_nav_demo_cpp/records/"
        + std::to_string(audio_id) + ".wav";
    const std::string cmd = "aplay \"" + audio_path + "\" >/dev/null 2>&1 &";
    if (system(cmd.c_str()) != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to play audio: %s", audio_path.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Playing audio after path generation: %s (ocr_locked=%s)",
                audio_path.c_str(), ocr_result_locked_ ? "true" : "false");
}

void LidarNavControl::publish_boxsuc_handshake() {
    std_msgs::msg::String out;
    out.data = "666";
    for (int i = 0; i < 10; ++i) {
        boxsuc_pub_->publish(out);
    }
    RCLCPP_INFO(this->get_logger(),
                "Published '666' to boxsuc x10 after path generation.");
}

std::vector<int> LidarNavControl::compute_dropoff_indices(
    const std::vector<int>& classes) const
{
    std::vector<int> dropoff_indices(classes.size(), -1);
    std::array<int, 4> occ{};

    for (size_t i = 0; i < classes.size(); ++i) {
        const int d = classes[i];
        if (d < 0 || d > 3) {
            continue;
        }
        occ[static_cast<size_t>(d)]++;
        const int layer1_slot = class_to_layer1_slot_[static_cast<size_t>(d)];
        dropoff_indices[i] = layer1_slot + (occ[static_cast<size_t>(d)] - 1) * dropoff_layer_stride_;
    }
    return dropoff_indices;
}

std::vector<PickupDropTask> LidarNavControl::build_tasks(
    const std::vector<int>& classes) const
{
    std::vector<PickupDropTask> tasks;
    if (classes.size() != 8) {
        return tasks;
    }

    std::vector<size_t> visit_order;
    visit_order.reserve(8);

    for (size_t i = 0; i < 8; ++i) {
        visit_order.push_back(i);
    }

    const bool startup_pickup_valid =
        startup_pickup_enable_ &&
        startup_pickup_index_ >= 0 &&
        static_cast<size_t>(startup_pickup_index_) < pickup_flat_.size() / 4;
    const size_t startup_slot = startup_pickup_valid
        ? eightboxes_slot_for_pickup_index(static_cast<size_t>(startup_pickup_index_))
        : eightboxes_pickup_index_map_.size();
    const bool startup_slot_valid = startup_slot < 8;

    if (startup_slot_valid) {
        visit_order.erase(
            std::remove(visit_order.begin(), visit_order.end(), startup_slot),
            visit_order.end());
    }

    // Stable sort remaining tasks: back row (is_front < 0.5) before front row
    // (is_front >= 0.5), preserving relative order within each group.
    std::stable_sort(visit_order.begin(), visit_order.end(),
        [this](size_t a, size_t b) {
            const size_t phys_a = pickup_index_for_eightboxes_slot(a);
            const size_t phys_b = pickup_index_for_eightboxes_slot(b);
            const bool front_a = (phys_a < pickup_flat_.size() / 4) &&
                                 (pickup_flat_[phys_a * 4 + 3] >= 0.5);
            const bool front_b = (phys_b < pickup_flat_.size() / 4) &&
                                 (pickup_flat_[phys_b * 4 + 3] >= 0.5);
            return front_a < front_b;  // false(后排) < true(前排)
        });

    if (startup_slot_valid) {
        visit_order.insert(visit_order.begin(), startup_slot);
    }

    std::vector<size_t> remaining_slots;
    remaining_slots.reserve(visit_order.size());
    for (size_t string_idx : visit_order) {
        const int cargo_class = classes[string_idx];
        if (cargo_class < 0 || cargo_class > 3) {
            continue;
        }
        const int layer1_slot = class_to_layer1_slot_[static_cast<size_t>(cargo_class)];
        if (layer1_slot < 0 || layer1_slot > 3) {
            continue;
        }
        const size_t physical_pickup_idx = pickup_index_for_eightboxes_slot(string_idx);
        if (physical_pickup_idx >= pickup_flat_.size() / 4) {
            continue;
        }
        remaining_slots.push_back(string_idx);
    }

    std::vector<size_t> ordered_slots;
    ordered_slots.reserve(remaining_slots.size());
    std::array<int, 4> greedy_dropoff_occurrence{};
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    bool cursor_valid = false;
    auto append_ordered_slot = [&](size_t string_idx) {
        const int cargo_class = classes[string_idx];
        const int layer1_slot = class_to_layer1_slot_[static_cast<size_t>(cargo_class)];
        const int occurrence =
            ++greedy_dropoff_occurrence[static_cast<size_t>(cargo_class)];
        const int dropoff_idx =
            layer1_slot + (occurrence - 1) * dropoff_layer_stride_;
        if (dropoff_idx < 0 ||
            static_cast<size_t>(dropoff_idx) >= dropoff_flat_.size() / 3) {
            return;
        }
        ordered_slots.push_back(string_idx);
        cursor_x = dropoff_flat_[static_cast<size_t>(dropoff_idx) * 3];
        cursor_y = dropoff_flat_[static_cast<size_t>(dropoff_idx) * 3 + 1];
        cursor_valid = true;
    };

    while (!remaining_slots.empty()) {
        auto next_it = remaining_slots.begin();
        if (cursor_valid) {
            const double y_weight = pickup_greedy_y_distance_weight_;
            auto weighted_dist_sq = [&](double px, double py) {
                const double dx = px - cursor_x;
                const double dy = py - cursor_y;
                return dx * dx + (y_weight * dy) * (y_weight * dy);
            };
            next_it = std::min_element(
                remaining_slots.begin(), remaining_slots.end(),
                [&](size_t a, size_t b) {
                    const size_t pickup_a = pickup_index_for_eightboxes_slot(a);
                    const size_t pickup_b = pickup_index_for_eightboxes_slot(b);
                    const double ax = pickup_flat_[pickup_a * 4];
                    const double ay = pickup_flat_[pickup_a * 4 + 1];
                    const double bx = pickup_flat_[pickup_b * 4];
                    const double by = pickup_flat_[pickup_b * 4 + 1];
                    return weighted_dist_sq(ax, ay) < weighted_dist_sq(bx, by);
                });
        }
        const size_t string_idx = *next_it;
        remaining_slots.erase(next_it);
        append_ordered_slot(string_idx);
    }

    std::array<int, 4> dropoff_occurrence_by_class{};
    for (size_t string_idx : ordered_slots) {
        const int cargo_class = classes[string_idx];
        const int layer1_slot = class_to_layer1_slot_[static_cast<size_t>(cargo_class)];
        const int occurrence =
            ++dropoff_occurrence_by_class[static_cast<size_t>(cargo_class)];
        const int dropoff_idx =
            layer1_slot + (occurrence - 1) * dropoff_layer_stride_;
        if (dropoff_idx < 0 ||
            static_cast<size_t>(dropoff_idx) >= dropoff_flat_.size() / 3) {
            continue;
        }
        const size_t physical_pickup_idx = pickup_index_for_eightboxes_slot(string_idx);
        tasks.push_back(PickupDropTask{
            string_idx,
            physical_pickup_idx,
            static_cast<size_t>(dropoff_idx),
            cargo_class,
            occurrence
        });
    }
    return tasks;
}

void LidarNavControl::try_complete_pre_scan_handshake() {
    if (pre_scan_event_sent_ || !pre_scan_pause_after_done_) {
        return;
    }
    if (!pre_scan_spin_done_) {
        return;
    }
    if (!entry_trajectory_ready()) {
        return;
    }
    if (!path_generated_) {
        return;
    }

    publish_logistics_event(LogisticsEventType::PRE_SCAN_DONE, current_wp_idx_);
    pre_scan_event_sent_ = true;
    pre_scan_waiting_resume_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Path ready after OCR/eightboxes wait. Published PRE_SCAN_DONE; waiting for arm lift/resume.");
}

void LidarNavControl::arm_ocr_wait_after_eightboxes() {
    ocr_wait_t0_ = this->get_clock()->now();
    ocr_wait_armed_ = true;
}

void LidarNavControl::clear_ocr_wait_after_eightboxes() {
    ocr_wait_armed_ = false;
}

void LidarNavControl::maybe_generate_path_without_ocr_on_timeout() {
    if (path_generated_ || !ocr_wait_armed_ || ocr_result_locked_) {
        return;
    }
    if (!pending_priorities_valid_ || pending_raw_classes_.size() != 8) {
        return;
    }
    const double elapsed = (this->get_clock()->now() - ocr_wait_t0_).seconds();
    if (elapsed < ocr_wait_after_eightboxes_sec_) {
        return;
    }
    RCLCPP_WARN(this->get_logger(),
                "No OCR within %.1f s after eightboxes; generating left-to-right path.",
                ocr_wait_after_eightboxes_sec_);
    try_generate_path_with_pending_priorities(true);
}

void LidarNavControl::try_generate_path_with_pending_priorities(bool allow_without_ocr) {
    if (path_generated_) {
        RCLCPP_INFO(this->get_logger(), "Path already generated. Skip pending priorities.");
        return;
    }
    if (pre_scan_pause_after_done_ &&
        pre_path_spin_enable_ &&
        !pre_scan_spin_done_) {
        RCLCPP_INFO(this->get_logger(), "Priority path ready, waiting for pre-scan to finish.");
        return;
    }
    if (pre_scan_pause_after_done_ && !pre_scan_entry_point_ready()) {
        RCLCPP_INFO(this->get_logger(), "Priority path ready, waiting for pre-scan entry point.");
        return;
    }
    if (!ocr_result_locked_ && !allow_without_ocr) {
        RCLCPP_INFO(this->get_logger(),
                    "No OCR lock yet. Waiting up to %.1f s after eightboxes.",
                    ocr_wait_after_eightboxes_sec_);
        return;
    }
    if (!pending_priorities_valid_ || pending_raw_classes_.size() != 8) {
        self_path_valid_ = true;
        RCLCPP_INFO(this->get_logger(), "Pending priorities not valid or size not 8. Ignoring.");
        return;
    }

    std::string classes_str = "eightboxes classes = {";
    for (size_t idx = 0; idx < pending_raw_classes_.size(); ++idx) {
        classes_str += std::to_string(pending_raw_classes_[idx]);
        if (idx + 1 < pending_raw_classes_.size()) {
            classes_str += ", ";
        }
    }
    classes_str += "}";
    RCLCPP_INFO(this->get_logger(), "%s", classes_str.c_str());

    generate_path(pending_raw_classes_);

    if (path_generated_) {
        pending_priorities_valid_ = false;
        clear_ocr_wait_after_eightboxes();
    }
}

namespace {

bool points_near(double x1, double y1, double x2, double y2, double tolerance = 0.01) {
    return std::hypot(x1 - x2, y1 - y2) < tolerance;
}

void append_transition_node(std::vector<GraphPoint>& nodes, const std::vector<double>& transition) {
    if (transition.size() >= 3) {
        nodes.push_back({transition[0], transition[1], transition[2]});
    }
}

double perpendicular_distance(
    const GraphPoint& point, const GraphPoint& line_start, const GraphPoint& line_end)
{
    const double dx = line_end.x - line_start.x;
    const double dy = line_end.y - line_start.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) {
        return std::hypot(point.x - line_start.x, point.y - line_start.y);
    }
    return std::abs(dx * (line_start.y - point.y) - dy * (line_start.x - point.x)) / len;
}

void collect_decimated_indices(
    const std::vector<GraphPoint>& path,
    size_t first,
    size_t last,
    double epsilon,
    std::set<size_t>& keep)
{
    if (last <= first) {
        return;
    }
    if (last <= first + 1) {
        keep.insert(first);
        keep.insert(last);
        return;
    }

    double max_dist = 0.0;
    size_t farthest = first;
    for (size_t i = first + 1; i < last; ++i) {
        const double dist = perpendicular_distance(path[i], path[first], path[last]);
        if (dist > max_dist) {
            max_dist = dist;
            farthest = i;
        }
    }

    if (max_dist > epsilon) {
        keep.insert(farthest);
        collect_decimated_indices(path, first, farthest, epsilon, keep);
        collect_decimated_indices(path, farthest, last, epsilon, keep);
    } else {
        keep.insert(first);
        keep.insert(last);
    }
}

std::vector<size_t> decimate_transition_indices(
    const std::vector<GraphPoint>& path,
    size_t begin,
    size_t end_exclusive,
    double tol_x,
    double tol_y,
    bool cross_isolation_band,
    const lidar_nav::IsolationBandRegion* region)
{
    std::vector<size_t> empty;
    if (end_exclusive <= begin) {
        return empty;
    }
    if (end_exclusive - begin == 1) {
        return {begin};
    }

    if (cross_isolation_band) {
        std::vector<size_t> all;
        all.reserve(end_exclusive - begin);
        for (size_t i = begin; i < end_exclusive; ++i) {
            all.push_back(i);
        }
        return all;
    }

    const size_t last = end_exclusive - 1;
    const double epsilon = std::min(tol_x, tol_y);

    std::set<size_t> keep;
    collect_decimated_indices(path, begin, last, epsilon, keep);

    for (size_t i = begin + 1; i < end_exclusive; ++i) {
        if (region != nullptr && !region->empty()) {
            const bool prev_in = region->contains(path[i - 1].x, path[i - 1].y);
            const bool curr_in = region->contains(path[i].x, path[i].y);
            if (prev_in != curr_in) {
                keep.insert(i);
            }
        }
    }
    keep.insert(begin);
    keep.insert(last);

    return {keep.begin(), keep.end()};
}

}  // namespace

void LidarNavControl::init_astar_graph() {
    astar_planner_.set_neighbor_radius(astar_neighbor_radius_);
    astar_planner_.set_isolation_y_range(isolation_band_y_min_, isolation_band_y_max_);

    std::string add_point_path = add_point_file_;
    if (add_point_path.empty()) {
        try {
            add_point_path =
                ament_index_cpp::get_package_share_directory("lidar_nav_demo_cpp") +
                "/config/add_point.txt";
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to resolve add_point_file: %s", ex.what());
            return;
        }
    }

    std::vector<GraphPoint> nodes;
    std::vector<GraphPoint> add_point_nodes;
    if (!astar_planner_.load_add_point_file(add_point_path)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to load add_point_file: %s", add_point_path.c_str());
    } else {
        add_point_nodes = astar_planner_.static_nodes();
        nodes = add_point_nodes;
    }

    isolation_band_region_.configure(
        isolation_band_y_min_, isolation_band_y_max_,
        astar_neighbor_radius_, astar_neighbor_radius_);
    isolation_band_region_.build_from_add_points(add_point_nodes);

    append_transition_node(nodes, trans_front_);
    append_transition_node(nodes, trans_back_);
    append_transition_node(nodes, trans_mid_);
    astar_planner_.load_static_nodes(nodes);

    std::string viz_dir = path_viz_dir_;
    if (viz_dir.empty()) {
        try {
            viz_dir =
                ament_index_cpp::get_package_share_directory("lidar_nav_demo_cpp") +
                "/config/path_viz";
        } catch (const std::exception& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to resolve path_viz_dir, fallback to /tmp/lidar_nav_path_viz: %s",
                        ex.what());
            viz_dir = "/tmp/lidar_nav_path_viz";
        }
    }
    std::error_code ec;
    std::filesystem::create_directories(viz_dir, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to create path_viz_dir '%s': %s",
                    viz_dir.c_str(), ec.message().c_str());
    }
    path_viz_dir_ = viz_dir;

    std::vector<GraphPoint> logistics_overlay;
    for (size_t i = 0; i + 3 < pickup_flat_.size(); i += 4) {
        logistics_overlay.push_back(
            {pickup_flat_[i], pickup_flat_[i + 1], pickup_flat_[i + 2]});
    }
    for (size_t i = 0; i + 2 < dropoff_flat_.size(); i += 3) {
        logistics_overlay.push_back(
            {dropoff_flat_[i], dropoff_flat_[i + 1], dropoff_flat_[i + 2]});
    }
    astar_planner_.set_logistics_waypoints_overlay(logistics_overlay);

    RCLCPP_INFO(this->get_logger(),
                "A* graph ready: %zu nodes, %zu pickup/dropoff overlay, "
                "neighbor_radius=%.2f, isolation_band_nodes=%zu (y=[%.2f,%.2f]), "
                "add_point=%s, viz_dir=%s",
                nodes.size(), logistics_overlay.size(),
                astar_neighbor_radius_, isolation_band_region_.node_count(),
                isolation_band_y_min_, isolation_band_y_max_,
                add_point_path.c_str(), path_viz_dir_.c_str());
}

void LidarNavControl::update_path_viz_slots(
    const std::vector<int>& slot_classes,
    const std::vector<PickupDropTask>& tasks)
{
    std::vector<PathVizPickupSlot> pickup_slots;
    std::vector<PathVizDropoffSlot> dropoff_slots;

    const size_t priority_slot =
        tasks.empty() ? 8 : tasks.front().eightboxes_string_idx;

    if (slot_classes.size() == 8) {
        for (size_t string_idx = 0; string_idx < 8; ++string_idx) {
            const size_t phys = pickup_index_for_eightboxes_slot(string_idx);
            if (phys >= pickup_flat_.size() / 4) {
                continue;
            }
            const int cargo_class = slot_classes[string_idx];
            if (cargo_class < 0 || cargo_class > 3) {
                continue;
            }
            pickup_slots.push_back(PathVizPickupSlot{
                pickup_flat_[phys * 4],
                pickup_flat_[phys * 4 + 1],
                static_cast<int>(string_idx),
                cargo_class,
                string_idx == priority_slot});
        }
    }

    const size_t num_dropoff = dropoff_flat_.size() / 3;
    for (size_t drop_idx = 0; drop_idx < num_dropoff; ++drop_idx) {
        const int layer = (drop_idx >= static_cast<size_t>(dropoff_layer_stride_)) ? 1 : 0;
        const int layer1_slot = static_cast<int>(
            drop_idx % static_cast<size_t>(dropoff_layer_stride_));
        int cargo_class = -1;
        if (layer1_slot >= 0 &&
            static_cast<size_t>(layer1_slot) < dropoff_color_order_.size()) {
            cargo_class = dropoff_color_order_[static_cast<size_t>(layer1_slot)];
        }
        if (cargo_class < 0 || cargo_class > 3) {
            continue;
        }
        dropoff_slots.push_back(PathVizDropoffSlot{
            dropoff_flat_[drop_idx * 3],
            dropoff_flat_[drop_idx * 3 + 1],
            static_cast<int>(drop_idx),
            cargo_class,
            layer});
    }

    astar_planner_.set_path_viz_slots(pickup_slots, dropoff_slots);
}

void LidarNavControl::write_pickup_order_viz(
    const std::vector<int>& slot_classes,
    const std::vector<PickupDropTask>& tasks) const
{
    if (tasks.empty() || path_viz_dir_.empty()) {
        return;
    }

    std::error_code ec;
    std::filesystem::create_directories(path_viz_dir_, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to create path_viz_dir '%s': %s",
                    path_viz_dir_.c_str(), ec.message().c_str());
        return;
    }

    constexpr double kWidth = 900.0;
    constexpr double kHeight = 720.0;
    constexpr double kMargin = 55.0;
    constexpr double kXMin = -5.0;
    constexpr double kXMax = 5.0;
    constexpr double kYMin = 0.0;
    constexpr double kYMax = 9.0;

    auto to_svg = [&](double x, double y) {
        const double sx = kMargin + (x - kXMin) / (kXMax - kXMin) * (kWidth - 2.0 * kMargin);
        const double sy = kHeight - kMargin - (y - kYMin) / (kYMax - kYMin) * (kHeight - 2.0 * kMargin);
        return std::pair<double, double>{sx, sy};
    };
    auto class_color = [](int cargo_class) {
        switch (cargo_class) {
            case 0: return "#2ca02c";
            case 1: return "#888888";
            case 2: return "#1f77b4";
            case 3: return "#d62728";
            default: return "#666666";
        }
    };

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << kWidth
        << "\" height=\"" << kHeight << "\" viewBox=\"0 0 " << kWidth << " " << kHeight << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#f8f8f8\"/>\n";
    svg << "<text x=\"18\" y=\"28\" font-size=\"16\" fill=\"#222\">pickup_order.svg</text>\n";
    svg << "<text x=\"18\" y=\"48\" font-size=\"12\" fill=\"#555\">number = actual pickup/dropoff order</text>\n";

    for (double gx = std::ceil(kXMin); gx <= std::floor(kXMax); gx += 1.0) {
        const auto a = to_svg(gx, kYMin);
        const auto b = to_svg(gx, kYMax);
        svg << "<line x1=\"" << a.first << "\" y1=\"" << a.second
            << "\" x2=\"" << b.first << "\" y2=\"" << b.second
            << "\" stroke=\"#dddddd\" stroke-width=\"1\"/>\n";
    }
    for (double gy = std::ceil(kYMin); gy <= std::floor(kYMax); gy += 1.0) {
        const auto a = to_svg(kXMin, gy);
        const auto b = to_svg(kXMax, gy);
        svg << "<line x1=\"" << a.first << "\" y1=\"" << a.second
            << "\" x2=\"" << b.first << "\" y2=\"" << b.second
            << "\" stroke=\"#dddddd\" stroke-width=\"1\"/>\n";
    }

    if (slot_classes.size() == 8) {
        for (size_t string_idx = 0; string_idx < 8; ++string_idx) {
            const size_t pickup_idx = pickup_index_for_eightboxes_slot(string_idx);
            if (pickup_idx >= pickup_flat_.size() / 4) {
                continue;
            }
            const auto p = to_svg(pickup_flat_[pickup_idx * 4], pickup_flat_[pickup_idx * 4 + 1]);
            svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
                << "\" r=\"7\" fill=\"#ffffff\" stroke=\"#999999\" stroke-width=\"1\"/>\n";
            svg << "<text x=\"" << (p.first + 9) << "\" y=\"" << (p.second - 8)
                << "\" font-size=\"10\" fill=\"#777\">P" << pickup_idx << "/S" << string_idx << "</text>\n";
        }
    }

    svg << "<polyline fill=\"none\" stroke=\"#222222\" stroke-width=\"2.5\" stroke-dasharray=\"5,5\" points=\"";
    for (const auto& task : tasks) {
        if (task.pickup_idx >= pickup_flat_.size() / 4) {
            continue;
        }
        const auto p = to_svg(pickup_flat_[task.pickup_idx * 4], pickup_flat_[task.pickup_idx * 4 + 1]);
        svg << p.first << "," << p.second << " ";
    }
    svg << "\"/>\n";

    for (size_t order = 0; order < tasks.size(); ++order) {
        const auto& task = tasks[order];
        if (task.pickup_idx >= pickup_flat_.size() / 4) {
            continue;
        }
        const int cargo_class =
            task.eightboxes_string_idx < slot_classes.size()
                ? slot_classes[task.eightboxes_string_idx]
                : task.cargo_class;
        const auto p = to_svg(pickup_flat_[task.pickup_idx * 4], pickup_flat_[task.pickup_idx * 4 + 1]);
        svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
            << "\" r=\"15\" fill=\"" << class_color(cargo_class)
            << "\" stroke=\"#111111\" stroke-width=\"2\"/>\n";
        svg << "<text x=\"" << p.first << "\" y=\"" << (p.second + 5)
            << "\" text-anchor=\"middle\" font-size=\"14\" font-weight=\"700\" fill=\"#ffffff\">"
            << (order + 1) << "</text>\n";
        svg << "<text x=\"" << (p.first + 18) << "\" y=\"" << (p.second + 18)
            << "\" font-size=\"11\" fill=\"#222\">P" << task.pickup_idx
            << " -> D" << task.dropoff_idx << " "
            << cargo_color_name(cargo_class) << "</text>\n";
    }

    svg << "</svg>\n";

    const std::string svg_path = path_viz_dir_ + "/pickup_order.svg";
    std::ofstream out(svg_path);
    if (!out) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to write pickup order viz: %s",
                    svg_path.c_str());
        return;
    }
    out << svg.str();
    RCLCPP_INFO(this->get_logger(),
                "Pickup order viz saved: %s",
                svg_path.c_str());
}

bool LidarNavControl::find_pickup_staging_point(
    double pickup_x, double pickup_y, bool is_rear_row,
    double& sx, double& sy, double& syaw) const
{
    const auto& nodes = astar_planner_.static_nodes();
    size_t best_idx = nodes.size();
    double best_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < nodes.size(); ++i) {
        const bool y_ok = is_rear_row
            ? (nodes[i].y > pickup_rear_staging_y_min_)
            : (nodes[i].y <= pickup_front_staging_y_max_);
        if (!y_ok) {
            continue;
        }
        const double d = std::hypot(pickup_x - nodes[i].x, pickup_y - nodes[i].y);
        if (d < best_dist) {
            best_dist = d;
            best_idx = i;
        }
    }

    if (best_idx >= nodes.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No staging graph node for %s pickup (%.3f, %.3f), y %s %.2f",
                     is_rear_row ? "rear" : "front", pickup_x, pickup_y,
                     is_rear_row ? ">" : "<=",
                     is_rear_row ? pickup_rear_staging_y_min_ : pickup_front_staging_y_max_);
        return false;
    }

    sx = nodes[best_idx].x;
    sy = nodes[best_idx].y;
    syaw = nodes[best_idx].yaw;
    RCLCPP_INFO(this->get_logger(),
                "Pickup staging (%s): (%.3f, %.3f) -> nearest staging (%.3f, %.3f), dist=%.3fm",
                is_rear_row ? "rear" : "front",
                pickup_x, pickup_y, sx, sy, best_dist);
    return true;
}

bool LidarNavControl::append_direct_nearest_segment(
    double from_x, double from_y,
    double to_x, double to_y, double to_yaw,
    WaypointType final_type, int action_id,
    const std::string& viz_tag,
    bool yaw_align_at_nearest)
{
    const auto& nodes = astar_planner_.static_nodes();
    if (nodes.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No graph nodes loaded for segment '%s'", viz_tag.c_str());
        return false;
    }

    const size_t nearest_idx = astar_planner_.nearest_static_node(to_x, to_y);
    const double nx = nodes[nearest_idx].x;
    const double ny = nodes[nearest_idx].y;

    std::vector<GraphPoint> viz_path;
    viz_path.push_back({from_x, from_y, 0.0});
    viz_path.push_back({nx, ny, nodes[nearest_idx].yaw});
    viz_path.push_back({to_x, to_y, to_yaw});
    const std::string svg_path = path_viz_dir_ + "/segment_" + viz_tag + ".svg";
    astar_planner_.visualize_segment(viz_path, svg_path, -5.0, 5.0, 0.0, 9.0);
    RCLCPP_INFO(this->get_logger(),
                "Direct segment '%s': nearest node (%.3f, %.3f) -> target (%.3f, %.3f), saved %s",
                viz_tag.c_str(), nx, ny, to_x, to_y, svg_path.c_str());

    constexpr double kApproachSkipDist = 0.15;
    const bool from_far_from_nearest =
        std::hypot(from_x - nx, from_y - ny) > kApproachSkipDist;
    const bool need_approach_wp =
        from_far_from_nearest &&
        (waypoints_.empty() ||
         !points_near(waypoints_.back().x, waypoints_.back().y, nx, ny));

    if (need_approach_wp) {
        const bool wp_cross_band = isolation_band_region_.contains(nx, ny);
        const bool tight_lateral_zone =
            wp_cross_band || lidar_nav::is_in_tight_lateral_y_zone(ny);
        TargetPoint approach{};
        approach.x = nx;
        approach.y = ny;
        approach.yaw = to_yaw;
        approach.type = WaypointType::TRANSITION;
        approach.cross_isolation_band = wp_cross_band;
        approach.tight_lateral_zone = tight_lateral_zone;
        approach.direct_nearest_approach = true;
        approach.graph_yaw_align_after_arrival = yaw_align_at_nearest;
        waypoints_.push_back(approach);
        const char* final_name = "TRANSITION";
        if (final_type == WaypointType::PICKUP) {
            final_name = "PICKUP";
        } else if (final_type == WaypointType::DROPOFF) {
            final_name = "DROPOFF";
        }
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': direct nearest approach (%.3f, %.3f) then %s",
                    viz_tag.c_str(), nx, ny, final_name);
    }

    const bool is_manip_segment =
        final_type == WaypointType::PICKUP || final_type == WaypointType::DROPOFF;
    if (is_manip_segment) {
        if (!waypoints_.empty() &&
            points_near(waypoints_.back().x, waypoints_.back().y, to_x, to_y)) {
            RCLCPP_WARN(this->get_logger(),
                        "Segment '%s': nearest node near manip target, still appending exact %s "
                        "at (%.3f, %.3f, %.3f)",
                        viz_tag.c_str(),
                        final_type == WaypointType::PICKUP ? "PICKUP" : "DROPOFF",
                        to_x, to_y, to_yaw);
        }
        waypoints_.push_back({to_x, to_y, to_yaw, final_type, action_id});
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': manip waypoint at (%.3f, %.3f, %.3f yaw), action_id=%d",
                    viz_tag.c_str(), to_x, to_y, to_yaw, action_id);
    } else if (waypoints_.empty() ||
               !points_near(waypoints_.back().x, waypoints_.back().y, to_x, to_y)) {
        const bool end_cross_band = isolation_band_region_.contains(to_x, to_y);
        const bool tight_lateral_zone =
            end_cross_band || lidar_nav::is_in_tight_lateral_y_zone(to_y);
        waypoints_.push_back(
            {to_x, to_y, to_yaw, WaypointType::TRANSITION, -1,
             end_cross_band, tight_lateral_zone});
    }
    return true;
}

bool LidarNavControl::append_dropoff_retract_waypoint(
    double manip_x, double manip_y,
    double goal_yaw,
    const std::string& viz_tag)
{
    const auto& nodes = astar_planner_.static_nodes();
    if (nodes.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No graph nodes for dropoff retract '%s'", viz_tag.c_str());
        return false;
    }

    const size_t nearest_idx = astar_planner_.nearest_static_node(manip_x, manip_y);
    const double nx = nodes[nearest_idx].x;
    const double ny = nodes[nearest_idx].y;

    constexpr double kApproachSkipDist = 0.15;
    const double manip_to_nearest = std::hypot(manip_x - nx, manip_y - ny);
    if (manip_to_nearest <= kApproachSkipDist) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': dropoff (%.3f, %.3f) already near graph node (%.3f, %.3f), "
                    "skip retract",
                    viz_tag.c_str(), manip_x, manip_y, nx, ny);
        return true;
    }
    if (!waypoints_.empty() &&
        points_near(waypoints_.back().x, waypoints_.back().y, nx, ny)) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': already at dropoff nearest (%.3f, %.3f), skip retract",
                    viz_tag.c_str(), nx, ny);
        return true;
    }

    const bool wp_cross_band = isolation_band_region_.contains(nx, ny);
    const bool tight_lateral_zone =
        wp_cross_band || lidar_nav::is_in_tight_lateral_y_zone(ny);
    TargetPoint retract{};
    retract.x = nx;
    retract.y = ny;
    retract.yaw = goal_yaw;
    retract.type = WaypointType::TRANSITION;
    retract.cross_isolation_band = wp_cross_band;
    retract.tight_lateral_zone = tight_lateral_zone;
    retract.manip_to_nearest_arrival = true;
    retract.graph_yaw_align_after_arrival = false;
    waypoints_.push_back(retract);
    RCLCPP_INFO(this->get_logger(),
                "Segment '%s': dropoff retract heading_dock to (%.3f, %.3f, %.3f yaw), dist=%.3fm",
                viz_tag.c_str(), nx, ny, goal_yaw, manip_to_nearest);
    return true;
}

bool LidarNavControl::append_direct_manip_travel(
    double to_x, double to_y, double to_yaw,
    WaypointType final_type, int action_id,
    const std::string& viz_tag)
{
    if (!waypoints_.empty() &&
        points_near(waypoints_.back().x, waypoints_.back().y, to_x, to_y) &&
        waypoints_.back().type == final_type) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': already at target (%.3f, %.3f), skip direct travel",
                    viz_tag.c_str(), to_x, to_y);
        return true;
    }

    const bool end_cross_band = isolation_band_region_.contains(to_x, to_y);
    const bool tight_lateral_zone =
        end_cross_band || lidar_nav::is_in_tight_lateral_y_zone(to_y);
    TargetPoint dest{};
    dest.x = to_x;
    dest.y = to_y;
    dest.yaw = to_yaw;
    dest.type = final_type;
    dest.action_id = action_id;
    dest.cross_isolation_band = end_cross_band;
    dest.tight_lateral_zone = tight_lateral_zone;
    dest.direct_nearest_approach = true;
    dest.graph_yaw_align_after_arrival = false;
    waypoints_.push_back(dest);

    const char* type_name = "TRANSITION";
    if (final_type == WaypointType::PICKUP) {
        type_name = "PICKUP";
    } else if (final_type == WaypointType::DROPOFF) {
        type_name = "DROPOFF";
    }
    RCLCPP_INFO(this->get_logger(),
                "Segment '%s': direct FORWARD_ONLY travel to %s (%.3f, %.3f, %.3f yaw)",
                viz_tag.c_str(), type_name, to_x, to_y, to_yaw);
    return true;
}

bool LidarNavControl::append_astar_segment(
    double from_x, double from_y,
    double to_x, double to_y, double to_yaw,
    WaypointType final_type, int action_id,
    const std::string& viz_tag)
{
    const auto segment_path = astar_planner_.find_shortest_path(from_x, from_y, to_x, to_y);
    if (segment_path.empty()) {
        const auto connectivity = astar_planner_.check_connectivity(from_x, from_y, to_x, to_y);
        RCLCPP_ERROR(this->get_logger(),
                     "A* failed for segment '%s': (%.3f, %.3f) -> (%.3f, %.3f), "
                     "start_connected=%s (nearest %.3fm, comp=%zu), "
                     "goal_connected=%s (nearest %.3fm, comp=%zu), "
                     "graph_path_exists=%s, radius=%.2fm",
                     viz_tag.c_str(), from_x, from_y, to_x, to_y,
                     connectivity.start_connected ? "true" : "false",
                     connectivity.start_nearest_dist,
                     connectivity.start_component_size,
                     connectivity.goal_connected ? "true" : "false",
                     connectivity.goal_nearest_dist,
                     connectivity.goal_component_size,
                     connectivity.graph_path_exists ? "true" : "false",
                     astar_planner_.neighbor_radius());
        return false;
    }

    const std::string svg_path = path_viz_dir_ + "/segment_" + viz_tag + ".svg";
    astar_planner_.visualize_segment(segment_path, svg_path, -5.0, 5.0, 0.0, 9.0);
    RCLCPP_INFO(this->get_logger(),
                "A* segment '%s': %zu points, saved %s",
                viz_tag.c_str(), segment_path.size(), svg_path.c_str());

    size_t start_idx = 0;
    if (!waypoints_.empty() &&
        points_near(waypoints_.back().x, waypoints_.back().y,
                    segment_path.front().x, segment_path.front().y)) {
        start_idx = 1;
    }

    const bool is_manip_segment =
        final_type == WaypointType::PICKUP || final_type == WaypointType::DROPOFF;

    const bool cross_isolation_band =
        isolation_band_region_.path_intersects(segment_path, start_idx, segment_path.size()) ||
        isolation_band_region_.contains(from_x, from_y) ||
        isolation_band_region_.contains(to_x, to_y);

    const size_t intermediate_end =
        (is_manip_segment && !cross_isolation_band)
            ? (segment_path.size() > start_idx ? segment_path.size() - 1 : start_idx)
            : segment_path.size();
    const bool segment_tight_lateral_y =
        lidar_nav::is_in_tight_lateral_y_zone(from_y) ||
        lidar_nav::is_in_tight_lateral_y_zone(to_y);
    if (cross_isolation_band) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': cross-isolation-band path (%.3f,%.3f)->(%.3f,%.3f), "
                    "using tight lateral limits",
                    viz_tag.c_str(), from_x, from_y, to_x, to_y);
    } else if (segment_tight_lateral_y) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': y in [%.1f,%.1f] zone (%.3f,%.3f)->(%.3f,%.3f), "
                    "using tight lateral limits",
                    viz_tag.c_str(),
                    lidar_nav::kTightLateralYMin, lidar_nav::kTightLateralYMax,
                    from_x, from_y, to_x, to_y);
    }

    const bool from_manip =
        !waypoints_.empty() &&
        (waypoints_.back().type == WaypointType::PICKUP ||
         waypoints_.back().type == WaypointType::DROPOFF);

    const auto kept_indices = decimate_transition_indices(
        segment_path, start_idx, intermediate_end,
        transition_position_tolerance_x_, transition_position_tolerance_y_,
        cross_isolation_band, &isolation_band_region_);
    if (kept_indices.size() < intermediate_end - start_idx) {
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': decimated TRANSITION waypoints %zu -> %zu",
                    viz_tag.c_str(), intermediate_end - start_idx, kept_indices.size());
    }
    for (size_t k = 0; k < kept_indices.size(); ++k) {
        const size_t idx = kept_indices[k];
        if (!waypoints_.empty() &&
            points_near(waypoints_.back().x, waypoints_.back().y,
                        segment_path[idx].x, segment_path[idx].y)) {
            continue;
        }
        const bool wp_cross_band =
            isolation_band_region_.contains(segment_path[idx].x, segment_path[idx].y);
        const bool tight_lateral_zone =
            wp_cross_band ||
            lidar_nav::is_in_tight_lateral_y_zone(segment_path[idx].y);
        const bool corridor_lateral_only =
            cross_isolation_band &&
            lidar_nav::corridor_segment_lateral_only_arrival(
                wp_cross_band,
                segment_path[idx].y,
                k == 0,
                k == kept_indices.size() - 1,
                pickup_rear_staging_y_min_);
        const bool manip_approach_arrival =
            is_manip_segment && (k + 1 == kept_indices.size());
        const bool post_manip_exit = from_manip && k == 0;
        // 仅取/放货前最后一个图点（manip_approach）使用目标 yaw；其余中继 TRANSITION
        // 忽略 add_point 中的 yaw，配合 FORWARD_ONLY 只做纵向+转向追点。
        const double wp_yaw = manip_approach_arrival ? to_yaw : 0.0;
        const bool graph_yaw_align_after_arrival = false;
        const WaypointType post_manip_source =
            post_manip_exit ? waypoints_.back().type : WaypointType::TRANSITION;
        waypoints_.push_back(
            {segment_path[idx].x, segment_path[idx].y, wp_yaw,
             WaypointType::TRANSITION, -1, wp_cross_band, tight_lateral_zone,
             corridor_lateral_only, manip_approach_arrival, post_manip_exit,
             graph_yaw_align_after_arrival});
        if (post_manip_exit) {
            RCLCPP_INFO(this->get_logger(),
                        "Segment '%s': post-%s exit TRANSITION at (%.3f, %.3f), "
                        "using heading_dock (no FORWARD_ONLY / yaw-first)",
                        viz_tag.c_str(),
                        post_manip_source == WaypointType::PICKUP ? "PICKUP" : "DROPOFF",
                        segment_path[idx].x, segment_path[idx].y);
        }
        if (manip_approach_arrival) {
            RCLCPP_INFO(this->get_logger(),
                        "Segment '%s': manip_approach TRANSITION at (%.3f, %.3f, %.3f yaw), "
                        "heading_dock with %s yaw",
                        viz_tag.c_str(),
                        segment_path[idx].x, segment_path[idx].y, wp_yaw,
                        final_type == WaypointType::PICKUP ? "PICKUP" : "DROPOFF");
        }
    }

    if (cross_isolation_band && is_manip_segment && !kept_indices.empty()) {
        const size_t last_idx = kept_indices.back();
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': corridor exit TRANSITION at (%.3f, %.3f), "
                    "lateral_only=%s manip_approach=%s, then %s",
                    viz_tag.c_str(),
                    segment_path[last_idx].x, segment_path[last_idx].y,
                    waypoints_.back().corridor_lateral_only_arrival ? "true" : "false",
                    waypoints_.back().manip_approach_arrival ? "true" : "false",
                    final_type == WaypointType::PICKUP ? "PICKUP" : "DROPOFF");
    }

    if (is_manip_segment) {
        if (!waypoints_.empty() &&
            points_near(waypoints_.back().x, waypoints_.back().y, to_x, to_y)) {
            RCLCPP_WARN(this->get_logger(),
                        "Segment '%s': last graph node near manip target, still appending exact %s "
                        "at (%.3f, %.3f, %.3f)",
                        viz_tag.c_str(),
                        final_type == WaypointType::PICKUP ? "PICKUP" : "DROPOFF",
                        to_x, to_y, to_yaw);
        }
        waypoints_.push_back({to_x, to_y, to_yaw, final_type, action_id});
        RCLCPP_INFO(this->get_logger(),
                    "Segment '%s': manip waypoint at (%.3f, %.3f, %.3f yaw), action_id=%d",
                    viz_tag.c_str(), to_x, to_y, to_yaw, action_id);
    } else if (waypoints_.empty() ||
               !points_near(waypoints_.back().x, waypoints_.back().y, to_x, to_y)) {
        const bool end_cross_band = isolation_band_region_.contains(to_x, to_y);
        const bool tight_lateral_zone =
            end_cross_band || lidar_nav::is_in_tight_lateral_y_zone(to_y);
        waypoints_.push_back(
            {to_x, to_y, to_yaw, WaypointType::TRANSITION, -1,
             end_cross_band, tight_lateral_zone});
    }
    return true;
}

void LidarNavControl::publish_logistics_event(LogisticsEventType event_type, size_t waypoint_index) {
    std_msgs::msg::Int32MultiArray msg;
    int action_id = -1;
    if (waypoint_index < waypoints_.size()) {
        action_id = waypoints_[waypoint_index].action_id;
    }
    msg.data = {
        static_cast<int>(event_type),
        static_cast<int>(waypoint_index),
        logistics_event_seq_++,
        action_id
    };
    logistics_event_pub_->publish(msg);
}

void LidarNavControl::generate_path(const std::vector<int>& slot_classes) {
    const size_t num_pickup = pickup_flat_.size() / 4;
    const size_t num_dropoff = dropoff_flat_.size() / 3;

    if (slot_classes.size() < 8) {
        RCLCPP_ERROR(this->get_logger(),
                     "Received slot_classes count (%zu) < 8. Path not generated.",
                     slot_classes.size());
        return;
    }
    if (num_pickup < 8 || num_dropoff < 8) {
        RCLCPP_ERROR(this->get_logger(),
                     "Need 8 pickup and 8 dropoff points, got pickup=%zu dropoff=%zu.",
                     num_pickup, num_dropoff);
        return;
    }

    const auto tasks = build_tasks(slot_classes);
    if (tasks.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No valid pickup/dropoff tasks built. Path not generated.");
        return;
    }

    update_path_viz_slots(slot_classes, tasks);
    write_pickup_order_viz(slot_classes, tasks);

    waypoints_.clear();
    startup_first_pickup_wp_valid_ = false;
    startup_first_pickup_wp_idx_ = 0;

    RCLCPP_INFO(this->get_logger(),
                "Path visit order: OCR ignored for pickup priority; using configured pickup order");

    for (size_t task_idx = 0; task_idx < tasks.size(); ++task_idx) {
        const auto& task = tasks[task_idx];
        RCLCPP_INFO(this->get_logger(),
                    "Task[%zu]: pickup[%zu] %s(%d) occ=%d -> dropoff[%zu]",
                    task_idx, task.pickup_idx,
                    cargo_color_name(task.cargo_class).c_str(), task.cargo_class,
                    task.occurrence, task.dropoff_idx);
    }

    double from_x = 0.0;
    double from_y = 0.0;
    if (entry_traj_loaded_ && !entry_traj_.empty()) {
        from_x = entry_traj_.back().x;
        from_y = entry_traj_.back().y;
        RCLCPP_INFO(this->get_logger(),
                    "Using entry_traj end (%.3f, %.3f) as path planning start",
                    from_x, from_y);
    } else if (got_tf_) {
        from_x = x_;
        from_y = y_;
    } else if (trans_front_.size() >= 2) {
        from_x = trans_front_[0];
        from_y = trans_front_[1];
        RCLCPP_WARN(this->get_logger(),
                    "No TF/entry_traj when generating path; using transition_front as start");
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "No entry_traj, TF pose, or transition_front for path start");
        path_generated_ = false;
        return;
    }

    for (size_t task_idx = 0; task_idx < tasks.size(); ++task_idx) {
        const auto& task = tasks[task_idx];
        const size_t i = task.pickup_idx;
        const size_t d_idx = task.dropoff_idx;

        const double pickup_x = pickup_flat_[i * 4];
        const double pickup_y = pickup_flat_[i * 4 + 1];
        const double pickup_yaw = pickup_flat_[i * 4 + 2];
        const double dropoff_x = dropoff_flat_[d_idx * 3];
        const double dropoff_y = dropoff_flat_[d_idx * 3 + 1];
        const double dropoff_yaw = dropoff_flat_[d_idx * 3 + 2];

        if (task_idx > 0) {
            from_x = waypoints_.back().x;
            from_y = waypoints_.back().y;
        }

        const std::string task_tag = "task" + std::to_string(task_idx);
        if (task_idx == 0) {
            const std::string pickup_tag = task_tag + "_to_pickup";
            if (!append_direct_manip_travel(pickup_x, pickup_y, pickup_yaw,
                                            WaypointType::PICKUP, -1, pickup_tag)) {
                path_generated_ = false;
                return;
            }
            if (startup_entry_replay_enabled() &&
                startup_pickup_index_ >= 0 &&
                static_cast<size_t>(startup_pickup_index_) == i &&
                !waypoints_.empty()) {
                startup_first_pickup_wp_valid_ = true;
                startup_first_pickup_wp_idx_ = waypoints_.size() - 1;
                RCLCPP_INFO(this->get_logger(),
                            "Startup pickup[%zu] mapped to WP[%zu]; entry trajectory replay armed after pickup.",
                            i, startup_first_pickup_wp_idx_);
            }
        }

        const std::string dropoff_tag = task_tag + "_to_dropoff";
        if (!append_direct_manip_travel(dropoff_x, dropoff_y, dropoff_yaw,
                                        WaypointType::DROPOFF, static_cast<int>(d_idx),
                                        dropoff_tag)) {
            path_generated_ = false;
            return;
        }

        double exit_goal_yaw = dropoff_yaw;
        if (task_idx + 1 < tasks.size()) {
            const size_t next_i = tasks[task_idx + 1].pickup_idx;
            exit_goal_yaw = pickup_flat_[next_i * 4 + 2];
        } else if (trans_front_.size() >= 3) {
            exit_goal_yaw = trans_front_[2];
        }

        const std::string retract_tag = task_tag + "_dropoff_retract";
        if (!append_dropoff_retract_waypoint(dropoff_x, dropoff_y, exit_goal_yaw,
                                             retract_tag)) {
            path_generated_ = false;
            return;
        }

        if (task_idx + 1 < tasks.size()) {
            const auto& next_task = tasks[task_idx + 1];
            const size_t next_i = next_task.pickup_idx;
            const double next_pickup_x = pickup_flat_[next_i * 4];
            const double next_pickup_y = pickup_flat_[next_i * 4 + 1];
            const double next_pickup_yaw = pickup_flat_[next_i * 4 + 2];
            const std::string exit_tag = task_tag + "_to_next_pickup";
            if (!append_direct_manip_travel(next_pickup_x, next_pickup_y, next_pickup_yaw,
                                             WaypointType::PICKUP, -1, exit_tag)) {
                path_generated_ = false;
                return;
            }
        } else if (trans_front_.size() >= 3) {
            const std::string exit_tag = task_tag + "_to_transition_front";
            if (!append_direct_manip_travel(trans_front_[0], trans_front_[1], trans_front_[2],
                                            WaypointType::TRANSITION, -1, exit_tag)) {
                path_generated_ = false;
                return;
            }
        }
    }

    path_generated_ = true;
    path_fallback_wait_armed_ = false;
    pre_path_spin_phase_ = PrePathSpinPhase::Idle;
    pre_path_spin_yaw_idx_ = 0;
    current_wp_idx_ = 0;

    // 跳过离当前位置太近的起始航路点，避免 bearing 在近距离数值不稳定
    if (got_tf_ && !waypoints_.empty()) {
        constexpr double kSkipCloseWpDist = 0.15;
        while (current_wp_idx_ < waypoints_.size()) {
            const auto& wp = waypoints_[current_wp_idx_];
            const double d = std::hypot(wp.x - x_, wp.y - y_);
            if (d > kSkipCloseWpDist) {
                break;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Skipping close WP[%zu] (%.3f, %.3f), dist=%.3f m",
                        current_wp_idx_, wp.x, wp.y, d);
            ++current_wp_idx_;
        }
        if (current_wp_idx_ >= waypoints_.size()) {
            RCLCPP_WARN(this->get_logger(),
                        "All waypoints too close after skip; keeping last WP.");
            current_wp_idx_ = waypoints_.size() - 1;
        }
    }

    reset_arrival_debounce();
    reset_graph_yaw_align();
    logistics_prev_ctrl_time_valid_ = false;
    if (logistics_controller_) {
        logistics_controller_->reset();
    }
    logistics_finished_sent_ = false;
    pre_scan_event_sent_ = false;
    pre_scan_waiting_resume_ = false;
    pre_scan_timeout_fallback_ = false;
    pre_scan_started_ = false;

    RCLCPP_INFO(this->get_logger(), "Path generated! %zu pickup tasks, total waypoints: %zu",
                tasks.size(), waypoints_.size());
    for (size_t wp_idx = 0; wp_idx < waypoints_.size(); ++wp_idx) {
        const auto& wp = waypoints_[wp_idx];
        const char* type_name = "TRANSITION";
        if (wp.type == WaypointType::PICKUP) {
            type_name = "PICKUP";
        } else if (wp.type == WaypointType::DROPOFF) {
            type_name = "DROPOFF";
        }
        RCLCPP_INFO(this->get_logger(),
                    "WP[%zu] %s (%.3f, %.3f, %.3f yaw) action_id=%d",
                    wp_idx, type_name, wp.x, wp.y, wp.yaw, wp.action_id);
    }
    publish_boxsuc_handshake();
    play_path_ready_audio();
}
