#ifndef LIDAR_NAV_DEMO_CPP_FIELD_LAYOUT_LOADER_HPP_
#define LIDAR_NAV_DEMO_CPP_FIELD_LAYOUT_LOADER_HPP_

#include <string>
#include <vector>

namespace lidar_nav {

struct GeneratedManipPoints {
    std::vector<double> pickup_flat;
    std::vector<double> dropoff_flat;
};

// Load pickup_points (32 values) and dropoff_points (24 values) from yaml file.
bool load_generated_manip_points(
    const std::string& path, GeneratedManipPoints& out, std::string& error);

}  // namespace lidar_nav

#endif  // LIDAR_NAV_DEMO_CPP_FIELD_LAYOUT_LOADER_HPP_
