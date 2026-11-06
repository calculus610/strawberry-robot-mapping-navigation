#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class HeightProjectionMap : public rclcpp::Node
{
public:
  HeightProjectionMap() : Node("height_projection_map")
  {
    const auto input = declare_parameter<std::string>("cloud_topic", "/Laser_map");
    const auto output = declare_parameter<std::string>("map_topic", "/map_height_projection");
    frame_ = declare_parameter<std::string>("cloud_frame", "camera_init");
    minimum_z_ = declare_parameter<double>("min_z", -0.2);
    maximum_z_ = declare_parameter<double>("max_z", 0.8);
    resolution_ = declare_parameter<double>("resolution", 0.05);
    maximum_cells_ = declare_parameter<int64_t>("max_grid_cells", 10000000);
    if (!std::isfinite(minimum_z_) || !std::isfinite(maximum_z_) ||
      minimum_z_ > maximum_z_ || !std::isfinite(resolution_) || resolution_ <= 0.0 ||
      maximum_cells_ <= 0 || frame_.empty())
    {
      throw std::invalid_argument("invalid height, resolution, grid size, or frame parameters");
    }
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output, rclcpp::QoS(1).reliable().transient_local());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {project(*cloud);});
    RCLCPP_INFO(
      get_logger(), "Cumulative cloud %s -> %s; frame=%s, Z=[%.3f, %.3f] m, grid=%.3f m. "
      "Height only: no ground/person/noise filtering; unmarked cells stay unknown.",
      input.c_str(), output.c_str(), frame_.c_str(), minimum_z_, maximum_z_, resolution_);
  }

private:
  void project(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (cloud.header.frame_id != frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected cumulative cloud frame '%s', received '%s'; no transform applied.",
        frame_.c_str(), cloud.header.frame_id.c_str());
      return;
    }
    const uint64_t point_count = uint64_t(cloud.width) * cloud.height;
    if (cloud.is_bigendian || cloud.point_step == 0 ||
      uint64_t(cloud.row_step) != uint64_t(cloud.width) * cloud.point_step ||
      point_count * cloud.point_step > cloud.data.size())
    {
      RCLCPP_ERROR(get_logger(), "Expected a tightly packed little-endian cumulative cloud.");
      return;
    }
    for (const std::string name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&name](const auto & item) {return item.name == name;});
      if (field == cloud.fields.end() ||
        field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->count != 1 ||
        uint64_t(field->offset) + sizeof(float) > cloud.point_step)
      {
        RCLCPP_ERROR(get_logger(), "Expected a valid float32 %s field.", name.c_str());
        return;
      }
    }

    std::vector<std::pair<int32_t, int32_t>> cells;
    cells.reserve(static_cast<size_t>(point_count));
    int32_t min_x = std::numeric_limits<int32_t>::max();
    int32_t min_y = min_x;
    int32_t max_x = std::numeric_limits<int32_t>::min();
    int32_t max_y = max_x;
    if (point_count != 0) {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (uint64_t i = 0; i < point_count; ++i, ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          *z < static_cast<float>(minimum_z_) || *z > static_cast<float>(maximum_z_))
        {
          continue;
        }
        const double gx = std::floor(double(*x) / resolution_);
        const double gy = std::floor(double(*y) / resolution_);
        if (gx < std::numeric_limits<int32_t>::min() ||
          gx > std::numeric_limits<int32_t>::max() ||
          gy < std::numeric_limits<int32_t>::min() || gy > std::numeric_limits<int32_t>::max())
        {
          RCLCPP_ERROR(get_logger(), "Cloud coordinate exceeds grid index limits; update rejected.");
          return;
        }
        const auto cx = static_cast<int32_t>(gx);
        const auto cy = static_cast<int32_t>(gy);
        cells.emplace_back(cx, cy);
        min_x = std::min(min_x, cx); max_x = std::max(max_x, cx);
        min_y = std::min(min_y, cy); max_y = std::max(max_y, cy);
      }
    }

    nav_msgs::msg::OccupancyGrid map;
    map.header = cloud.header;
    map.info.map_load_time = cloud.header.stamp;
    map.info.resolution = static_cast<float>(resolution_);
    map.info.origin.orientation.w = 1.0;
    if (cells.empty()) {
      map.info.width = 1;
      map.info.height = 1;
      map.data.assign(1, -1);
    } else {
      const uint64_t width = uint64_t(int64_t(max_x) - min_x + 1);
      const uint64_t height = uint64_t(int64_t(max_y) - min_y + 1);
      if (width > std::numeric_limits<uint32_t>::max() ||
        height > std::numeric_limits<uint32_t>::max() ||
        width > uint64_t(maximum_cells_) / height)
      {
        RCLCPP_ERROR(get_logger(), "Projected extent exceeds max_grid_cells; update rejected.");
        return;
      }
      map.info.width = static_cast<uint32_t>(width);
      map.info.height = static_cast<uint32_t>(height);
      map.info.origin.position.x = double(min_x) * resolution_;
      map.info.origin.position.y = double(min_y) * resolution_;
      map.data.assign(static_cast<size_t>(width * height), -1);
      for (const auto & cell : cells) {
        const auto ix = uint64_t(int64_t(cell.first) - min_x);
        const auto iy = uint64_t(int64_t(cell.second) - min_y);
        map.data[static_cast<size_t>(iy * width + ix)] = 100;
      }
    }
    // Rebuild from each complete /Laser_map snapshot. Do not append the same
    // cumulative points again, infer free space, or cast flattened clearing rays.
    publisher_->publish(map);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Projected %zu/%llu points into %u x %u cells.", cells.size(),
      static_cast<unsigned long long>(point_count), map.info.width, map.info.height);
  }

  std::string frame_;
  double minimum_z_, maximum_z_, resolution_;
  int64_t maximum_cells_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HeightProjectionMap>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("height_projection_map"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
