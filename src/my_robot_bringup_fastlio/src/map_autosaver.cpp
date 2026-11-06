#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

std::string make_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::filesystem::path expand_home(const std::string & path)
{
  if (path.empty() || path.front() != '~') {
    return std::filesystem::path(path);
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return std::filesystem::path(path);
  }

  if (path.size() == 1) {
    return std::filesystem::path(home);
  }
  if (path[1] == '/') {
    return std::filesystem::path(home) / path.substr(2);
  }
  return std::filesystem::path(path);
}

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

class MapAutosaver : public rclcpp::Node
{
public:
  MapAutosaver()
  : Node("map_autosaver")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    output_directory_ = expand_home(
      declare_parameter<std::string>("output_directory", "~/maps/manual_mapping"));
    map_name_ = declare_parameter<std::string>("map_name", "");
    map_name_prefix_ = declare_parameter<std::string>("map_name_prefix", "manual");
    periodic_save_interval_ = declare_parameter<double>("periodic_save_interval", 60.0);
    free_threshold_ = declare_parameter<double>("free_threshold", 0.25);
    occupied_threshold_ = declare_parameter<double>("occupied_threshold", 0.65);

    if (map_name_.empty()) {
      map_name_ = map_name_prefix_ + "_" + make_timestamp();
    }
    if (map_name_.find('/') != std::string::npos ||
      map_name_.find('\\') != std::string::npos ||
      map_name_prefix_.empty() ||
      map_name_prefix_.find('/') != std::string::npos ||
      map_name_prefix_.find('\\') != std::string::npos)
    {
      throw std::invalid_argument(
              "map_name and map_name_prefix must be file names, not paths");
    }
    if (free_threshold_ < 0.0 || occupied_threshold_ > 1.0 ||
      free_threshold_ >= occupied_threshold_)
    {
      throw std::invalid_argument(
              "thresholds must satisfy 0.0 <= free_threshold < occupied_threshold <= 1.0");
    }

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = std::move(message);
      });

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save_now",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::string detail;
        response->success = save_latest_map("manual request", detail);
        response->message = detail;
      });

    if (periodic_save_interval_ > 0.0) {
      periodic_timer_ = create_wall_timer(
        std::chrono::duration<double>(periodic_save_interval_),
        [this]() {
          std::string unused;
          save_latest_map("periodic backup", unused, false);
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "Map autosave armed: %s -> %s/%s.{pgm,yaml}; interval %.1f s",
      map_topic_.c_str(), output_directory_.c_str(), map_name_.c_str(),
      periodic_save_interval_);
  }

  bool save_on_shutdown()
  {
    std::string detail;
    const bool saved = save_latest_map("shutdown", detail);
    if (!saved) {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    return saved;
  }

private:
  bool save_latest_map(
    const std::string & reason, std::string & detail, const bool log_success = true)
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = latest_map_;
    }

    if (!map) {
      detail = "No /map message has been received; nothing was saved.";
      return false;
    }
    if (map->info.width == 0 || map->info.height == 0 ||
      map->data.size() !=
      static_cast<std::size_t>(map->info.width) * static_cast<std::size_t>(map->info.height))
    {
      detail = "The latest occupancy grid is empty or malformed; nothing was saved.";
      return false;
    }

    std::lock_guard<std::mutex> save_lock(save_mutex_);
    try {
      std::filesystem::create_directories(output_directory_);
      const auto base_path = output_directory_ / map_name_;
      const auto pgm_path = base_path.string() + ".pgm";
      const auto yaml_path = base_path.string() + ".yaml";
      const auto pgm_temporary = pgm_path + ".tmp";
      const auto yaml_temporary = yaml_path + ".tmp";

      write_pgm(*map, pgm_temporary);
      write_yaml(*map, yaml_temporary, std::filesystem::path(pgm_path).filename().string());

      std::filesystem::rename(pgm_temporary, pgm_path);
      std::filesystem::rename(yaml_temporary, yaml_path);

      detail = "Saved " + pgm_path + " and " + yaml_path;
      if (log_success) {
        RCLCPP_INFO(get_logger(), "%s (%s)", detail.c_str(), reason.c_str());
      }
      return true;
    } catch (const std::exception & error) {
      detail = "Failed to save map: " + std::string(error.what());
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
      return false;
    }
  }

  void write_pgm(
    const nav_msgs::msg::OccupancyGrid & map, const std::filesystem::path & output_path) const
  {
    std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("cannot open " + output_path.string());
    }

    output << "P5\n# CREATOR: my_robot_bringup_fastlio map_autosaver\n";
    output << map.info.width << " " << map.info.height << "\n255\n";

    for (int row = static_cast<int>(map.info.height) - 1; row >= 0; --row) {
      for (std::uint32_t column = 0; column < map.info.width; ++column) {
        const std::size_t index =
          static_cast<std::size_t>(row) * map.info.width + column;
        const std::int8_t occupancy = map.data[index];

        std::uint8_t pixel = 205;
        if (occupancy >= 0) {
          const double probability = static_cast<double>(occupancy) / 100.0;
          if (probability >= occupied_threshold_) {
            pixel = 0;
          } else if (probability <= free_threshold_) {
            pixel = 254;
          }
        }
        output.write(reinterpret_cast<const char *>(&pixel), sizeof(pixel));
      }
    }

    if (!output) {
      throw std::runtime_error("failed while writing " + output_path.string());
    }
  }

  void write_yaml(
    const nav_msgs::msg::OccupancyGrid & map, const std::filesystem::path & output_path,
    const std::string & image_name) const
  {
    std::ofstream output(output_path, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("cannot open " + output_path.string());
    }

    output << std::setprecision(12);
    output << "image: " << image_name << "\n";
    output << "mode: trinary\n";
    output << "resolution: " << map.info.resolution << "\n";
    output << "origin: [" << map.info.origin.position.x << ", "
           << map.info.origin.position.y << ", "
           << quaternion_to_yaw(map.info.origin.orientation) << "]\n";
    output << "negate: 0\n";
    output << "occupied_thresh: " << occupied_threshold_ << "\n";
    output << "free_thresh: " << free_threshold_ << "\n";

    if (!output) {
      throw std::runtime_error("failed while writing " + output_path.string());
    }
  }

  std::string map_topic_;
  std::filesystem::path output_directory_;
  std::string map_name_;
  std::string map_name_prefix_;
  double periodic_save_interval_;
  double free_threshold_;
  double occupied_threshold_;

  std::mutex map_mutex_;
  std::mutex save_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapAutosaver>();
  rclcpp::spin(node);
  node->save_on_shutdown();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
