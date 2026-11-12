#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include "nav2_map_server/map_io.hpp"
#include "octomap_server/octomap_server.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace strawberry_octomap
{
namespace fs = std::filesystem;
using Base = octomap_server::OctomapServer;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Trigger = std_srvs::srv::Trigger;

constexpr std::size_t kMaxGridCells = 1'000'000;
constexpr double kDefaultMinHeight = 0.10;
constexpr double kDefaultMaxHeight = 1.25;
constexpr auto kPreviewPeriod = std::chrono::seconds(1);
constexpr auto kDiagnosticPeriod = std::chrono::seconds(10);
constexpr std::array<const char *, 4> kOutputFiles{
  "map.pgm", "map.yaml", "octree.ot", "metadata.json"};

struct Projection
{
  OccupancyGrid map;
  std::size_t occupied = 0;
  std::size_t free = 0;
};

std::uint64_t lower_bits(int count)
{
  return count == 64 ? std::numeric_limits<std::uint64_t>::max() :
         (std::uint64_t{1} << count) - 1;
}

std::string json_string(const std::string & value)
{
  std::ostringstream output;
  output << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') {
      output << '\\' << c;
    } else if (c < 32) {
      output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c);
    } else {
      output << c;
    }
  }
  output << '"';
  return output.str();
}

class OccupancyServer final : public Base
{
public:
  explicit OccupancyServer(const rclcpp::NodeOptions & options)
  : Base(options)
  {
    output_dir_ = declare_parameter<std::string>("output_dir", "");
    if (!output_dir_.is_absolute()) {
      throw std::invalid_argument("output_dir must be the absolute path of the session directory");
    }
    if (filter_ground_plane_) {
      throw std::invalid_argument("this experiment requires filter_ground_plane=false");
    }
    if (!get_parameter("octomap_path").as_string().empty()) {
      throw std::invalid_argument("this experiment requires a new, empty OctoMap");
    }

    // Keep valid endpoints above and below the projection band for 3D ray updates.
    point_cloud_min_z_ = std::numeric_limits<float>::lowest();
    point_cloud_max_z_ = std::numeric_limits<float>::max();
    min_height_ = occupancy_min_z_ == -100.0 ? kDefaultMinHeight : occupancy_min_z_;
    max_height_ = occupancy_max_z_ == 100.0 ? kDefaultMaxHeight : occupancy_max_z_;
    if (!std::isfinite(res_) || res_ <= 0.0) {
      throw std::invalid_argument("resolution must be finite and positive");
    }
    zero_key_ = octree_->coordToKey(0.0);
    key_limit_ = 1 << tree_depth_;
    z_begin_ = height_boundary_key(min_height_);
    z_end_ = height_boundary_key(max_height_);
    if (z_end_ <= z_begin_ || z_end_ - z_begin_ > 64) {
      throw std::invalid_argument("the projection band must contain 1 to 64 height voxels");
    }

    // The base class uses one latch flag for every publisher. Only the 2D grid needs retention.
    map_pub_ = create_publisher<OccupancyGrid>(
      "projected_map", rclcpp::QoS(1).reliable().transient_local());
    if (declare_parameter<bool>("publish_web_map", false)) {
      web_map_pub_ = create_publisher<OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local());
    }
    // Keep the inherited read-only octree services and remove the editing services.
    clear_bbox_srv_.reset();
    reset_srv_.reset();
    set_param_res_.reset();
    set_param_res_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "set mapping parameters when starting a new session";
        return result;
      });
    finalize_srv_ = create_service<Trigger>(
      "/strawberry_octomap/finalize",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        finalize(*response);
      });
    RCLCPP_INFO(
      get_logger(), "Projecting %.2f m voxels in [%.2f, %.2f) m (%d layers)",
      res_, min_height_, max_height_, z_end_ - z_begin_);
  }

  ~OccupancyServer() override
  {
    // Join the base TF listener before destroying the state used by its queued callbacks.
    tf2_listener_.reset();
    point_cloud_sub_.unsubscribe();
    tf_point_cloud_sub_.reset();
  }

  void insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override
  {
    // A TF-ready cloud can be released by the listener while the executor handles a service.
    std::lock_guard<std::recursive_mutex> guard(tree_mutex_);
    if (frozen_) {
      return;
    }
    ++received_scans_;
    if (cloud->header.frame_id != "strawberry_lidar") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring cloud with unexpected frame %s",
        cloud->header.frame_id.c_str());
      return;
    }
    const rclcpp::Time stamp(cloud->header.stamp);
    if (latest_stamp_ && stamp.nanoseconds() <= latest_stamp_->nanoseconds()) {
      return;
    }
    Base::insertCloudCallback(cloud);
  }

  bool onOctomapBinarySrv(
    const std::shared_ptr<OctomapSrv::Request> request,
    const std::shared_ptr<OctomapSrv::Response> response) override
  {
    std::lock_guard<std::recursive_mutex> guard(tree_mutex_);
    return Base::onOctomapBinarySrv(request, response);
  }

  bool onOctomapFullSrv(
    const std::shared_ptr<OctomapSrv::Request> request,
    const std::shared_ptr<OctomapSrv::Response> response) override
  {
    std::lock_guard<std::recursive_mutex> guard(tree_mutex_);
    return Base::onOctomapFullSrv(request, response);
  }

protected:
  void publishAll(const rclcpp::Time & stamp) override
  {
    std::lock_guard<std::recursive_mutex> guard(tree_mutex_);
    latest_stamp_ = stamp;
    ++integrated_scans_;
    const auto now = std::chrono::steady_clock::now();
    if (last_preview_ && now - *last_preview_ < kPreviewPeriod) {
      return;
    }
    last_preview_ = now;
    try {
      if (const auto projection = project(stamp)) {
        publish_grid(projection->map);
        if (!ready_ && projection->occupied + projection->free > 0) {
          ready_ = true;
          RCLCPP_INFO(get_logger(), "OCTOMAP_MAP_READY");
        }
        if (!last_diagnostics_ || now - *last_diagnostics_ >= kDiagnosticPeriod) {
          last_diagnostics_ = now;
          RCLCPP_INFO(
            get_logger(),
            "OCTOMAP_PROGRESS accepted_scan_stamp_ns=%lld scans_received=%zu scans_integrated=%zu "
            "tree_nodes=%zu occupied_cells=%zu free_cells=%zu",
            static_cast<long long>(stamp.nanoseconds()), received_scans_, integrated_scans_,
            octree_->size(), projection->occupied, projection->free);
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "2D projection failed: %s", error.what());
    }
  }

private:
  void publish_grid(const OccupancyGrid & map)
  {
    map_pub_->publish(map);
    if (web_map_pub_) {
      web_map_pub_->publish(map);
    }
  }

  int height_boundary_key(double height) const
  {
    const double scaled = height / res_;
    if (!std::isfinite(scaled) || scaled < -key_limit_ || scaled > key_limit_) {
      throw std::invalid_argument("projection height is outside the OctoMap key range");
    }
    const auto rounded = std::llround(scaled);
    const auto key = zero_key_ + rounded;
    if (std::abs(scaled - rounded) > 1e-8 || key < 0 || key > key_limit_) {
      throw std::invalid_argument("projection heights must align with OctoMap voxel boundaries");
    }
    return static_cast<int>(key);
  }

  std::optional<Projection> project(const rclcpp::Time & stamp) const
  {
    // Work in integer keys, using half-open bounds even for compressed leaves.
    const octomap::OcTreeKey lower(0, 0, static_cast<octomap::key_type>(z_begin_));
    const auto last_key = static_cast<octomap::key_type>(key_limit_ - 1);
    const octomap::OcTreeKey upper(
      last_key, last_key, static_cast<octomap::key_type>(z_end_ - 1));
    const auto end = octree_->end_leafs_bbx();
    int x_begin = key_limit_, x_end = 0;
    int y_begin = key_limit_, y_end = 0;
    for (auto leaf = octree_->begin_leafs_bbx(lower, upper); leaf != end; ++leaf) {
      const auto key = leaf.getIndexKey();
      const int span = 1 << (tree_depth_ - leaf.getDepth());
      if (key[2] + span <= z_begin_ || key[2] >= z_end_) {
        continue;
      }
      x_begin = std::min(x_begin, static_cast<int>(key[0]));
      y_begin = std::min(y_begin, static_cast<int>(key[1]));
      x_end = std::max(x_end, key[0] + span);
      y_end = std::max(y_end, key[1] + span);
    }
    if (x_end <= x_begin || y_end <= y_begin) {
      return std::nullopt;
    }
    const auto width = static_cast<std::size_t>(x_end - x_begin);
    const auto height = static_cast<std::size_t>(y_end - y_begin);
    if (width > kMaxGridCells / height) {
      throw std::length_error("projected grid exceeds 1000000 cells");
    }

    Projection result;
    auto & map = result.map;
    map.header.frame_id = world_frame_id_;
    map.header.stamp = stamp;
    map.info.map_load_time = stamp;
    map.info.resolution = static_cast<float>(res_);
    map.info.width = static_cast<std::uint32_t>(width);
    map.info.height = static_cast<std::uint32_t>(height);
    map.info.origin.position.x = (x_begin - zero_key_) * res_;
    map.info.origin.position.y = (y_begin - zero_key_) * res_;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(width * height, -1);
    std::vector<std::uint64_t> free_layers(map.data.size(), 0);
    const std::uint64_t all_layers = lower_bits(z_end_ - z_begin_);

    for (auto leaf = octree_->begin_leafs_bbx(lower, upper); leaf != end; ++leaf) {
      const auto key = leaf.getIndexKey();
      const int span = 1 << (tree_depth_ - leaf.getDepth());
      const int first_layer = std::max(static_cast<int>(key[2]), z_begin_) - z_begin_;
      const int end_layer = std::min(key[2] + span, z_end_) - z_begin_;
      if (end_layer <= first_layer) {
        continue;
      }
      const bool occupied = octree_->isNodeOccupied(*leaf);
      const std::uint64_t coverage = lower_bits(end_layer) & ~lower_bits(first_layer);
      const int first_x = key[0] - x_begin;
      const int first_y = key[1] - y_begin;
      for (int y = first_y; y < first_y + span; ++y) {
        const auto row = static_cast<std::size_t>(y) * width;
        for (int x = first_x; x < first_x + span; ++x) {
          const auto index = row + static_cast<std::size_t>(x);
          if (occupied) {
            map.data[index] = 100;
          } else {
            free_layers[index] |= coverage;
          }
        }
      }
    }

    for (std::size_t index = 0; index < map.data.size(); ++index) {
      if (map.data[index] == 100) {
        ++result.occupied;
      } else if (free_layers[index] == all_layers) {
        map.data[index] = 0;
        ++result.free;
      }
    }
    return result;
  }

  void write_metadata(const fs::path & path, const Projection & result) const
  {
    const auto & map = result.map;
    std::ofstream stream;
    stream.imbue(std::locale::classic());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream.open(path);
    stream << std::setprecision(12)
           << "{\n  \"status\": \"complete\",\n"
           << "  \"stamp\": {\"sec\": " << map.header.stamp.sec << ", \"nanosec\": "
           << map.header.stamp.nanosec << "},\n"
           << "  \"frame_id\": " << json_string(map.header.frame_id) << ",\n"
           << "  \"resolution_m\": " << res_ << ",\n"
           << "  \"projection_height_m\": [" << min_height_ << ", " << max_height_ << "],\n"
           << "  \"projection_upper_bound_exclusive\": true,\n"
           << "  \"height_voxels\": " << z_end_ - z_begin_ << ",\n"
           << "  \"origin\": [" << map.info.origin.position.x << ", "
           << map.info.origin.position.y << ", 0],\n"
           << "  \"width\": " << map.info.width << ", \"height\": " << map.info.height << ",\n"
           << "  \"occupied_cells\": " << result.occupied << ", \"free_cells\": " << result.free
           << ", \"unknown_cells\": " << map.data.size() - result.occupied - result.free << ",\n"
           << "  \"octree_nodes\": " << octree_->size() << ",\n"
           << "  \"octree_estimated_bytes\": " << octree_->memoryUsage() << ",\n"
           << "  \"received_scans\": " << received_scans_ << ", \"integrated_scans\": "
           << integrated_scans_ << ",\n"
           << "  \"sensor_model\": {\"hit\": " << octree_->getProbHit() << ", \"miss\": "
           << octree_->getProbMiss() << ", \"min\": " << octree_->getClampingThresMin()
           << ", \"max\": " << octree_->getClampingThresMax() << ", \"occupancy_threshold\": "
           << octree_->getOccupancyThres() << ", \"max_range_m\": " << max_range_ << "},\n"
           << "  \"versions\": {\"strawberry_octomap\": "
           << json_string(STRAWBERRY_PACKAGE_VERSION) << ", \"octomap_server\": "
           << json_string(STRAWBERRY_OCTOMAP_SERVER_VERSION) << ", \"octomap_core\": "
           << json_string(STRAWBERRY_OCTOMAP_VERSION) << ", \"nav2_map_server\": "
           << json_string(STRAWBERRY_NAV2_MAP_SERVER_VERSION) << "}\n}\n";
    stream.close();
  }

  void write_map_yaml(const fs::path & path, const OccupancyGrid & map) const
  {
    // Humble map_io writes YAML with three significant digits, which can shift the grid origin.
    std::ofstream stream;
    stream.imbue(std::locale::classic());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream.open(path);
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "image: map.pgm\nmode: trinary\nresolution: " << res_ << "\norigin: ["
           << map.info.origin.position.x << ", " << map.info.origin.position.y
           << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n";
    stream.close();
  }

  void save(const Projection & result) const
  {
    if (!fs::is_directory(output_dir_)) {
      throw std::runtime_error("output_dir does not exist");
    }
    for (const auto * name : kOutputFiles) {
      if (fs::exists(output_dir_ / name)) {
        throw std::runtime_error(std::string("will not overwrite existing output: ") + name);
      }
    }
    const auto staging = output_dir_ / (".octomap-finalize-" + std::to_string(getpid()));
    if (!fs::create_directory(staging)) {
      throw std::runtime_error("could not create the staging directory");
    }

    std::size_t committed = 0;
    try {
      nav2_map_server::SaveParameters parameters;
      parameters.map_file_name = (staging / "map").string();
      parameters.image_format = "pgm";
      parameters.mode = nav2_map_server::MapMode::Trinary;
      parameters.free_thresh = 0.19;  // Preserve gray 205 as unknown when Nav2 reloads the PGM.
      parameters.occupied_thresh = 0.65;
      if (!nav2_map_server::saveMapToFile(result.map, parameters)) {
        throw std::runtime_error("Nav2 could not save map.pgm and map.yaml");
      }
      write_map_yaml(staging / "map.yaml", result.map);

      std::ofstream tree;
      tree.exceptions(std::ios::badbit | std::ios::failbit);
      tree.open(staging / "octree.ot", std::ios::binary);
      if (!octree_->write(tree)) {
        throw std::runtime_error("OctoMap could not save octree.ot");
      }
      tree.close();
      write_metadata(staging / "metadata.json", result);

      for (const auto * name : kOutputFiles) {
        const auto file = staging / name;
        if (!fs::is_regular_file(file) || fs::file_size(file) == 0) {
          throw std::runtime_error(std::string("missing or empty output: ") + name);
        }
      }
      // Commit metadata last so the caller can distinguish partial output after interruption.
      for (const auto * name : kOutputFiles) {
        fs::rename(staging / name, output_dir_ / name);
        ++committed;
      }
    } catch (...) {
      for (std::size_t index = 0; index < committed; ++index) {
        std::error_code ignored;
        fs::remove(output_dir_ / kOutputFiles[index], ignored);
      }
      std::error_code ignored;
      fs::remove_all(staging, ignored);
      throw;
    }
    std::error_code ignored;
    fs::remove(staging, ignored);
  }

  void finalize(Trigger::Response & response)
  {
    std::lock_guard<std::recursive_mutex> guard(tree_mutex_);
    if (saved_) {
      response.success = true;
      response.message = (output_dir_ / "map.yaml").string();
      return;
    }

    frozen_ = true;
    try {
      if (!latest_stamp_) {
        throw std::runtime_error("no point cloud was accepted by OctoMap");
      }
      const auto projection = project(*latest_stamp_);
      if (!projection || projection->occupied + projection->free == 0) {
        throw std::runtime_error("final projection has no known cells");
      }
      publish_grid(projection->map);
      save(*projection);
      saved_ = true;
      response.success = true;
      response.message = (output_dir_ / "map.yaml").string();
      RCLCPP_INFO(get_logger(), "Saved final 2D map and OctoMap in %s", output_dir_.c_str());
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
      RCLCPP_ERROR(get_logger(), "Finalization failed: %s", error.what());
    }
  }

  fs::path output_dir_;
  int zero_key_ = 0, key_limit_ = 0, z_begin_ = 0, z_end_ = 0;
  double min_height_ = 0.0, max_height_ = 0.0;
  std::recursive_mutex tree_mutex_;
  std::optional<rclcpp::Time> latest_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_preview_;
  std::optional<std::chrono::steady_clock::time_point> last_diagnostics_;
  std::size_t received_scans_ = 0;
  std::size_t integrated_scans_ = 0;
  bool ready_ = false;
  bool frozen_ = false;
  bool saved_ = false;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr web_map_pub_;
  rclcpp::Service<Trigger>::SharedPtr finalize_srv_;
};
}  // namespace strawberry_octomap

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<strawberry_octomap::OccupancyServer>(rclcpp::NodeOptions{});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("strawberry_octomap"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
