#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
struct Key
{
  int32_t x, y, z;
  bool operator==(const Key & b) const {return x == b.x && y == b.y && z == b.z;}
};
struct Hash
{
  size_t operator()(const Key & k) const
  {
    size_t h = std::hash<int32_t>{}(k.x);
    h ^= std::hash<int32_t>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int32_t>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
struct Evidence
{
  tf2::Vector3 point;
  std::vector<tf2::Vector3> views;
};
struct ImuSample
{
  int64_t stamp;
  tf2::Vector3 acc, gyro;
};
struct GroundPlane
{
  tf2::Vector3 normal;
  double offset;
  size_t inliers;
};
struct ReferenceSample
{
  int64_t stamp;
  GroundPlane plane;
};
double angle_degrees(const tf2::Vector3 & a, const tf2::Vector3 & b)
{
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 180.0 / std::acos(-1.0);
}
tf2::Transform transform(const geometry_msgs::msg::Transform & t)
{
  tf2::Quaternion q(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
  if (q.length2() < 1e-12) {throw std::runtime_error("invalid transform quaternion");}
  q.normalize();
  return tf2::Transform(q, tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}
bool finite(const tf2::Vector3 & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
}  // namespace

class RackObstacleLayer : public rclcpp::Node
{
public:
  RackObstacleLayer() : Node("rack_obstacle_layer"), buffer_(get_clock()), listener_(buffer_)
  {
    min_height_ = declare_parameter<double>("min_height", 0.3);
    max_height_ = declare_parameter<double>("max_height", 1.0);
    xy_size_ = declare_parameter<double>("evidence_xy_size", 0.05);
    z_size_ = declare_parameter<double>("evidence_z_size", 0.10);
    view_separation_ = declare_parameter<double>("min_view_separation", 0.10);
    min_views_ = declare_parameter<int>("min_views", 3);
    bottom_height_ = declare_parameter<double>("sensor_bottom_height", 0.20);
    bottom_in_body_ = tf2::Vector3(
      declare_parameter<double>("sensor_bottom_body_x", -0.011),
      declare_parameter<double>("sensor_bottom_body_y", -0.02329),
      declare_parameter<double>("sensor_bottom_body_z", -0.00288));
    imu_samples_ = declare_parameter<int>("reference_imu_samples", 100);
    max_gyro_ = declare_parameter<double>("reference_max_gyro", 0.05);
    max_acc_spread_ = declare_parameter<double>("reference_max_relative_acc_spread", 0.02);
    stable_duration_ = declare_parameter<double>("reference_stable_duration", 3.0);
    stable_frames_ = declare_parameter<int>("reference_stable_frames", 10);
    stable_angle_ = declare_parameter<double>("reference_stable_angle_deg", 0.5);
    stable_offset_ = declare_parameter<double>("reference_stable_offset", 0.03);
    imu_ground_angle_ = declare_parameter<double>("reference_max_imu_ground_angle_deg", 3.0);
    plane_distance_ = declare_parameter<double>("reference_plane_distance", 0.025);
    height_tolerance_ = declare_parameter<double>("reference_sensor_height_tolerance", 0.05);
    plane_min_points_ = declare_parameter<int>("reference_min_plane_points", 35);
    plane_min_ratio_ = declare_parameter<double>("reference_min_plane_ratio", 0.20);
    ground_range_ = declare_parameter<double>("reference_ground_max_range", 3.0);
    check_period_ = declare_parameter<double>("reference_check_period", 0.2);
    max_reference_age_ = declare_parameter<double>("reference_max_age", 2.0);
    monitor_angle_ = declare_parameter<double>("reference_monitor_angle_deg", 1.0);
    monitor_offset_ = declare_parameter<double>("reference_monitor_offset", 0.06);
    mismatch_confirmations_ = declare_parameter<int>("reference_mismatch_confirmations", 3);
    max_cells_ = declare_parameter<int64_t>("max_grid_cells", 10000000);
    max_evidence_ = declare_parameter<int64_t>("max_evidence_voxels", 1000000);
    stable_frame_ = declare_parameter<std::string>("stable_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    if (!std::isfinite(min_height_) || !std::isfinite(max_height_) ||
      min_height_ < 0 || max_height_ < min_height_ ||
      !std::isfinite(xy_size_) || xy_size_ <= 0 ||
      !std::isfinite(z_size_) || z_size_ <= 0 ||
      !std::isfinite(view_separation_) || view_separation_ <= 0 || min_views_ < 2 ||
      !std::isfinite(bottom_height_) || bottom_height_ <= 0 || !finite(bottom_in_body_) ||
      imu_samples_ < 10 || !std::isfinite(max_gyro_) || max_gyro_ <= 0 ||
      !std::isfinite(max_acc_spread_) || max_acc_spread_ <= 0 ||
      !std::isfinite(stable_duration_) || stable_duration_ <= 0 || stable_frames_ < 3 ||
      !std::isfinite(stable_angle_) || stable_angle_ <= 0 || stable_angle_ > 5 ||
      !std::isfinite(stable_offset_) || stable_offset_ <= 0 ||
      !std::isfinite(imu_ground_angle_) || imu_ground_angle_ <= 0 || imu_ground_angle_ > 15 ||
      !std::isfinite(plane_distance_) || plane_distance_ <= 0 ||
      !std::isfinite(height_tolerance_) || height_tolerance_ <= 0 || height_tolerance_ >= bottom_height_ ||
      plane_min_points_ < 10 || !std::isfinite(plane_min_ratio_) ||
      plane_min_ratio_ <= 0 || plane_min_ratio_ > 1 ||
      !std::isfinite(ground_range_) || ground_range_ <= 0.5 ||
      !std::isfinite(check_period_) || check_period_ <= 0 ||
      !std::isfinite(max_reference_age_) || max_reference_age_ <= check_period_ ||
      !std::isfinite(monitor_angle_) || monitor_angle_ <= 0 || monitor_angle_ > 5 ||
      !std::isfinite(monitor_offset_) || monitor_offset_ <= 0 ||
      mismatch_confirmations_ < 2 || mismatch_confirmations_ > 20 ||
      max_cells_ <= 0 || max_evidence_ <= 0 || stable_frame_.empty() || body_frame_.empty())
    {
      throw std::invalid_argument("invalid rack-layer parameters");
    }
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    fused_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map_rack_overlay", map_qos);
    layer_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/rack_layer/confirmed_map", map_qos);
    base_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        if (!valid_map(*map)) {return;}
        base_ = map;
        if (reference_ready_ && (get_clock()->now().nanoseconds() - last_ground_stamp_) * 1e-9 >
          max_reference_age_) {hold_reference("ground temporarily not observable");}
        publish();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/livox/imu",
      rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr imu) {
        const auto & a = imu->linear_acceleration;
        const auto & w = imu->angular_velocity;
        tf2::Vector3 acc(a.x, a.y, a.z), gyro(w.x, w.y, w.z);
        if (!finite(acc) || !finite(gyro)) {imu_window_.clear(); return;}
        const int64_t stamp = rclcpp::Time(imu->header.stamp).nanoseconds();
        if (!imu_window_.empty() && stamp <= imu_window_.back().stamp) {return;}
        imu_window_.push_back(ImuSample{stamp, acc, gyro});
        while (imu_window_.size() > size_t(4 * imu_samples_)) {imu_window_.pop_front();}
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_body",
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {observe(*cloud);});
    RCLCPP_INFO(get_logger(),
      "Rack test: leave old /map unchanged; overlay height [%.2f, %.2f] m, "
      "%d viewpoints separated by %.2f m. Keep still during initial height reference. "
      "Height reference requires stable geometry checked against IMU and measured "
      "sensor-bottom height %.3f m; ongoing checks enabled. No TF edits.",
      min_height_, max_height_, min_views_, view_separation_, bottom_height_);
  }

private:
  bool valid_map(const nav_msgs::msg::OccupancyGrid & m)
  {
    const auto & q = m.info.origin.orientation;
    if (m.header.frame_id.empty() || !std::isfinite(m.info.resolution) ||
      m.info.resolution <= 0 || m.info.width == 0 || m.info.height == 0 ||
      uint64_t(m.info.width) * m.info.height != m.data.size() ||
      m.data.size() > uint64_t(max_cells_) ||
      !std::isfinite(m.info.origin.position.x) || !std::isfinite(m.info.origin.position.y) ||
      std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
      std::abs(q.z) > 1e-6 || std::abs(std::abs(q.w) - 1.0) > 1e-6)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid, oversized, or rotated base grid; update rejected.");
      return false;
    }
    return true;
  }

  bool imu_reference(int64_t stamp, tf2::Vector3 & mean, bool & stationary)
  {
    // Match the scan timestamp, excluding future IMU samples already delivered.
    std::vector<const ImuSample *> window;
    for (auto it = imu_window_.rbegin(); it != imu_window_.rend(); ++it) {
      if (it->stamp > stamp) {continue;}
      window.push_back(&*it);
      if (window.size() == size_t(imu_samples_)) {break;}
    }
    if (window.size() < size_t(imu_samples_) || (stamp - window.front()->stamp) * 1e-9 > 0.25) {
      return false;
    }
    mean.setValue(0, 0, 0); stationary = true;
    for (const auto * sample : window) {
      if (sample->gyro.length() > max_gyro_) {stationary = false;}
      mean += sample->acc;
    }
    mean /= double(window.size());
    if (mean.length() < 1e-6) {return false;}
    double variance = 0;
    for (const auto * sample : window) {variance += (sample->acc - mean).length2();}
    if (std::sqrt(variance / window.size()) / mean.length() > max_acc_spread_) {
      stationary = false;
    }
    mean.normalize();
    return true;
  }

  bool fit_ground(const std::vector<tf2::Vector3> & body, const tf2::Vector3 & rough_up,
    GroundPlane & result)
  {
    std::vector<tf2::Vector3> points;
    for (const auto & p : body) {
      const double range = std::hypot(p.x(), p.y());
      // Only calibrate near the measured floor. Obstacle selection still uses
      // the original [min_height,max_height] gate, without another point filter.
      const double rough_height = rough_up.dot(p - bottom_in_body_) + bottom_height_;
      if (range >= 0.5 && range <= ground_range_ && std::abs(rough_height) <= 0.25) {
        points.push_back(p);
      }
    }
    if (points.size() < size_t(plane_min_points_)) {return false;}
    if (points.size() > 1200) {
      std::vector<tf2::Vector3> sampled;
      for (size_t i = 0; i < 1200; ++i) {sampled.push_back(points[i * points.size() / 1200]);}
      points.swap(sampled);
    }
    auto plausible = [&](const tf2::Vector3 & n, double d) {
        return n.dot(rough_up) >= std::cos(15.0 * std::acos(-1.0) / 180.0) &&
               std::abs(n.dot(bottom_in_body_) + d - bottom_height_) <= height_tolerance_;
      };
    std::uniform_int_distribution<size_t> choose(0, points.size() - 1);
    size_t best_count = 0;
    tf2::Vector3 normal;
    double offset = 0;
    for (int trial = 0; trial < 120; ++trial) {
      const auto & a = points[choose(random_)];
      const auto & b = points[choose(random_)];
      const auto & c = points[choose(random_)];
      auto n = (b - a).cross(c - a);
      if (n.length2() < 1e-10) {continue;}
      n.normalize();
      if (n.dot(rough_up) < 0) {n = -n;}
      const double d = -n.dot(a);
      if (!plausible(n, d)) {continue;}
      size_t count = 0;
      for (const auto & p : points) {if (std::abs(n.dot(p) + d) <= plane_distance_) {++count;}}
      if (count > best_count) {best_count = count; normal = n; offset = d;}
    }
    if (best_count < size_t(plane_min_points_) ||
      double(best_count) / points.size() < plane_min_ratio_) {return false;}
    for (int refine = 0; refine < 2; ++refine) {
      std::vector<tf2::Vector3> inliers;
      tf2::Vector3 center(0, 0, 0);
      for (const auto & p : points) {
        if (std::abs(normal.dot(p) + offset) <= plane_distance_) {
          inliers.push_back(p); center += p;
        }
      }
      if (inliers.size() < size_t(plane_min_points_) ||
        double(inliers.size()) / points.size() < plane_min_ratio_) {return false;}
      center /= double(inliers.size());
      tf2::Matrix3x3 covariance(0, 0, 0, 0, 0, 0, 0, 0, 0);
      for (const auto & p : inliers) {
        const auto delta = p - center;
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            covariance[row][col] += delta[row] * delta[col] / inliers.size();
          }
        }
      }
      // Existing tf2 Jacobi solver: no new dependency or external segmenter.
      tf2::Matrix3x3 axes;
      covariance.diagonalize(axes, 1e-10, 40);
      int smallest = 0;
      for (int j = 1; j < 3; ++j) {
        if (covariance[j][j] < covariance[smallest][smallest]) {smallest = j;}
      }
      double second = std::numeric_limits<double>::max();
      for (int j = 0; j < 3; ++j) {
        if (j != smallest) {second = std::min(second, double(covariance[j][j]));}
      }
      if (second < 0.015) {return false;}  // Reject a thin line posing as a plane.
      normal = axes.getColumn(smallest).normalized();
      if (normal.dot(rough_up) < 0) {normal = -normal;}
      offset = -normal.dot(center);
      if (!plausible(normal, offset)) {return false;}
      best_count = inliers.size();
    }
    result = GroundPlane{normal, offset, best_count};
    return true;
  }

  void invalidate_reference(const char * reason)
  {
    if (reference_ready_) {
      RCLCPP_WARN(get_logger(),
        "HEIGHT_REFERENCE_INVALIDATED epoch=%zu reason=%s; withdrawing %zu confirmed voxels "
        "and %zu candidates. Baseline preserved; new reference must be verified again.",
        reference_epoch_, reason, confirmed_.size(), evidence_.size());
    }
    reference_ready_ = false; reference_verified_ = false;
    reference_window_.clear(); mismatch_window_.clear(); evidence_.clear(); confirmed_.clear();
  }

  void hold_reference(const char * reason)
  {
    if (reference_verified_) {
      RCLCPP_WARN(get_logger(),
        "HEIGHT_REFERENCE_HOLD epoch=%zu reason=%s; accepting no new evidence. "
        "Retaining %zu previously verified voxels; no new reference/epoch declared.",
        reference_epoch_, reason, confirmed_.size());
    }
    reference_verified_ = false;
  }

  bool verify_reference(const std::vector<tf2::Vector3> & body,
    const tf2::Transform & stable_body, int64_t stamp)
  {
    if (last_check_stamp_ && (stamp - last_check_stamp_) * 1e-9 < check_period_) {
      return reference_ready_ && reference_verified_;
    }
    last_check_stamp_ = stamp;
    tf2::Vector3 imu_up;
    bool stationary = false;
    GroundPlane local;
    if (!imu_reference(stamp, imu_up, stationary) || !fit_ground(body, imu_up, local)) {
      reference_window_.clear();
      if (reference_ready_ && (stamp - last_ground_stamp_) * 1e-9 > max_reference_age_) {
        hold_reference("no recent trustworthy near-ground plane / IMU");
      }
      return reference_ready_ && reference_verified_;
    }
    GroundPlane world{stable_body.getBasis() * local.normal, local.offset, local.inliers};
    world.offset -= world.normal.dot(stable_body.getOrigin());
    const auto sensor_bottom = stable_body * bottom_in_body_;
    if (reference_ready_) {
      const auto anchor = sensor_bottom - up_ * (up_.dot(sensor_bottom) + ground_offset_);
      const double angle = angle_degrees(up_, world.normal);
      const double shift = std::abs(world.normal.dot(anchor) + world.offset);
      if (angle > monitor_angle_ || shift > monitor_offset_) {
        hold_reference("plane mismatch awaiting consecutive geometric confirmation");
        if (!mismatch_window_.empty()) {
          const auto & first = mismatch_window_.front();
          const double disagreement = std::abs((world.normal - first.plane.normal).dot(sensor_bottom) +
            world.offset - first.plane.offset);
          if ((stamp - mismatch_window_.back().stamp) * 1e-9 > max_reference_age_ ||
            angle_degrees(first.plane.normal, world.normal) > stable_angle_ ||
            disagreement > stable_offset_) {mismatch_window_.clear();}
        }
        mismatch_window_.push_back(ReferenceSample{stamp, world});
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Height reference mismatch: angle=%.4f deg plane_shift=%.4f m consistent_checks=%zu/%d",
          angle, shift, mismatch_window_.size(), mismatch_confirmations_);
        if (mismatch_window_.size() >= size_t(mismatch_confirmations_)) {
          invalidate_reference("consecutive consistent geometry proves reference changed");
        }
        return false;
      }
      if (!reference_verified_) {
        RCLCPP_INFO(get_logger(), "HEIGHT_REFERENCE_RESUMED epoch=%zu; original plane verified again",
          reference_epoch_);
      }
      reference_verified_ = true; mismatch_window_.clear();
      last_ground_stamp_ = stamp;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "HEIGHT_REFERENCE_CHECK epoch=%zu plane_angle=%.4f deg plane_shift=%.4f m inliers=%zu",
        reference_epoch_, angle, shift, local.inliers);
      return true;
    }
    if (!stationary || angle_degrees(imu_up, local.normal) > imu_ground_angle_) {
      reference_window_.clear(); return false;
    }
    if (!reference_window_.empty()) {
      const auto & anchor = reference_window_.front().plane;
      const double disagreement = std::abs((world.normal - anchor.normal).dot(sensor_bottom) +
        world.offset - anchor.offset);
      if (angle_degrees(anchor.normal, world.normal) > stable_angle_ ||
        disagreement > stable_offset_) {reference_window_.clear();}
    }
    reference_window_.push_back(ReferenceSample{stamp, world});
    if (reference_window_.size() < size_t(stable_frames_) ||
      (stamp - reference_window_.front().stamp) * 1e-9 < stable_duration_) {return false;}
    up_.setValue(0, 0, 0); ground_offset_ = 0;
    for (const auto & sample : reference_window_) {
      up_ += sample.plane.normal; ground_offset_ += sample.plane.offset;
    }
    up_ /= double(reference_window_.size()); ground_offset_ /= double(reference_window_.size());
    ground_offset_ /= up_.length(); up_.normalize();
    reference_ready_ = true; reference_verified_ = true;
    last_ground_stamp_ = stamp; ++reference_epoch_;
    RCLCPP_INFO(get_logger(),
      "HEIGHT_REFERENCE_READY epoch=%zu frame=%s up=[%.9f,%.9f,%.9f] offset=%.9f "
      "stable_frames=%zu duration=%.3f s ground_inliers=%zu imu_ground_angle=%.4f deg. "
      "Geometry-verified local flat reference; ongoing checks enabled.",
      reference_epoch_, stable_frame_.c_str(), up_.x(), up_.y(), up_.z(), ground_offset_,
      reference_window_.size(), (stamp - reference_window_.front().stamp) * 1e-9,
      local.inliers, angle_degrees(imu_up, local.normal));
    reference_window_.clear();
    return true;
  }

  void observe(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (cloud.header.frame_id != body_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Expected per-frame body cloud.");
      return;
    }
    // Never count a retransmitted scan as another observation.
    if (have_stamp_ && rclcpp::Time(cloud.header.stamp).nanoseconds() <= last_stamp_) {return;}
    tf2::Transform stable_body;
    try {
      stable_body = transform(buffer_.lookupTransform(
        stable_frame_, body_frame_, cloud.header.stamp, rclcpp::Duration::from_seconds(0.1)).transform);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for scan-time TF: %s", e.what());
      return;
    }
    const uint64_t count = uint64_t(cloud.width) * cloud.height;
    if (cloud.is_bigendian || cloud.point_step == 0 ||
      uint64_t(cloud.row_step) != uint64_t(cloud.width) * cloud.point_step ||
      count * cloud.point_step > cloud.data.size()) {return;}
    for (const std::string name : {"x", "y", "z"}) {
      const auto f = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&name](const auto & field) {return field.name == name;});
      if (f == cloud.fields.end() || f->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        f->count != 1 || uint64_t(f->offset) + 4 > cloud.point_step) {return;}
    }
    have_stamp_ = true;
    last_stamp_ = rclcpp::Time(cloud.header.stamp).nanoseconds();
    std::vector<tf2::Vector3> body;
    if (count > 0) {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (uint64_t i = 0; i < count; ++i, ++x, ++y, ++z) {
        const tf2::Vector3 p(*x, *y, *z);
        if (finite(p)) {body.push_back(p);}
      }
    }
    const bool was_ready = reference_ready_;
    if (!verify_reference(body, stable_body, last_stamp_)) {
      if (was_ready) {publish();}
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "HEIGHT_REFERENCE_WAIT: no new evidence. Before READY / after INVALIDATED, baseline only; "
        "during HOLD, verified history is retained. Need scan-time IMU and trustworthy ground geometry.");
      return;
    }
    std::unordered_set<Key, Hash> this_scan;
    const auto view = stable_body.getOrigin();
    size_t new_confirmed = 0;
    if (!body.empty()) {
      for (const auto & body_point : body) {
        const auto p = stable_body * body_point;
        const double h = up_.dot(p) + ground_offset_;
        if (h < min_height_ - 1e-6 || h > max_height_ + 1e-6) {continue;}
        const double fx = std::floor(p.x() / xy_size_);
        const double fy = std::floor(p.y() / xy_size_);
        const double fz = std::floor(p.z() / z_size_);
        constexpr double lower = double(std::numeric_limits<int32_t>::min());
        constexpr double upper = double(std::numeric_limits<int32_t>::max());
        if (fx < lower || fx > upper || fy < lower || fy > upper || fz < lower || fz > upper) {continue;}
        const Key key{int32_t(fx), int32_t(fy), int32_t(fz)};
        if (!this_scan.insert(key).second) {continue;}
        auto found = evidence_.find(key);
        if (found == evidence_.end()) {
          if (evidence_.size() >= size_t(max_evidence_)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
              "Evidence capacity reached; new candidates rejected, existing evidence retained.");
            continue;
          }
          found = evidence_.emplace(key, Evidence{p, {}}).first;
        }
        auto & ev = found->second;
        if (ev.views.size() >= size_t(min_views_)) {continue;}
        bool distinct = true;
        for (const auto & old : ev.views) {
          if (std::hypot(view.x() - old.x(), view.y() - old.y()) < view_separation_) {
            distinct = false; break;
          }
        }
        if (!distinct) {continue;}
        ev.views.push_back(view);
        if (ev.views.size() == size_t(min_views_)) {confirmed_.push_back(ev.point); ++new_confirmed;}
      }
    }
    if (new_confirmed) {publish();}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Rack evidence: %zu candidate voxels, %zu confirmed voxels. No angular nearest-point selection; "
      "confirmed evidence persists for this experimental session.", evidence_.size(), confirmed_.size());
  }

  void publish()
  {
    if (!base_) {return;}
    const auto & b = *base_;
    tf2::Transform map_stable = tf2::Transform::getIdentity();
    if (!confirmed_.empty() && b.header.frame_id != stable_frame_) {
      try {
        map_stable = transform(buffer_.lookupTransform(b.header.frame_id, stable_frame_,
          b.header.stamp, rclcpp::Duration::from_seconds(0.1)).transform);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No matching base-map TF; publishing unchanged baseline: %s", e.what());
        fused_pub_->publish(b);
        auto empty = b; empty.data.assign(b.data.size(), -1); layer_pub_->publish(empty);
        return;
      }
    }
    const double res = b.info.resolution;
    int64_t min_x = 0, min_y = 0;
    int64_t max_x = int64_t(b.info.width) - 1, max_y = int64_t(b.info.height) - 1;
    std::vector<std::pair<int64_t, int64_t>> cells;
    for (const auto & point : confirmed_) {
      const auto p = map_stable * point;
      const double x = std::floor((p.x() - b.info.origin.position.x) / res);
      const double y = std::floor((p.y() - b.info.origin.position.y) / res);
      if (!std::isfinite(x) || !std::isfinite(y) ||
        std::abs(x) > double(max_cells_) || std::abs(y) > double(max_cells_)) {continue;}
      cells.emplace_back(int64_t(x), int64_t(y));
      min_x = std::min(min_x, int64_t(x)); max_x = std::max(max_x, int64_t(x));
      min_y = std::min(min_y, int64_t(y)); max_y = std::max(max_y, int64_t(y));
    }
    const uint64_t width = uint64_t(max_x - min_x + 1), height = uint64_t(max_y - min_y + 1);
    if (width > uint64_t(max_cells_) / height) {
      RCLCPP_ERROR(get_logger(), "Expanded fusion exceeds grid limit; publishing unchanged baseline.");
      fused_pub_->publish(b);
      auto empty = b; empty.data.assign(b.data.size(), -1); layer_pub_->publish(empty);
      return;
    }
    auto fused = b;
    fused.info.width = uint32_t(width); fused.info.height = uint32_t(height);
    fused.info.origin.position.x += double(min_x) * res;
    fused.info.origin.position.y += double(min_y) * res;
    fused.data.assign(size_t(width * height), -1);
    for (uint32_t row = 0; row < b.info.height; ++row) {
      std::copy_n(b.data.begin() + size_t(row) * b.info.width, b.info.width,
        fused.data.begin() + size_t(int64_t(row) - min_y) * width + size_t(-min_x));
    }
    auto layer = fused;
    layer.data.assign(fused.data.size(), -1);
    for (const auto & c : cells) {
      const size_t i = size_t(c.second - min_y) * width + size_t(c.first - min_x);
      fused.data[i] = 100; layer.data[i] = 100;
    }
    fused_pub_->publish(fused);
    layer_pub_->publish(layer);
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  double min_height_, max_height_, xy_size_, z_size_, view_separation_, bottom_height_;
  double max_gyro_, max_acc_spread_;
  double stable_duration_, stable_angle_, stable_offset_, imu_ground_angle_, plane_distance_;
  double height_tolerance_, plane_min_ratio_, ground_range_, check_period_, max_reference_age_;
  double monitor_angle_, monitor_offset_, ground_offset_ = 0;
  int stable_frames_, plane_min_points_, mismatch_confirmations_;
  int min_views_, imu_samples_;
  int64_t max_cells_, max_evidence_, last_stamp_ = 0;
  bool reference_ready_ = false, reference_verified_ = false, have_stamp_ = false;
  std::string stable_frame_, body_frame_;
  int64_t last_check_stamp_ = 0, last_ground_stamp_ = 0;
  size_t reference_epoch_ = 0;
  tf2::Vector3 bottom_in_body_, up_;
  std::deque<ImuSample> imu_window_;
  std::vector<ReferenceSample> reference_window_;
  std::vector<ReferenceSample> mismatch_window_;
  std::mt19937 random_{42};
  std::unordered_map<Key, Evidence, Hash> evidence_;
  std::vector<tf2::Vector3> confirmed_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr base_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_pub_, layer_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr base_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<RackObstacleLayer>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rack_obstacle_layer"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
