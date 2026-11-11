/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, ZZJ WHEELTEC project contributors.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 *********************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore_lite_msgs/msg/explore_status.hpp>

namespace ackermann_frontier_explorer
{

using ComputePath = nav2_msgs::action::ComputePathToPose;
using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
using GetState = lifecycle_msgs::srv::GetState;
using Navigate = nav2_msgs::action::NavigateToPose;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double pointDistance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

}  // namespace

struct Candidate
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Point frontier_center;
  double information_gain{0.0};
  double frontier_length{0.0};
  double clearance{0.0};
  double heading_error{0.0};
  double heuristic_score{0.0};
};

struct RankedFrontier
{
  frontier_exploration::Frontier frontier;
  double information_gain{0.0};
};

struct EvaluatedCandidate
{
  Candidate candidate;
  double score{std::numeric_limits<double>::infinity()};
  double path_length{0.0};
  double reverse_length{0.0};
  std::size_t gear_switches{0};
  double steering_effort{0.0};
};

struct FailureRecord
{
  geometry_msgs::msg::Point frontier_center;
  std::string reason;
  rclcpp::Time expires_at{0, 0, RCL_ROS_TIME};
  std::uint64_t map_revision{0};
  int attempts{1};
};

enum class NoReachableKind
{
  NoFrontiers,
  NoViewpoints,
  NoPaths,
  TemporarilySuppressed,
};

enum class ClearanceState
{
  Clear,
  Lethal,
  Transient,
};

struct ClearanceCheck
{
  ClearanceState state{ClearanceState::Transient};
  double measured_clearance{0.0};
};

class AckermannFrontierExplorer : public rclcpp::Node
{
public:
  AckermannFrontierExplorer()
  : Node("ackermann_frontier_explorer"),
    logger_(get_logger()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    costmap_client_(*this, &tf_buffer_)
  {
    declare_parameter<double>("planning_period", 5.0);
    declare_parameter<double>("post_goal_replan_delay", 1.0);
    declare_parameter<double>("evaluation_timeout", 40.0);
    declare_parameter<double>("progress_timeout", 35.0);
    declare_parameter<double>("progress_epsilon", 0.05);
    declare_parameter<double>("progress_yaw_epsilon", 0.08);
    declare_parameter<double>("map_freshness_timeout", 5.0);
    declare_parameter<double>("completion_map_stability_duration", 10.0);
    declare_parameter<double>("min_frontier_size", 0.45);
    declare_parameter<double>("candidate_standoff", 0.45);
    declare_parameter<double>("candidate_standoff_max", 0.75);
    declare_parameter<double>("footprint_clearance", 0.23);
    declare_parameter<double>("clearance_search_radius", 0.60);
    declare_parameter<double>("candidate_back_clearance_distance", 0.0);
    declare_parameter<double>("minimum_candidate_travel_distance", 0.50);
    declare_parameter<double>("candidate_inward_search_radius", 0.40);
    declare_parameter<int>("candidate_inward_angle_samples", 5);
    declare_parameter<double>("candidate_inward_angle_span", 1.0471975512);
    declare_parameter<std::string>(
      "global_costmap_topic", "/global_costmap/costmap_raw");
    declare_parameter<double>("global_costmap_freshness_timeout", 5.0);
    declare_parameter<std::string>(
      "local_costmap_topic", "/local_costmap/costmap_raw");
    declare_parameter<double>("local_costmap_freshness_timeout", 3.0);
    declare_parameter<int>("safety_transient_failure_cycles", 3);
    declare_parameter<double>("emergency_stop_clearance", 0.42);
    declare_parameter<double>("candidate_deduplication_distance", 0.20);
    declare_parameter<int>("candidate_angle_samples", 24);
    declare_parameter<int>("max_frontiers_to_consider", 20);
    declare_parameter<int>("max_viewpoints_per_frontier", 2);
    declare_parameter<int>("max_frontiers_to_evaluate", 6);
    declare_parameter<double>("blacklist_radius", 0.50);
    declare_parameter<double>("blacklist_ttl", 90.0);
    declare_parameter<int>("blacklist_map_revision_delta", 3);
    declare_parameter<double>("frontier_preselection_gain_weight", 0.8);
    declare_parameter<double>("information_gain_radius", 1.75);
    declare_parameter<double>("frontier_length_gain_weight", 0.35);
    declare_parameter<double>("candidate_heading_weight", 0.8);
    declare_parameter<double>("path_length_weight", 1.0);
    declare_parameter<double>("reverse_length_weight", 2.0);
    declare_parameter<double>("gear_switch_weight", 1.5);
    declare_parameter<double>("heading_change_weight", 0.4);
    declare_parameter<double>("steering_effort_weight", 0.2);
    declare_parameter<double>("clearance_weight", 0.25);
    declare_parameter<double>("information_gain_weight", 0.8);
    declare_parameter<double>("max_reverse_fraction", 1.0);
    declare_parameter<double>("max_reverse_length", 0.8);
    declare_parameter<int>("max_gear_switches", 4);
    declare_parameter<std::string>("planner_id", "GridBased");
    declare_parameter<int>("completion_stability_cycles", 3);
    declare_parameter<double>("completion_grace_period", 60.0);
    declare_parameter<double>("navigation_rejection_retry_delay", 5.0);
    declare_parameter<bool>("return_to_start", true);
    declare_parameter<double>("return_timeout", 120.0);
    declare_parameter<bool>("save_on_completion", true);
    declare_parameter<std::string>(
      "map_save_service", "/map_autosaver/save_now");
    declare_parameter<double>("map_save_service_timeout", 10.0);
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<int>("final_stop_repeat_count", 10);
    declare_parameter<int>("final_stop_repeat_interval_ms", 20);
    declare_parameter<bool>("exit_on_completion", true);
    declare_parameter<bool>("visualize", true);
    declare_parameter<bool>("dry_run", false);

    get_parameter("planning_period", planning_period_);
    get_parameter("post_goal_replan_delay", post_goal_replan_delay_);
    get_parameter("evaluation_timeout", evaluation_timeout_);
    get_parameter("progress_timeout", progress_timeout_);
    get_parameter("progress_epsilon", progress_epsilon_);
    get_parameter("progress_yaw_epsilon", progress_yaw_epsilon_);
    get_parameter("map_freshness_timeout", map_freshness_timeout_);
    get_parameter(
      "completion_map_stability_duration",
      completion_map_stability_duration_);
    get_parameter("min_frontier_size", min_frontier_size_);
    get_parameter("candidate_standoff", candidate_standoff_);
    get_parameter("candidate_standoff_max", candidate_standoff_max_);
    get_parameter("footprint_clearance", footprint_clearance_);
    get_parameter("clearance_search_radius", clearance_search_radius_);
    get_parameter(
      "candidate_back_clearance_distance",
      candidate_back_clearance_distance_);
    get_parameter(
      "minimum_candidate_travel_distance",
      minimum_candidate_travel_distance_);
    get_parameter(
      "candidate_inward_search_radius",
      candidate_inward_search_radius_);
    get_parameter(
      "candidate_inward_angle_samples",
      candidate_inward_angle_samples_);
    get_parameter(
      "candidate_inward_angle_span",
      candidate_inward_angle_span_);
    get_parameter(
      "global_costmap_topic", global_costmap_topic_);
    get_parameter(
      "global_costmap_freshness_timeout",
      global_costmap_freshness_timeout_);
    get_parameter("local_costmap_topic", local_costmap_topic_);
    get_parameter(
      "local_costmap_freshness_timeout",
      local_costmap_freshness_timeout_);
    get_parameter(
      "safety_transient_failure_cycles",
      safety_transient_failure_cycles_);
    get_parameter("emergency_stop_clearance", emergency_stop_clearance_);
    get_parameter(
      "candidate_deduplication_distance",
      candidate_deduplication_distance_);
    get_parameter("candidate_angle_samples", candidate_angle_samples_);
    get_parameter("max_frontiers_to_consider", max_frontiers_to_consider_);
    get_parameter("max_viewpoints_per_frontier", max_viewpoints_per_frontier_);
    get_parameter("max_frontiers_to_evaluate", max_frontiers_to_evaluate_);
    get_parameter("blacklist_radius", blacklist_radius_);
    get_parameter("blacklist_ttl", blacklist_ttl_);
    get_parameter("blacklist_map_revision_delta", blacklist_map_revision_delta_);
    get_parameter(
      "frontier_preselection_gain_weight",
      frontier_preselection_gain_weight_);
    get_parameter("information_gain_radius", information_gain_radius_);
    get_parameter(
      "frontier_length_gain_weight",
      frontier_length_gain_weight_);
    get_parameter("candidate_heading_weight", candidate_heading_weight_);
    get_parameter("path_length_weight", path_length_weight_);
    get_parameter("reverse_length_weight", reverse_length_weight_);
    get_parameter("gear_switch_weight", gear_switch_weight_);
    get_parameter("heading_change_weight", heading_change_weight_);
    get_parameter("steering_effort_weight", steering_effort_weight_);
    get_parameter("clearance_weight", clearance_weight_);
    get_parameter("information_gain_weight", information_gain_weight_);
    get_parameter("max_reverse_fraction", max_reverse_fraction_);
    get_parameter("max_reverse_length", max_reverse_length_);
    get_parameter("max_gear_switches", max_gear_switches_);
    get_parameter("planner_id", planner_id_);
    get_parameter("completion_stability_cycles", completion_stability_cycles_);
    get_parameter("completion_grace_period", completion_grace_period_);
    get_parameter(
      "navigation_rejection_retry_delay",
      navigation_rejection_retry_delay_);
    get_parameter("return_to_start", return_to_start_);
    get_parameter("return_timeout", return_timeout_);
    get_parameter("save_on_completion", save_on_completion_);
    get_parameter("map_save_service", map_save_service_);
    get_parameter("map_save_service_timeout", map_save_service_timeout_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("final_stop_repeat_count", final_stop_repeat_count_);
    get_parameter(
      "final_stop_repeat_interval_ms",
      final_stop_repeat_interval_ms_);
    get_parameter("exit_on_completion", exit_on_completion_);
    get_parameter("visualize", visualize_);
    get_parameter("dry_run", dry_run_);

    if (candidate_standoff_max_ < candidate_standoff_) {
      candidate_standoff_max_ = candidate_standoff_;
    }
    post_goal_replan_delay_ = std::max(post_goal_replan_delay_, 0.1);
    minimum_candidate_travel_distance_ =
      std::max(minimum_candidate_travel_distance_, 0.0);
    candidate_inward_search_radius_ =
      std::max(candidate_inward_search_radius_, 0.10);
    candidate_inward_angle_samples_ =
      std::max(candidate_inward_angle_samples_, 1);
    candidate_inward_angle_span_ =
      std::max(0.0, std::min(candidate_inward_angle_span_, kPi));
    global_costmap_freshness_timeout_ =
      std::max(global_costmap_freshness_timeout_, 1.0);
    local_costmap_freshness_timeout_ =
      std::max(local_costmap_freshness_timeout_, 0.5);
    safety_transient_failure_cycles_ =
      std::max(safety_transient_failure_cycles_, 1);
    emergency_stop_clearance_ =
      std::max(0.0, std::min(emergency_stop_clearance_, footprint_clearance_));
    candidate_angle_samples_ = std::max(candidate_angle_samples_, 8);
    max_frontiers_to_consider_ = std::max(max_frontiers_to_consider_, 1);
    max_viewpoints_per_frontier_ = std::max(max_viewpoints_per_frontier_, 1);
    max_frontiers_to_evaluate_ = std::max(max_frontiers_to_evaluate_, 1);
    completion_stability_cycles_ = std::max(completion_stability_cycles_, 1);
    completion_grace_period_ = std::max(completion_grace_period_, 0.0);
    navigation_rejection_retry_delay_ =
      std::max(navigation_rejection_retry_delay_, planning_period_);
    return_timeout_ = std::max(return_timeout_, planning_period_);
    map_save_service_timeout_ = std::max(map_save_service_timeout_, planning_period_);
    evaluation_timeout_ = std::max(evaluation_timeout_, planning_period_);
    map_freshness_timeout_ = std::max(map_freshness_timeout_, planning_period_);
    completion_map_stability_duration_ =
      std::max(completion_map_stability_duration_, 0.0);
    blacklist_ttl_ = std::max(blacklist_ttl_, planning_period_);
    blacklist_map_revision_delta_ = std::max(blacklist_map_revision_delta_, 1);
    information_gain_radius_ =
      std::max(information_gain_radius_, candidate_standoff_max_);
    frontier_length_gain_weight_ = std::max(frontier_length_gain_weight_, 0.0);
    candidate_heading_weight_ = std::max(candidate_heading_weight_, 0.0);
    max_reverse_fraction_ =
      std::max(0.0, std::min(max_reverse_fraction_, 1.0));
    max_reverse_length_ = std::max(max_reverse_length_, 0.0);
    max_gear_switches_ = std::max(max_gear_switches_, 0);
    final_stop_repeat_count_ = std::max(final_stop_repeat_count_, 1);
    final_stop_repeat_interval_ms_ =
      std::max(final_stop_repeat_interval_ms_, 1);

    frontier_search_ = frontier_exploration::FrontierSearch(
      costmap_client_.getCostmap(), 1.0, 1.0, min_frontier_size_, logger_);

    compute_path_client_ =
      rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    navigate_client_ =
      rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    controller_state_client_ =
      create_client<GetState>("/controller_server/get_state");
    planner_state_client_ =
      create_client<GetState>("/planner_server/get_state");
    navigator_state_client_ =
      create_client<GetState>("/bt_navigator/get_state");
    map_save_client_ =
      create_client<std_srvs::srv::Trigger>(map_save_service_);
    stop_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    global_costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      global_costmap_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav2_msgs::msg::Costmap::SharedPtr message) {
        latest_global_costmap_ = message;
        last_global_costmap_time_ = now();
      });
    local_costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      local_costmap_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav2_msgs::msg::Costmap::SharedPtr message) {
        latest_local_costmap_ = message;
        last_local_costmap_time_ = now();
      });

    auto status_qos = rclcpp::QoS(10).transient_local();
    status_pub_ = create_publisher<explore_lite_msgs::msg::ExploreStatus>(
      "explore/status", status_qos);
    resume_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          resume();
        } else {
          pause();
        }
      });

    if (visualize_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "explore/frontiers", 10);
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<std::int64_t>(planning_period_ * 1000.0)),
      std::bind(&AckermannFrontierExplorer::tick, this));
    safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AckermannFrontierExplorer::checkNavigationSafety, this));

    publishStatus(explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED);
    RCLCPP_INFO(
      logger_,
      "Ackermann frontier explorer started: hold goals until completion, "
      "evaluate at most %d diverse viewpoints with Smac planner '%s'; "
      "complete after %.0f s grace plus %d no-frontier cycles; "
      "existing but unreachable frontiers remain blocked, "
      "return-to-start %s%s",
      max_frontiers_to_evaluate_, planner_id_.c_str(),
      completion_grace_period_,
      completion_stability_cycles_,
      return_to_start_ ? "enabled" : "disabled",
      dry_run_ ? " (dry run: navigation disabled)" : "");
  }

  ~AckermannFrontierExplorer() override
  {
    if (navigate_client_ &&
      rclcpp::ok(get_node_base_interface()->get_context()))
    {
      navigate_client_->async_cancel_all_goals();
    }
  }

private:
  bool costmapIsFresh(
    const nav2_msgs::msg::Costmap::SharedPtr & costmap,
    const rclcpp::Time & received_time,
    double freshness_timeout) const
  {
    return
      costmap &&
      received_time.nanoseconds() != 0 &&
      now() - received_time <=
      rclcpp::Duration::from_seconds(freshness_timeout);
  }

  bool globalCostmapIsFresh() const
  {
    return costmapIsFresh(
      latest_global_costmap_, last_global_costmap_time_,
      global_costmap_freshness_timeout_);
  }

  bool localCostmapIsFresh() const
  {
    return costmapIsFresh(
      latest_local_costmap_, last_local_costmap_time_,
      local_costmap_freshness_timeout_);
  }

  ClearanceCheck costmapClearance(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::Point & point,
    double required_clearance,
    bool unknown_is_unsafe) const
  {
    ClearanceCheck result;
    const auto & metadata = costmap.metadata;
    if (metadata.resolution <= 0.0 ||
      metadata.size_x == 0 || metadata.size_y == 0 ||
      costmap.data.size() !=
      static_cast<std::size_t>(metadata.size_x) * metadata.size_y)
    {
      return result;
    }

    const double origin_yaw = tf2::getYaw(metadata.origin.orientation);
    const double cosine = std::cos(origin_yaw);
    const double sine = std::sin(origin_yaw);
    const double dx = point.x - metadata.origin.position.x;
    const double dy = point.y - metadata.origin.position.y;
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    const int center_x =
      static_cast<int>(std::floor(local_x / metadata.resolution));
    const int center_y =
      static_cast<int>(std::floor(local_y / metadata.resolution));
    if (center_x < 0 || center_y < 0 ||
      center_x >= static_cast<int>(metadata.size_x) ||
      center_y >= static_cast<int>(metadata.size_y))
    {
      return result;
    }

    const int radius_cells = static_cast<int>(
      std::ceil(required_clearance / metadata.resolution));
    double lethal_clearance =
      required_clearance + static_cast<double>(metadata.resolution);
    double unknown_clearance = lethal_clearance;
    for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y) {
      for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
        const double distance =
          std::hypot(
          static_cast<double>(offset_x),
          static_cast<double>(offset_y)) * metadata.resolution;
        if (distance > required_clearance) {
          continue;
        }

        const int cell_x = center_x + offset_x;
        const int cell_y = center_y + offset_y;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(metadata.size_x) ||
          cell_y >= static_cast<int>(metadata.size_y))
        {
          unknown_clearance = std::min(unknown_clearance, distance);
          continue;
        }

        const auto cost = costmap.data[
          static_cast<std::size_t>(cell_y) * metadata.size_x +
          static_cast<std::size_t>(cell_x)];
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          lethal_clearance = std::min(lethal_clearance, distance);
        } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
          unknown_clearance = std::min(unknown_clearance, distance);
        }
      }
    }

    if (lethal_clearance + 1e-6 < required_clearance) {
      result.state = ClearanceState::Lethal;
      result.measured_clearance = lethal_clearance;
      return result;
    }
    if (unknown_is_unsafe &&
      unknown_clearance + 1e-6 < required_clearance)
    {
      result.state = ClearanceState::Transient;
      result.measured_clearance = unknown_clearance;
      return result;
    }

    result.state = ClearanceState::Clear;
    result.measured_clearance = lethal_clearance;
    return result;
  }

  ClearanceCheck globalCostmapClearance(
    const geometry_msgs::msg::Point & point,
    double required_clearance) const
  {
    if (!globalCostmapIsFresh()) {
      return {};
    }
    if (!latest_global_costmap_->header.frame_id.empty() &&
      latest_global_costmap_->header.frame_id !=
      costmap_client_.getGlobalFrameID())
    {
      return {};
    }
    return costmapClearance(
      *latest_global_costmap_, point, required_clearance, true);
  }

  bool globalCostmapHasClearance(
    const geometry_msgs::msg::Point & point,
    double required_clearance,
    double & measured_clearance) const
  {
    const auto result = globalCostmapClearance(point, required_clearance);
    measured_clearance = result.measured_clearance;
    return result.state == ClearanceState::Clear;
  }

  ClearanceCheck localRobotClearance()
  {
    if (!localCostmapIsFresh()) {
      return {};
    }

    const auto & frame = latest_local_costmap_->header.frame_id;
    if (frame.empty()) {
      return {};
    }

    geometry_msgs::msg::Point robot_point;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame, costmap_client_.getBaseFrameID(), tf2::TimePointZero);
      robot_point.x = transform.transform.translation.x;
      robot_point.y = transform.transform.translation.y;
      robot_point.z = transform.transform.translation.z;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        logger_, *get_clock(), 2000,
        "Transient local-costmap robot transform failure: %s", error.what());
      return {};
    }

    return costmapClearance(
      *latest_local_costmap_, robot_point,
      emergency_stop_clearance_, true);
  }

  void cancelUnsafeNavigation(
    const std::string & reason,
    double measured_clearance,
    double required_clearance)
  {
    if (!navigating_ || returning_to_start_) {
      return;
    }

    RCLCPP_ERROR(
      logger_,
      "Canceling frontier navigation for safety: %s; measured clearance "
      "%.2f m, required %.2f m",
      reason.c_str(), measured_clearance, required_clearance);
    recordFailure(current_candidate_.frontier_center, reason);
    safety_transient_failure_count_ = 0;
    ++navigation_generation_;
    navigating_ = false;
    navigation_goal_handle_.reset();
    navigate_client_->async_cancel_all_goals();
    stop_pub_->publish(geometry_msgs::msg::Twist());
    navigation_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(planning_period_);
  }

  void handleTransientSafetyFailure(
    const std::string & reason,
    double measured_clearance,
    double required_clearance)
  {
    ++safety_transient_failure_count_;
    RCLCPP_WARN(
      logger_,
      "Transient navigation safety check failure %d/%d: %s; measured "
      "clearance %.2f m, required %.2f m",
      safety_transient_failure_count_, safety_transient_failure_cycles_,
      reason.c_str(), measured_clearance, required_clearance);
    if (safety_transient_failure_count_ >=
      safety_transient_failure_cycles_)
    {
      cancelUnsafeNavigation(
        "persistent transient safety failure: " + reason,
        measured_clearance, required_clearance);
    }
  }

  void checkNavigationSafety()
  {
    if (!navigating_ || returning_to_start_ || finalization_started_) {
      return;
    }

    const auto target_check = globalCostmapClearance(
      current_candidate_.pose.pose.position, footprint_clearance_);
    const auto robot_check = localRobotClearance();
    if (target_check.state == ClearanceState::Lethal) {
      cancelUnsafeNavigation(
        "selected viewpoint has a lethal obstacle in the live global costmap",
        target_check.measured_clearance, footprint_clearance_);
      return;
    }
    if (robot_check.state == ClearanceState::Lethal) {
      cancelUnsafeNavigation(
        "local costmap detected a lethal obstacle inside the robot safety "
        "envelope",
        robot_check.measured_clearance, emergency_stop_clearance_);
      return;
    }
    if (target_check.state == ClearanceState::Transient) {
      handleTransientSafetyFailure(
        "global target costmap is stale, invalid, or temporarily unknown",
        target_check.measured_clearance, footprint_clearance_);
      return;
    }
    if (robot_check.state == ClearanceState::Transient) {
      handleTransientSafetyFailure(
        "local robot costmap is stale, invalid, or temporarily unknown",
        robot_check.measured_clearance, emergency_stop_clearance_);
      return;
    }

    if (safety_transient_failure_count_ > 0) {
      RCLCPP_INFO(
        logger_,
        "Navigation safety data recovered after %d transient failure(s)",
        safety_transient_failure_count_);
      safety_transient_failure_count_ = 0;
    }
  }

  void tick()
  {
    if (finalization_started_) {
      if (final_save_pending_) {
        tryFinalMapSave();
      }
      return;
    }

    if (evaluating_) {
      checkEvaluationTimeout();
      return;
    }

    if (paused_ || navigating_) {
      if (navigating_) {
        checkProgressTimeout();
      }
      return;
    }

    if (!nav2_lifecycle_active_) {
      requestNav2LifecycleStates();
      return;
    }

    if (navigation_retry_not_before_.nanoseconds() != 0 &&
      now() < navigation_retry_not_before_)
    {
      RCLCPP_INFO_THROTTLE(
        logger_, *get_clock(), 5000,
        "Waiting before the next frontier evaluation");
      return;
    }

    if (!compute_path_client_->action_server_is_ready() ||
      !navigate_client_->action_server_is_ready())
    {
      RCLCPP_INFO_THROTTLE(
        logger_, *get_clock(), 5000,
        "Waiting for ComputePathToPose and NavigateToPose action servers");
      return;
    }

    if (!mapIsFresh()) {
      resetCompletionEvidence();
      RCLCPP_WARN_THROTTLE(
        logger_, *get_clock(), 5000,
        "Exploration held: /map has not been refreshed within %.1f s",
        map_freshness_timeout_);
      return;
    }

    startEvaluation();
  }

  void startEvaluation()
  {
    robot_pose_ = costmap_client_.getRobotPose();
    const double quaternion_norm =
      std::sqrt(
      robot_pose_.orientation.x * robot_pose_.orientation.x +
      robot_pose_.orientation.y * robot_pose_.orientation.y +
      robot_pose_.orientation.z * robot_pose_.orientation.z +
      robot_pose_.orientation.w * robot_pose_.orientation.w);
    if (quaternion_norm < 0.5) {
      recordSystemBlocked("robot pose transform is unavailable");
      return;
    }
    if (!globalCostmapIsFresh()) {
      recordSystemBlocked("live global target costmap is unavailable or stale");
      return;
    }
    const auto robot_check = localRobotClearance();
    if (robot_check.state != ClearanceState::Clear) {
      std::ostringstream reason;
      if (robot_check.state == ClearanceState::Lethal) {
        reason << "local costmap reports a lethal obstacle inside the robot "
               << "safety envelope ("
               << robot_check.measured_clearance << " m measured, "
               << emergency_stop_clearance_ << " m required)";
      } else {
        reason << "local robot costmap is unavailable, stale, invalid, or "
               << "temporarily unknown";
      }
      recordSystemBlocked(reason.str());
      return;
    }

    if (!initial_pose_recorded_) {
      initial_pose_.header.frame_id = costmap_client_.getGlobalFrameID();
      initial_pose_.header.stamp = now();
      initial_pose_.pose = robot_pose_;
      initial_pose_recorded_time_ = now();
      initial_pose_recorded_ = true;
      RCLCPP_INFO(
        logger_, "Recorded exploration start pose at (%.2f, %.2f)",
        initial_pose_.pose.position.x, initial_pose_.pose.position.y);
    }

    auto frontiers = frontier_search_.searchFrom(robot_pose_.position);
    const std::size_t detected_frontiers = frontiers.size();
    last_detected_frontiers_ = detected_frontiers;
    const double resolution = costmap_client_.getCostmap()->getResolution();

    // Frontier length alone cannot distinguish a narrow map crack from a
    // genuinely large unexplored room. Rank frontiers by the unknown map area
    // around them, with a smaller boundary-length bonus for broad openings.
    std::vector<RankedFrontier> ranked_frontiers;
    ranked_frontiers.reserve(frontiers.size());
    {
      auto * costmap = costmap_client_.getCostmap();
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *costmap->getMutex());
      for (const auto & frontier : frontiers) {
        RankedFrontier ranked;
        ranked.frontier = frontier;
        ranked.information_gain =
          estimateInformationGain(*costmap, frontier.centroid) +
          frontier_length_gain_weight_ *
          static_cast<double>(frontier.size) * resolution;
        ranked_frontiers.push_back(std::move(ranked));
      }
    }
    std::sort(
      ranked_frontiers.begin(), ranked_frontiers.end(),
      [this](
        const RankedFrontier & first,
        const RankedFrontier & second)
      {
        return
        first.frontier.min_distance -
        frontier_preselection_gain_weight_ * first.information_gain <
        second.frontier.min_distance -
        frontier_preselection_gain_weight_ * second.information_gain;
      });

    candidates_.clear();
    evaluated_.clear();
    evaluation_index_ = 0;
    pruneFailureRecords();

    int considered_frontiers = 0;
    std::size_t blacklisted_frontiers = 0;
    for (const auto & ranked : ranked_frontiers) {
      if (considered_frontiers >= max_frontiers_to_consider_) {
        break;
      }
      const auto & frontier = ranked.frontier;
      if (isBlacklisted(frontier.centroid)) {
        ++blacklisted_frontiers;
        continue;
      }

      ++considered_frontiers;
      makeCandidates(
        frontier, ranked.information_gain, footprint_clearance_,
        candidate_back_clearance_distance_, candidates_);
    }

    const auto hybrid_budget =
      static_cast<std::size_t>(max_frontiers_to_evaluate_);

    std::sort(
      candidates_.begin(), candidates_.end(),
      [](const Candidate & first, const Candidate & second) {
        return first.heuristic_score < second.heuristic_score;
      });

    // Prefer one viewpoint per frontier before admitting a second viewing
    // angle. This keeps the expensive Hybrid-A* budget geographically diverse
    // while still allowing multi-angle retries when only a few frontiers exist.
    std::vector<Candidate> shortlisted;
    shortlisted.reserve(std::min(hybrid_budget, candidates_.size()));
    for (const auto & candidate : candidates_) {
      const bool frontier_already_selected = std::any_of(
        shortlisted.begin(), shortlisted.end(),
        [&candidate, resolution](const Candidate & existing) {
          return
          pointDistance(
            existing.frontier_center, candidate.frontier_center) <=
          resolution;
        });
      if (!frontier_already_selected) {
        shortlisted.push_back(candidate);
      }
      if (shortlisted.size() >= hybrid_budget) {
        break;
      }
    }
    if (shortlisted.size() < hybrid_budget) {
      for (const auto & candidate : candidates_) {
        const bool viewpoint_already_selected = std::any_of(
          shortlisted.begin(), shortlisted.end(),
          [&candidate](const Candidate & existing) {
            return
            pointDistance(
              existing.pose.pose.position,
              candidate.pose.pose.position) < 1e-6;
          });
        if (!viewpoint_already_selected) {
          shortlisted.push_back(candidate);
        }
        if (shortlisted.size() >= hybrid_budget) {
          break;
        }
      }
    }
    candidates_.swap(shortlisted);

    visualize(frontiers, candidates_, nullptr);
    RCLCPP_INFO(
      logger_,
      "Frontier evaluation: detected %zu, temporarily suppressed %zu, "
      "cheap-filtered frontiers %d, Hybrid candidates %zu, map revision %llu",
      detected_frontiers, blacklisted_frontiers, considered_frontiers,
      candidates_.size(),
      static_cast<unsigned long long>(costmap_client_.getMapRevision()));

    if (candidates_.empty()) {
      if (ranked_frontiers.empty()) {
        recordNoReachableFrontier(
          NoReachableKind::NoFrontiers,
          "no frontier cells detected");
      } else if (blacklisted_frontiers == ranked_frontiers.size() ||
        considered_frontiers == 0)
      {
        recordNoReachableFrontier(
          NoReachableKind::TemporarilySuppressed,
          "all detected frontiers are in the navigation blacklist");
      } else {
        recordNoReachableFrontier(
          NoReachableKind::NoViewpoints,
          "no valid known-free viewpoint for " +
          std::to_string(considered_frontiers) +
          " cheap-filtered frontiers");
      }
      return;
    }

    evaluating_ = true;
    ++evaluation_generation_;
    evaluation_deadline_ =
      now() + rclcpp::Duration::from_seconds(evaluation_timeout_);
    requestNextPath(evaluation_generation_);
  }

  std::vector<geometry_msgs::msg::Point> sampleFrontierAnchors(
    const frontier_exploration::Frontier & frontier,
    const nav2_costmap_2d::Costmap2D & costmap) const
  {
    std::vector<geometry_msgs::msg::Point> anchors;
    if (frontier.points.empty()) {
      return anchors;
    }

    // A cluster centroid can lie near the robot, inside an obstacle, or far
    // from a curved/ring-shaped frontier. Always seed from a real boundary.
    const auto closest = std::min_element(
      frontier.points.begin(), frontier.points.end(),
      [this](
        const geometry_msgs::msg::Point & first,
        const geometry_msgs::msg::Point & second)
      {
        return
        pointDistance(first, robot_pose_.position) <
        pointDistance(second, robot_pose_.position);
      });
    anchors.push_back(*closest);

    // Farthest-point sampling covers the frontier geometry without relying on
    // the BFS insertion order of its grid cells.
    const std::size_t maximum_anchors = std::min(
      static_cast<std::size_t>(candidate_angle_samples_),
      frontier.points.size());
    const double minimum_anchor_separation = std::max(
      2.0 * costmap.getResolution(),
      candidate_deduplication_distance_);
    while (anchors.size() < maximum_anchors) {
      const geometry_msgs::msg::Point * best_point = nullptr;
      double best_separation = -1.0;
      for (const auto & point : frontier.points) {
        double nearest_anchor = std::numeric_limits<double>::infinity();
        for (const auto & anchor : anchors) {
          nearest_anchor = std::min(
            nearest_anchor, pointDistance(point, anchor));
        }
        if (nearest_anchor > best_separation) {
          best_separation = nearest_anchor;
          best_point = &point;
        }
      }
      if (!best_point ||
        best_separation + 1e-6 < minimum_anchor_separation)
      {
        break;
      }
      anchors.push_back(*best_point);
    }
    return anchors;
  }

  bool estimateInwardDirection(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::Point & anchor,
    double & direction_x,
    double & direction_y) const
  {
    unsigned int anchor_x = 0;
    unsigned int anchor_y = 0;
    if (!costmap.worldToMap(anchor.x, anchor.y, anchor_x, anchor_y)) {
      return false;
    }

    const double resolution = costmap.getResolution();
    const int radius_cells = static_cast<int>(
      std::ceil(candidate_inward_search_radius_ / resolution));
    double vector_x = 0.0;
    double vector_y = 0.0;
    double total_weight = 0.0;

    // The weighted center of nearby known-free cells is on the explored side
    // of the boundary and gives a local inward normal.
    for (int offset_y = -radius_cells;
      offset_y <= radius_cells; ++offset_y)
    {
      for (int offset_x = -radius_cells;
        offset_x <= radius_cells; ++offset_x)
      {
        const double cell_distance =
          std::hypot(
          static_cast<double>(offset_x),
          static_cast<double>(offset_y)) * resolution;
        if (cell_distance < 0.5 * resolution ||
          cell_distance > candidate_inward_search_radius_)
        {
          continue;
        }
        const int cell_x = static_cast<int>(anchor_x) + offset_x;
        const int cell_y = static_cast<int>(anchor_y) + offset_y;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(costmap.getSizeInCellsX()) ||
          cell_y >= static_cast<int>(costmap.getSizeInCellsY()) ||
          costmap.getCost(
            static_cast<unsigned int>(cell_x),
            static_cast<unsigned int>(cell_y)) !=
          nav2_costmap_2d::FREE_SPACE)
        {
          continue;
        }

        double world_x = 0.0;
        double world_y = 0.0;
        costmap.mapToWorld(
          static_cast<unsigned int>(cell_x),
          static_cast<unsigned int>(cell_y), world_x, world_y);
        const double weight = std::max(
          resolution,
          candidate_inward_search_radius_ - cell_distance + resolution);
        vector_x += weight * (world_x - anchor.x);
        vector_y += weight * (world_y - anchor.y);
        total_weight += weight;
      }
    }

    if (total_weight > 0.0) {
      vector_x /= total_weight;
      vector_y /= total_weight;
    }
    double norm = std::hypot(vector_x, vector_y);
    if (norm < 0.5 * resolution) {
      // Around a small unknown hole the local vectors may cancel. The robot
      // lies in connected known-free space, so use it only as a direction
      // estimate; the candidate still has to pass every strict safety check.
      vector_x = robot_pose_.position.x - anchor.x;
      vector_y = robot_pose_.position.y - anchor.y;
      norm = std::hypot(vector_x, vector_y);
    }
    if (norm < 1e-6) {
      return false;
    }

    direction_x = vector_x / norm;
    direction_y = vector_y / norm;
    return true;
  }

  void makeCandidates(
    const frontier_exploration::Frontier & frontier,
    double information_gain,
    double required_clearance,
    double back_clearance_distance,
    std::vector<Candidate> & output)
  {
    auto * costmap = costmap_client_.getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *costmap->getMutex());

    const double resolution = costmap->getResolution();
    const double radial_step = std::max(2.0 * resolution, 0.10);
    std::vector<Candidate> frontier_candidates;
    const auto anchors = sampleFrontierAnchors(frontier, *costmap);

    const auto try_candidate =
      [this, &frontier, information_gain, required_clearance,
        back_clearance_distance, resolution, costmap,
        &frontier_candidates](
      const geometry_msgs::msg::Point & anchor,
      double angle,
      double radius)
      {
        const double x = anchor.x + radius * std::cos(angle);
        const double y = anchor.y + radius * std::sin(angle);
        const double robot_distance =
          std::hypot(x - robot_pose_.position.x, y - robot_pose_.position.y);
        if (robot_distance < minimum_candidate_travel_distance_) {
          return;
        }

        geometry_msgs::msg::Point candidate_point;
        candidate_point.x = x;
        candidate_point.y = y;
        const bool duplicate = std::any_of(
          frontier_candidates.begin(), frontier_candidates.end(),
          [resolution, &candidate_point](const Candidate & existing) {
            return pointDistance(
              existing.pose.pose.position, candidate_point) < resolution;
          });
        if (duplicate) {
          return;
        }

        double clearance = 0.0;
        if (!knownFreeClearance(
            *costmap, x, y, required_clearance, clearance))
        {
          return;
        }

        // Face the actual boundary point that generated this viewpoint.
        const double yaw = std::atan2(anchor.y - y, anchor.x - x);
        if (back_clearance_distance > 0.0) {
          const double back_x =
            x - back_clearance_distance * std::cos(yaw);
          const double back_y =
            y - back_clearance_distance * std::sin(yaw);
          double back_clearance = 0.0;
          if (!knownFreeClearance(
              *costmap, back_x, back_y, required_clearance, back_clearance))
          {
            return;
          }
          clearance = std::min(clearance, back_clearance);
        }

        const double robot_yaw = tf2::getYaw(robot_pose_.orientation);
        const double travel_heading =
          std::atan2(
          y - robot_pose_.position.y,
          x - robot_pose_.position.x);
        const double heading_error =
          std::abs(normalizeAngle(travel_heading - robot_yaw));
        Candidate candidate;
        candidate.pose.header.frame_id = costmap_client_.getGlobalFrameID();
        candidate.pose.header.stamp = now();
        candidate.pose.pose.position = candidate_point;
        candidate.pose.pose.position.z = 0.0;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        candidate.pose.pose.orientation = tf2::toMsg(orientation);
        candidate.frontier_center = frontier.centroid;
        candidate.information_gain = information_gain;
        candidate.frontier_length =
          static_cast<double>(frontier.size) * resolution;
        candidate.clearance = clearance;
        candidate.heading_error = heading_error;
        candidate.heuristic_score =
          robot_distance -
          frontier_preselection_gain_weight_ * candidate.information_gain -
          0.5 * clearance +
          candidate_heading_weight_ * candidate.heading_error;
        frontier_candidates.push_back(std::move(candidate));
      };

    // Search from each real boundary anchor toward nearby known-free space.
    for (const auto & anchor : anchors) {
      double inward_x = 0.0;
      double inward_y = 0.0;
      if (!estimateInwardDirection(
          *costmap, anchor, inward_x, inward_y))
      {
        continue;
      }
      const double inward_angle = std::atan2(inward_y, inward_x);
      for (int sample = 0;
        sample < candidate_inward_angle_samples_; ++sample)
      {
        const double offset =
          candidate_inward_angle_samples_ == 1 ? 0.0 :
          -candidate_inward_angle_span_ +
          2.0 * candidate_inward_angle_span_ *
          static_cast<double>(sample) /
          static_cast<double>(candidate_inward_angle_samples_ - 1);
        for (double radius = candidate_standoff_;
          radius <= candidate_standoff_max_ + 1e-6;
          radius += radial_step)
        {
          try_candidate(anchor, inward_angle + offset, radius);
        }
      }
    }

    // Ambiguous normals get a complete annulus fallback around the real
    // boundary anchors. Every existing safety and Hybrid-A* check remains.
    if (frontier_candidates.empty()) {
      for (const auto & anchor : anchors) {
        for (int sample = 0; sample < candidate_angle_samples_; ++sample) {
          const double angle =
            2.0 * kPi * static_cast<double>(sample) /
            static_cast<double>(candidate_angle_samples_);
          for (double radius = candidate_standoff_;
            radius <= candidate_standoff_max_ + 1e-6;
            radius += radial_step)
          {
            try_candidate(anchor, angle, radius);
          }
        }
      }
    }

    std::sort(
      frontier_candidates.begin(), frontier_candidates.end(),
      [](const Candidate & first, const Candidate & second) {
        return first.heuristic_score < second.heuristic_score;
      });

    int selected = 0;
    for (const auto & candidate : frontier_candidates) {
      const bool duplicate = std::any_of(
        output.begin(), output.end(),
        [this, &candidate](const Candidate & existing) {
          return
          pointDistance(
            existing.pose.pose.position,
            candidate.pose.pose.position) <
          candidate_deduplication_distance_;
        });
      if (duplicate) {
        continue;
      }
      output.push_back(candidate);
      ++selected;
      if (selected >= max_viewpoints_per_frontier_) {
        break;
      }
    }
  }

  double estimateInformationGain(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::Point & center) const
  {
    unsigned int center_x = 0;
    unsigned int center_y = 0;
    if (!costmap.worldToMap(center.x, center.y, center_x, center_y)) {
      return 0.0;
    }

    const double resolution = costmap.getResolution();
    const int radius_cells =
      static_cast<int>(std::ceil(information_gain_radius_ / resolution));
    std::size_t unknown_cells = 0;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
          resolution > information_gain_radius_)
        {
          continue;
        }
        const int cell_x = static_cast<int>(center_x) + dx;
        const int cell_y = static_cast<int>(center_y) + dy;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(costmap.getSizeInCellsX()) ||
          cell_y >= static_cast<int>(costmap.getSizeInCellsY()))
        {
          continue;
        }
        if (costmap.getCost(
            static_cast<unsigned int>(cell_x),
            static_cast<unsigned int>(cell_y)) ==
          nav2_costmap_2d::NO_INFORMATION)
        {
          ++unknown_cells;
        }
      }
    }

    return static_cast<double>(unknown_cells) * resolution * resolution;
  }

  bool knownFreeClearance(
    const nav2_costmap_2d::Costmap2D & costmap,
    double wx,
    double wy,
    double required_clearance,
    double & clearance) const
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(wx, wy, mx, my)) {
      return false;
    }
    if (costmap.getCost(mx, my) != nav2_costmap_2d::FREE_SPACE) {
      return false;
    }

    const double resolution = costmap.getResolution();
    const int cells =
      static_cast<int>(std::ceil(clearance_search_radius_ / resolution));
    double nearest_non_free = clearance_search_radius_;

    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        const double distance = std::hypot(
          static_cast<double>(dx), static_cast<double>(dy)) * resolution;
        if (distance > clearance_search_radius_) {
          continue;
        }

        const int cell_x = static_cast<int>(mx) + dx;
        const int cell_y = static_cast<int>(my) + dy;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(costmap.getSizeInCellsX()) ||
          cell_y >= static_cast<int>(costmap.getSizeInCellsY()))
        {
          nearest_non_free = std::min(nearest_non_free, distance);
          continue;
        }

        const auto cost = costmap.getCost(
          static_cast<unsigned int>(cell_x),
          static_cast<unsigned int>(cell_y));
        if (cost != nav2_costmap_2d::FREE_SPACE) {
          nearest_non_free = std::min(nearest_non_free, distance);
        }
      }
    }

    clearance = nearest_non_free;
    return clearance + 1e-6 >= required_clearance;
  }

  bool mapIsFresh() const
  {
    const auto last_message = costmap_client_.getLastMapMessageTime();
    return
      last_message.nanoseconds() != 0 &&
      now() - last_message <=
      rclcpp::Duration::from_seconds(map_freshness_timeout_);
  }

  bool mapIsStable() const
  {
    const auto last_change = costmap_client_.getLastMapChangeTime();
    return
      last_change.nanoseconds() != 0 &&
      now() - last_change >=
      rclcpp::Duration::from_seconds(completion_map_stability_duration_);
  }

  void resetCompletionEvidence()
  {
    no_frontier_cycles_ = 0;
    completion_evidence_map_revision_ = costmap_client_.getMapRevision();
    completion_evidence_initialized_ = false;
  }

  void recordSystemBlocked(const std::string & reason)
  {
    evaluating_ = false;
    resetCompletionEvidence();
    navigation_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(planning_period_);
    RCLCPP_WARN(
      logger_,
      "Exploration held because navigation is not currently plannable: %s. "
      "This condition is not completion evidence.",
      reason.c_str());
  }

  void pruneFailureRecords()
  {
    const auto current_revision = costmap_client_.getMapRevision();
    const auto current_time = now();
    failure_records_.erase(
      std::remove_if(
        failure_records_.begin(), failure_records_.end(),
        [this, current_revision, current_time](const FailureRecord & record) {
          const bool expired_by_time = current_time >= record.expires_at;
          const bool expired_by_map =
          current_revision >= record.map_revision +
          static_cast<std::uint64_t>(blacklist_map_revision_delta_);
          return expired_by_time || expired_by_map;
        }),
      failure_records_.end());
  }

  bool isBlacklisted(const geometry_msgs::msg::Point & frontier) const
  {
    return std::any_of(
      failure_records_.begin(), failure_records_.end(),
      [this, &frontier](const FailureRecord & blocked) {
        return
        pointDistance(blocked.frontier_center, frontier) <=
        blacklist_radius_;
      });
  }

  void recordFailure(
    const geometry_msgs::msg::Point & frontier,
    const std::string & reason)
  {
    const auto existing = std::find_if(
      failure_records_.begin(), failure_records_.end(),
      [this, &frontier](const FailureRecord & record) {
        return
        pointDistance(record.frontier_center, frontier) <=
        blacklist_radius_;
      });
    if (existing != failure_records_.end()) {
      ++existing->attempts;
      existing->reason = reason;
      existing->expires_at =
        now() + rclcpp::Duration::from_seconds(blacklist_ttl_);
      existing->map_revision = costmap_client_.getMapRevision();
      return;
    }

    FailureRecord record;
    record.frontier_center = frontier;
    record.reason = reason;
    record.expires_at =
      now() + rclcpp::Duration::from_seconds(blacklist_ttl_);
    record.map_revision = costmap_client_.getMapRevision();
    failure_records_.push_back(record);
  }

  void checkEvaluationTimeout()
  {
    if (!evaluating_ || now() <= evaluation_deadline_) {
      return;
    }

    ++evaluation_generation_;
    evaluating_ = false;
    compute_path_client_->async_cancel_all_goals();
    recordSystemBlocked(
      "frontier path evaluation exceeded " +
      std::to_string(evaluation_timeout_) + " seconds");
  }

  void requestNextPath(std::uint64_t generation)
  {
    if (!evaluating_ || generation != evaluation_generation_) {
      return;
    }
    if (!globalCostmapIsFresh()) {
      ++evaluation_generation_;
      recordSystemBlocked(
        "live global costmap became stale during frontier evaluation");
      return;
    }
    if (evaluation_index_ >= candidates_.size()) {
      finishEvaluation(generation);
      return;
    }

    const std::size_t index = evaluation_index_;
    double live_clearance = 0.0;
    if (!globalCostmapHasClearance(
        candidates_[index].pose.pose.position,
        footprint_clearance_, live_clearance))
    {
      RCLCPP_WARN(
        logger_,
        "Skipping candidate %zu/%zu: live global costmap clearance %.2f m "
        "is below the required %.2f m",
        index + 1, candidates_.size(), live_clearance, footprint_clearance_);
      ++evaluation_index_;
      requestNextPath(generation);
      return;
    }

    ComputePath::Goal goal;
    goal.goal = candidates_[index].pose;
    goal.planner_id = planner_id_;
    goal.use_start = false;
    RCLCPP_INFO(
      logger_,
      "Requesting Hybrid-A* path %zu/%zu to viewpoint (%.2f, %.2f)",
      index + 1, candidates_.size(),
      candidates_[index].pose.pose.position.x,
      candidates_[index].pose.pose.position.y);

    auto options =
      rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation, index](
      const ComputePathGoalHandle::SharedPtr & goal_handle)
      {
        if (generation != evaluation_generation_ || index != evaluation_index_) {
          return;
        }
        if (!goal_handle) {
          RCLCPP_INFO(
            logger_, "Planner rejected frontier candidate %zu", index + 1);
          ++evaluation_index_;
          requestNextPath(generation);
        }
      };
    options.result_callback =
      [this, generation, index](
      const ComputePathGoalHandle::WrappedResult & result)
      {
        if (generation != evaluation_generation_ || index != evaluation_index_) {
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->path.poses.size() >= 2)
        {
          auto evaluated =
            scorePath(candidates_[index], result.result->path);
          if (std::isfinite(evaluated.score)) {
            RCLCPP_INFO(
              logger_,
              "Candidate %zu feasible: path %.2f m, reverse %.0f%%, "
              "gear switches %zu, steering effort %.2f rad, "
              "unknown gain %.2f m^2, score %.2f",
              index + 1, evaluated.path_length,
              evaluated.path_length > 1e-6 ?
              100.0 * evaluated.reverse_length / evaluated.path_length : 0.0,
              evaluated.gear_switches, evaluated.steering_effort,
              evaluated.candidate.information_gain, evaluated.score);
            evaluated_.push_back(evaluated);
          } else {
            RCLCPP_INFO(
              logger_,
              "Candidate %zu rejected by Ackermann path limits: "
              "reverse %.2f m (%.0f%%), gear switches %zu",
              index + 1,
              evaluated.reverse_length,
              evaluated.path_length > 1e-6 ?
              100.0 * evaluated.reverse_length / evaluated.path_length : 0.0,
              evaluated.gear_switches);
          }
        } else {
          RCLCPP_INFO(
            logger_, "No valid Hybrid-A* path to candidate %zu", index + 1);
        }
        ++evaluation_index_;
        requestNextPath(generation);
      };

    compute_path_client_->async_send_goal(goal, options);
  }

  EvaluatedCandidate scorePath(
    const Candidate & candidate,
    const nav_msgs::msg::Path & path) const
  {
    EvaluatedCandidate evaluated;
    evaluated.candidate = candidate;

    bool have_direction = false;
    bool previous_reverse = false;
    double initial_heading_change = 0.0;
    const double robot_yaw = tf2::getYaw(robot_pose_.orientation);

    for (std::size_t index = 1; index < path.poses.size(); ++index) {
      const auto & previous = path.poses[index - 1].pose;
      const auto & current = path.poses[index].pose;
      const double dx = current.position.x - previous.position.x;
      const double dy = current.position.y - previous.position.y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length < 1e-4) {
        continue;
      }

      evaluated.path_length += segment_length;
      const double path_yaw = tf2::getYaw(previous.orientation);
      const double next_path_yaw = tf2::getYaw(current.orientation);
      evaluated.steering_effort +=
        std::abs(normalizeAngle(next_path_yaw - path_yaw));
      const bool reverse =
        std::cos(path_yaw) * dx + std::sin(path_yaw) * dy < 0.0;
      if (reverse) {
        evaluated.reverse_length += segment_length;
      }

      if (!have_direction) {
        have_direction = true;
        previous_reverse = reverse;
        const double travel_yaw = std::atan2(dy, dx);
        initial_heading_change =
          std::abs(normalizeAngle(travel_yaw - robot_yaw));
      } else if (reverse != previous_reverse) {
        ++evaluated.gear_switches;
        previous_reverse = reverse;
      }
    }

    const double reverse_fraction =
      evaluated.path_length > 1e-6 ?
      evaluated.reverse_length / evaluated.path_length : 0.0;
    if (reverse_fraction > max_reverse_fraction_ ||
      evaluated.reverse_length > max_reverse_length_ ||
      evaluated.gear_switches >
      static_cast<std::size_t>(max_gear_switches_))
    {
      return evaluated;
    }

    evaluated.score =
      path_length_weight_ * evaluated.path_length +
      reverse_length_weight_ * evaluated.reverse_length +
      gear_switch_weight_ * static_cast<double>(evaluated.gear_switches) +
      heading_change_weight_ * initial_heading_change +
      steering_effort_weight_ * evaluated.steering_effort +
      clearance_weight_ / std::max(candidate.clearance, 0.05) -
      information_gain_weight_ * candidate.information_gain;
    return evaluated;
  }

  void recordNoReachableFrontier(
    NoReachableKind kind,
    const std::string & reason)
  {
    if (kind == NoReachableKind::TemporarilySuppressed) {
      resetCompletionEvidence();
      RCLCPP_WARN(
        logger_,
        "%s; waiting for failure records to expire or the map to change. "
        "This condition is not completion evidence.",
        reason.c_str());
      return;
    }

    // Existing frontiers are direct evidence that the map is unfinished.
    // A strict Ackermann viewpoint or planner can be temporarily infeasible
    // because the initial known-free patch is small, a door is occluded, or
    // the live costmap is conservative.  None of those conditions is valid
    // completion evidence, regardless of how many times it repeats.
    if (kind != NoReachableKind::NoFrontiers) {
      resetCompletionEvidence();
      RCLCPP_WARN(
        logger_,
        "%s, while %zu frontier cluster(s) remain. Treating this as a "
        "temporary planning blockage, never exploration completion.",
        reason.c_str(), last_detected_frontiers_);
      return;
    }

    if (initial_pose_recorded_) {
      const double elapsed =
        (now() - initial_pose_recorded_time_).seconds();
      if (elapsed < completion_grace_period_) {
        resetCompletionEvidence();
        RCLCPP_WARN(
          logger_,
          "%s; completion suppressed for another %.1f s during startup grace",
          reason.c_str(), completion_grace_period_ - elapsed);
        return;
      }
    }

    if (!mapIsStable()) {
      resetCompletionEvidence();
      RCLCPP_WARN(
        logger_,
        "%s; completion held until the occupancy map is unchanged for %.1f s",
        reason.c_str(), completion_map_stability_duration_);
      return;
    }

    const auto current_revision = costmap_client_.getMapRevision();
    if (!completion_evidence_initialized_ ||
      current_revision != completion_evidence_map_revision_ ||
      kind != completion_evidence_kind_)
    {
      no_frontier_cycles_ = 0;
      completion_evidence_map_revision_ = current_revision;
      completion_evidence_kind_ = kind;
      completion_evidence_initialized_ = true;
    }

    ++no_frontier_cycles_;
    RCLCPP_WARN(
      logger_, "%s (%d/%d stable cycles)",
      reason.c_str(), no_frontier_cycles_, completion_stability_cycles_);
    if (no_frontier_cycles_ >= completion_stability_cycles_) {
      beginCompletionSequence(reason);
    }
  }

  void finishEvaluation(std::uint64_t generation)
  {
    if (generation != evaluation_generation_) {
      return;
    }
    evaluating_ = false;

    if (evaluated_.empty()) {
      recordNoReachableFrontier(
        NoReachableKind::NoPaths,
        "Smac Hybrid-A* found no feasible path to any frontier viewpoint");
      return;
    }

    resetCompletionEvidence();
    const auto best = std::min_element(
      evaluated_.begin(), evaluated_.end(),
      [](const EvaluatedCandidate & first, const EvaluatedCandidate & second) {
        return first.score < second.score;
      });
    current_candidate_ = best->candidate;
    visualize({}, candidates_, &current_candidate_);

    RCLCPP_INFO(
      logger_,
      "Selected frontier viewpoint (%.2f, %.2f), score %.2f, path %.2f m, "
      "reverse %.2f m, gear switches %zu, steering %.2f rad, "
      "clearance %.2f m, unknown gain %.2f m^2, "
      "heading error %.0f deg",
      best->candidate.pose.pose.position.x,
      best->candidate.pose.pose.position.y,
      best->score,
      best->path_length,
      best->reverse_length,
      best->gear_switches,
      best->steering_effort,
      best->candidate.clearance,
      best->candidate.information_gain,
      best->candidate.heading_error * 180.0 / kPi);
    if (dry_run_) {
      RCLCPP_INFO(
        logger_,
        "Dry run complete: selected viewpoint was not sent to NavigateToPose");
      paused_ = true;
      publishStatus(explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED);
      return;
    }
    sendNavigationGoal(current_candidate_);
  }

  void sendNavigationGoal(const Candidate & candidate)
  {
    double live_clearance = 0.0;
    if (!globalCostmapHasClearance(
        candidate.pose.pose.position,
        footprint_clearance_, live_clearance))
    {
      RCLCPP_ERROR(
        logger_,
        "Refusing selected frontier goal: live global costmap clearance "
        "%.2f m is below the required %.2f m",
        live_clearance, footprint_clearance_);
      recordFailure(
        candidate.frontier_center,
        "selected viewpoint failed final live costmap safety validation");
      navigation_retry_not_before_ =
        now() + rclcpp::Duration::from_seconds(planning_period_);
      return;
    }

    Navigate::Goal goal;
    goal.pose = candidate.pose;

    ++navigation_generation_;
    const auto generation = navigation_generation_;
    navigating_ = true;
    safety_transient_failure_count_ = 0;
    best_distance_remaining_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = now();
    last_progress_pose_ = robot_pose_;

    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](
      const NavigateGoalHandle::SharedPtr & goal_handle)
      {
        if (generation != navigation_generation_) {
          return;
        }
        navigation_goal_handle_ = goal_handle;
        if (!goal_handle) {
          navigating_ = false;
          navigation_goal_handle_.reset();
          nav2_lifecycle_active_ = false;
          resetCompletionEvidence();
          navigation_retry_not_before_ =
            now() + rclcpp::Duration::from_seconds(
            navigation_rejection_retry_delay_);
          RCLCPP_WARN(
            logger_,
            "NavigateToPose rejected selected frontier; keeping it eligible "
            "and retrying after Nav2 lifecycle revalidation");
        }
      };
    options.feedback_callback =
      [this, generation](
      NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const Navigate::Feedback> feedback)
      {
        if (generation != navigation_generation_ || !feedback) {
          return;
        }
        if (feedback->distance_remaining + progress_epsilon_ <
          best_distance_remaining_)
        {
          best_distance_remaining_ = feedback->distance_remaining;
          last_progress_time_ = now();
        }
      };
    options.result_callback =
      [this, generation](
      const NavigateGoalHandle::WrappedResult & result)
      {
        if (generation != navigation_generation_) {
          return;
        }
        navigating_ = false;
        navigation_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(logger_, "Frontier viewpoint reached");
          resetCompletionEvidence();
          schedulePostGoalEvaluation();
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_WARN(
            logger_,
            "Frontier navigation aborted; temporarily suppressing region");
          recordFailure(
            current_candidate_.frontier_center, "NavigateToPose aborted");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_INFO(logger_, "Frontier navigation canceled");
        } else {
          RCLCPP_WARN(logger_, "Frontier navigation returned an unknown result");
          recordFailure(
            current_candidate_.frontier_center,
            "NavigateToPose returned unknown result");
        }
      };

    navigate_client_->async_send_goal(goal, options);
  }

  void schedulePostGoalEvaluation()
  {
    navigation_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(post_goal_replan_delay_);
    RCLCPP_INFO(
      logger_,
      "Next frontier evaluation scheduled %.1f s after goal completion",
      post_goal_replan_delay_);
    post_goal_replan_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<std::int64_t>(post_goal_replan_delay_ * 1000.0)),
      [this]() {
        post_goal_replan_timer_->cancel();
        navigation_retry_not_before_ =
        rclcpp::Time(0, 0, RCL_ROS_TIME);
        if (!paused_ && !navigating_ && !evaluating_ &&
        !completion_started_ && !finalization_started_)
        {
          tick();
        }
      });
  }

  void requestNav2LifecycleStates()
  {
    if (nav2_lifecycle_query_pending_) {
      return;
    }
    if (!controller_state_client_->service_is_ready() ||
      !planner_state_client_->service_is_ready() ||
      !navigator_state_client_->service_is_ready())
    {
      RCLCPP_INFO_THROTTLE(
        logger_, *get_clock(), 5000,
        "Waiting for Nav2 lifecycle state services");
      return;
    }

    nav2_lifecycle_query_pending_ = true;
    nav2_lifecycle_responses_pending_ = 3;
    nav2_lifecycle_responses_active_ = true;

    auto request = std::make_shared<GetState::Request>();
    controller_state_client_->async_send_request(
      request,
      [this](rclcpp::Client<GetState>::SharedFuture future) {
        handleNav2LifecycleState("controller_server", future);
      });
    planner_state_client_->async_send_request(
      std::make_shared<GetState::Request>(),
      [this](rclcpp::Client<GetState>::SharedFuture future) {
        handleNav2LifecycleState("planner_server", future);
      });
    navigator_state_client_->async_send_request(
      std::make_shared<GetState::Request>(),
      [this](rclcpp::Client<GetState>::SharedFuture future) {
        handleNav2LifecycleState("bt_navigator", future);
      });
  }

  void handleNav2LifecycleState(
    const std::string & node_name,
    rclcpp::Client<GetState>::SharedFuture future)
  {
    bool active = false;
    std::string state_label{"request failed"};
    try {
      const auto response = future.get();
      active =
        response->current_state.id ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      state_label = response->current_state.label;
    } catch (const std::exception & error) {
      state_label = error.what();
    }

    if (!active) {
      nav2_lifecycle_responses_active_ = false;
      RCLCPP_INFO(
        logger_, "Waiting for %s lifecycle state active (current: %s)",
        node_name.c_str(), state_label.c_str());
    }

    --nav2_lifecycle_responses_pending_;
    if (nav2_lifecycle_responses_pending_ != 0) {
      return;
    }

    nav2_lifecycle_query_pending_ = false;
    nav2_lifecycle_active_ = nav2_lifecycle_responses_active_;
    if (nav2_lifecycle_active_) {
      resetCompletionEvidence();
      RCLCPP_INFO(
        logger_,
        "Nav2 controller_server, planner_server, and bt_navigator are active");
    }
  }

  void beginCompletionSequence(const std::string & reason)
  {
    if (completion_started_) {
      return;
    }

    completion_started_ = true;
    evaluating_ = false;
    ++evaluation_generation_;
    compute_path_client_->async_cancel_all_goals();
    publishStatus(explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE);
    RCLCPP_WARN(
      logger_, "Exploration complete: %s", reason.c_str());

    if (!return_to_start_) {
      beginFinalization("return-to-start disabled");
      return;
    }
    if (!initial_pose_recorded_) {
      RCLCPP_ERROR(
        logger_, "Cannot return to start because no initial pose was recorded");
      beginFinalization("initial pose unavailable");
      return;
    }

    sendReturnToStartGoal();
  }

  void sendReturnToStartGoal()
  {
    Navigate::Goal goal;
    goal.pose = initial_pose_;
    goal.pose.header.stamp = now();

    ++navigation_generation_;
    const auto generation = navigation_generation_;
    navigating_ = true;
    returning_to_start_ = true;
    return_started_time_ = now();
    publishStatus(
      explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN);
    RCLCPP_INFO(
      logger_,
      "Returning to exploration start pose (%.2f, %.2f), timeout %.1f s",
      goal.pose.pose.position.x, goal.pose.pose.position.y, return_timeout_);

    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](
      const NavigateGoalHandle::SharedPtr & goal_handle)
      {
        if (generation != navigation_generation_) {
          return;
        }
        navigation_goal_handle_ = goal_handle;
        if (!goal_handle) {
          navigating_ = false;
          returning_to_start_ = false;
          RCLCPP_ERROR(logger_, "NavigateToPose rejected return-to-start goal");
          beginFinalization("return goal rejected");
        }
      };
    options.result_callback =
      [this, generation](
      const NavigateGoalHandle::WrappedResult & result)
      {
        if (generation != navigation_generation_) {
          return;
        }
        navigating_ = false;
        returning_to_start_ = false;
        navigation_goal_handle_.reset();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(logger_, "Returned to exploration start pose");
          publishStatus(
            explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN);
          beginFinalization("return-to-start succeeded");
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_ERROR(logger_, "Return-to-start navigation aborted");
          beginFinalization("return-to-start aborted");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_WARN(logger_, "Return-to-start navigation canceled");
          beginFinalization("return-to-start canceled");
        } else {
          RCLCPP_ERROR(logger_, "Return-to-start returned an unknown result");
          beginFinalization("return-to-start unknown result");
        }
      };

    navigate_client_->async_send_goal(goal, options);
  }

  void beginFinalization(const std::string & reason)
  {
    if (finalization_started_) {
      return;
    }
    finalization_started_ = true;
    paused_ = true;
    evaluating_ = false;
    navigating_ = false;
    returning_to_start_ = false;
    ++evaluation_generation_;
    ++navigation_generation_;
    compute_path_client_->async_cancel_all_goals();
    navigate_client_->async_cancel_all_goals();
    startStopBurst();

    if (!save_on_completion_) {
      final_save_finished_ = true;
      RCLCPP_WARN(
        logger_, "Exploration stopped without final map save: %s",
        reason.c_str());
      maybeExitAfterFinalization();
      return;
    }

    final_save_reason_ = reason;
    final_save_pending_ = true;
    final_save_finished_ = false;
    final_save_request_sent_ = false;
    map_save_deadline_ =
      now() + rclcpp::Duration::from_seconds(map_save_service_timeout_);
    tryFinalMapSave();
  }

  void tryFinalMapSave()
  {
    if (!final_save_pending_) {
      return;
    }
    if (final_save_request_sent_) {
      if (now() <= map_save_deadline_) {
        return;
      }
      final_save_pending_ = false;
      final_save_finished_ = true;
      RCLCPP_ERROR(
        logger_,
        "Final map save request exceeded %.1f s; stopping without a "
        "confirmed save",
        map_save_service_timeout_);
      maybeExitAfterFinalization();
      return;
    }

    if (!map_save_client_->service_is_ready()) {
      if (now() <= map_save_deadline_) {
        RCLCPP_INFO_THROTTLE(
          logger_, *get_clock(), 5000,
          "Waiting for map save service %s",
          map_save_service_.c_str());
        return;
      }
      final_save_pending_ = false;
      final_save_finished_ = true;
      RCLCPP_ERROR(
        logger_, "Map save service %s was unavailable; final map was not saved",
        map_save_service_.c_str());
      maybeExitAfterFinalization();
      return;
    }

    final_save_request_sent_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    map_save_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        final_save_pending_ = false;
        final_save_finished_ = true;
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              logger_, "Final map saved after %s: %s",
              final_save_reason_.c_str(), response->message.c_str());
          } else {
            RCLCPP_ERROR(
              logger_, "Final map save failed after %s: %s",
              final_save_reason_.c_str(), response->message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            logger_, "Final map save request failed: %s", error.what());
        }
        maybeExitAfterFinalization();
      });
  }

  void startStopBurst()
  {
    final_stop_finished_ = false;
    final_stop_messages_sent_ = 0;
    const auto interval =
      std::chrono::milliseconds(final_stop_repeat_interval_ms_);
    stop_timer_ = create_wall_timer(
      interval,
      [this]() {
        geometry_msgs::msg::Twist stop;
        stop_pub_->publish(stop);
        ++final_stop_messages_sent_;
        if (final_stop_messages_sent_ < final_stop_repeat_count_) {
          return;
        }
        stop_timer_->cancel();
        final_stop_finished_ = true;
        RCLCPP_INFO(
          logger_, "Published %d final zero-velocity commands",
          final_stop_messages_sent_);
        maybeExitAfterFinalization();
      });
  }

  void maybeExitAfterFinalization()
  {
    if (!finalization_started_ || !final_save_finished_ ||
      !final_stop_finished_)
    {
      return;
    }
    if (exit_on_completion_) {
      RCLCPP_INFO(
        logger_,
        "Exploration finalization complete; shutting down autonomous "
        "exploration");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(
        logger_,
        "Exploration finalization complete; node remains paused by "
        "configuration");
    }
  }

  void checkProgressTimeout()
  {
    if (returning_to_start_) {
      if (now() - return_started_time_ <=
        rclcpp::Duration::from_seconds(return_timeout_))
      {
        return;
      }

      RCLCPP_ERROR(
        logger_,
        "Return-to-start exceeded %.1f s; canceling goal and saving map",
        return_timeout_);
      ++navigation_generation_;
      navigating_ = false;
      returning_to_start_ = false;
      navigation_goal_handle_.reset();
      navigate_client_->async_cancel_all_goals();
      beginFinalization("return-to-start timeout");
      return;
    }

    const auto current_pose = costmap_client_.getRobotPose();
    const double quaternion_norm =
      std::sqrt(
      current_pose.orientation.x * current_pose.orientation.x +
      current_pose.orientation.y * current_pose.orientation.y +
      current_pose.orientation.z * current_pose.orientation.z +
      current_pose.orientation.w * current_pose.orientation.w);
    if (quaternion_norm >= 0.5) {
      const double translation =
        pointDistance(current_pose.position, last_progress_pose_.position);
      const double rotation = std::abs(
        normalizeAngle(
          tf2::getYaw(current_pose.orientation) -
          tf2::getYaw(last_progress_pose_.orientation)));
      if (translation >= progress_epsilon_ ||
        rotation >= progress_yaw_epsilon_)
      {
        last_progress_pose_ = current_pose;
        last_progress_time_ = now();
      }
    }

    if (now() - last_progress_time_ <=
      rclcpp::Duration::from_seconds(progress_timeout_))
    {
      return;
    }

    RCLCPP_WARN(
      logger_,
      "No navigation progress for %.1f s; canceling and temporarily "
      "suppressing frontier",
      progress_timeout_);
    recordFailure(
      current_candidate_.frontier_center, "navigation progress timeout");
    ++navigation_generation_;
    navigating_ = false;
    navigation_goal_handle_.reset();
    navigate_client_->async_cancel_all_goals();
    navigation_retry_not_before_ =
      now() + rclcpp::Duration::from_seconds(planning_period_);
  }

  void pause()
  {
    if (completion_started_) {
      beginFinalization("completion sequence manually paused");
      return;
    }
    if (paused_) {
      return;
    }
    paused_ = true;
    evaluating_ = false;
    navigating_ = false;
    ++evaluation_generation_;
    ++navigation_generation_;
    compute_path_client_->async_cancel_all_goals();
    navigate_client_->async_cancel_all_goals();
    publishStatus(explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED);
    RCLCPP_INFO(logger_, "Ackermann exploration paused");
  }

  void resume()
  {
    if (completion_started_) {
      RCLCPP_WARN(
        logger_,
        "Exploration has completed; restart the node to begin a new map");
      return;
    }
    if (!paused_) {
      return;
    }
    paused_ = false;
    resetCompletionEvidence();
    publishStatus(explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED);
    RCLCPP_INFO(logger_, "Ackermann exploration resumed");
  }

  void publishStatus(const std::string & status)
  {
    explore_lite_msgs::msg::ExploreStatus message;
    message.status = status;
    status_pub_->publish(message);
  }

  void visualize(
    const std::vector<frontier_exploration::Frontier> & frontiers,
    const std::vector<Candidate> & candidates,
    const Candidate * selected)
  {
    if (!visualize_ || !marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = costmap_client_.getGlobalFrameID();
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    int id = 1;
    for (const auto & frontier : frontiers) {
      visualization_msgs::msg::Marker marker;
      marker.header = clear.header;
      marker.ns = "frontiers";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      marker.points = frontier.points;
      array.markers.push_back(marker);
    }

    for (const auto & candidate : candidates) {
      visualization_msgs::msg::Marker marker;
      marker.header = clear.header;
      marker.ns = "candidate_viewpoints";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = candidate.pose.pose;
      marker.scale.x = 0.30;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      marker.color.g = 1.0;
      marker.color.a = 0.8;
      array.markers.push_back(marker);
    }

    if (selected) {
      visualization_msgs::msg::Marker marker;
      marker.header = clear.header;
      marker.ns = "selected_viewpoint";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = selected->pose.pose;
      marker.scale.x = 0.45;
      marker.scale.y = 0.10;
      marker.scale.z = 0.10;
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.a = 1.0;
      array.markers.push_back(marker);
    }

    marker_pub_->publish(array);
  }

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  explore::Costmap2DClient costmap_client_;
  frontier_exploration::FrontierSearch frontier_search_;

  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
  rclcpp::Client<GetState>::SharedPtr controller_state_client_;
  rclcpp::Client<GetState>::SharedPtr planner_state_client_;
  rclcpp::Client<GetState>::SharedPtr navigator_state_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr map_save_client_;
  NavigateGoalHandle::SharedPtr navigation_goal_handle_;
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr post_goal_replan_timer_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  nav2_msgs::msg::Costmap::SharedPtr latest_global_costmap_;
  nav2_msgs::msg::Costmap::SharedPtr latest_local_costmap_;

  std::vector<Candidate> candidates_;
  std::vector<EvaluatedCandidate> evaluated_;
  std::vector<FailureRecord> failure_records_;
  Candidate current_candidate_;
  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::Pose last_progress_pose_;
  geometry_msgs::msg::PoseStamped initial_pose_;

  std::size_t evaluation_index_{0};
  std::size_t last_detected_frontiers_{0};
  std::uint64_t evaluation_generation_{0};
  std::uint64_t navigation_generation_{0};
  std::uint64_t completion_evidence_map_revision_{0};
  NoReachableKind completion_evidence_kind_{NoReachableKind::NoFrontiers};
  int no_frontier_cycles_{0};
  int final_stop_messages_sent_{0};
  int safety_transient_failure_count_{0};
  bool paused_{false};
  bool evaluating_{false};
  bool navigating_{false};
  bool initial_pose_recorded_{false};
  bool completion_started_{false};
  bool completion_evidence_initialized_{false};
  bool nav2_lifecycle_active_{false};
  bool nav2_lifecycle_query_pending_{false};
  bool nav2_lifecycle_responses_active_{false};
  bool returning_to_start_{false};
  bool finalization_started_{false};
  bool final_save_pending_{false};
  bool final_save_request_sent_{false};
  bool final_save_finished_{false};
  bool final_stop_finished_{false};

  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_pose_recorded_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_retry_not_before_{0, 0, RCL_ROS_TIME};
  rclcpp::Time return_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time map_save_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time evaluation_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_costmap_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_costmap_time_{0, 0, RCL_ROS_TIME};
  double best_distance_remaining_{std::numeric_limits<double>::infinity()};

  double planning_period_{5.0};
  double post_goal_replan_delay_{1.0};
  double evaluation_timeout_{40.0};
  double progress_timeout_{35.0};
  double progress_epsilon_{0.05};
  double progress_yaw_epsilon_{0.08};
  double map_freshness_timeout_{5.0};
  double completion_map_stability_duration_{10.0};
  double min_frontier_size_{0.45};
  double candidate_standoff_{0.45};
  double candidate_standoff_max_{0.75};
  double footprint_clearance_{0.23};
  double clearance_search_radius_{0.60};
  double candidate_back_clearance_distance_{0.0};
  double minimum_candidate_travel_distance_{0.50};
  double candidate_inward_search_radius_{0.40};
  int candidate_inward_angle_samples_{5};
  double candidate_inward_angle_span_{1.0471975512};
  double global_costmap_freshness_timeout_{5.0};
  double local_costmap_freshness_timeout_{3.0};
  double emergency_stop_clearance_{0.42};
  double candidate_deduplication_distance_{0.20};
  int candidate_angle_samples_{24};
  int max_frontiers_to_consider_{20};
  int max_viewpoints_per_frontier_{2};
  int max_frontiers_to_evaluate_{6};
  int safety_transient_failure_cycles_{3};
  int nav2_lifecycle_responses_pending_{0};
  double blacklist_radius_{0.50};
  double blacklist_ttl_{90.0};
  int blacklist_map_revision_delta_{3};
  double frontier_preselection_gain_weight_{0.8};
  double information_gain_radius_{1.75};
  double frontier_length_gain_weight_{0.35};
  double candidate_heading_weight_{0.8};
  double path_length_weight_{1.0};
  double reverse_length_weight_{2.0};
  double gear_switch_weight_{1.5};
  double heading_change_weight_{0.4};
  double steering_effort_weight_{0.2};
  double clearance_weight_{0.25};
  double information_gain_weight_{0.8};
  double max_reverse_fraction_{1.0};
  double max_reverse_length_{0.8};
  int max_gear_switches_{4};
  int completion_stability_cycles_{3};
  double completion_grace_period_{60.0};
  double navigation_rejection_retry_delay_{5.0};
  double return_timeout_{120.0};
  double map_save_service_timeout_{10.0};
  int final_stop_repeat_count_{10};
  int final_stop_repeat_interval_ms_{20};
  std::string planner_id_{"GridBased"};
  std::string global_costmap_topic_{"/global_costmap/costmap_raw"};
  std::string local_costmap_topic_{"/local_costmap/costmap_raw"};
  std::string map_save_service_{"/map_autosaver/save_now"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string final_save_reason_;
  bool return_to_start_{true};
  bool save_on_completion_{true};
  bool exit_on_completion_{true};
  bool visualize_{true};
  bool dry_run_{false};
};

}  // namespace ackermann_frontier_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node =
      std::make_shared<
      ackermann_frontier_explorer::AckermannFrontierExplorer>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ackermann_frontier_explorer"),
        "Explorer startup failed: %s", error.what());
      exit_code = 1;
    }
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
