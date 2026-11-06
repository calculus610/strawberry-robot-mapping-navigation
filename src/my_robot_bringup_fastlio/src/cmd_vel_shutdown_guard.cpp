#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

namespace
{

std::atomic_bool stop_requested{false};

void request_stop(int)
{
  stop_requested.store(true);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(
    argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

  auto node = std::make_shared<rclcpp::Node>("cmd_vel_shutdown_guard");
  const auto topic = node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto repeat_count = node->declare_parameter<int>("repeat_count", 10);
  const auto repeat_interval_ms =
    node->declare_parameter<int>("repeat_interval_ms", 20);

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  RCLCPP_INFO(
    node->get_logger(),
    "Shutdown guard armed on %s; it will publish zero velocity on SIGINT/SIGTERM",
    topic.c_str());

  while (rclcpp::ok() && !stop_requested.load()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (stop_requested.load()) {
    geometry_msgs::msg::Twist stop_message;
    const auto safe_repeat_count = std::max<std::int64_t>(1, repeat_count);
    const auto safe_interval_ms = std::max<std::int64_t>(1, repeat_interval_ms);
    for (std::int64_t index = 0; index < safe_repeat_count; ++index) {
      publisher->publish(stop_message);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(safe_interval_ms));
    }
    std::fprintf(
      stderr,
      "cmd_vel_shutdown_guard: published %lld zero-velocity messages on %s\n",
      static_cast<long long>(safe_repeat_count), topic.c_str());
    std::fflush(stderr);
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
