#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanQosRelay : public rclcpp::Node {
public:
  ScanQosRelay() : Node("scan_qos_relay") {
    const auto input_topic =
        this->declare_parameter<std::string>("input_topic", "/scan_raw");
    const auto output_topic =
        this->declare_parameter<std::string>("output_topic", "/scan");
    const auto input_depth =
        this->declare_parameter<int>("input_queue_depth", 20);
    const auto output_depth =
        this->declare_parameter<int>("output_queue_depth", 20);

    auto input_qos = rclcpp::SensorDataQoS().keep_last(input_depth);
    auto output_qos =
        rclcpp::QoS(rclcpp::KeepLast(output_depth)).reliable().durability_volatile();

    publisher_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, output_qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic, input_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          publisher_->publish(*msg);
        });

    RCLCPP_INFO(this->get_logger(),
                "Relaying LaserScan from '%s' (best effort) to '%s' (reliable)",
                input_topic.c_str(), output_topic.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanQosRelay>());
  rclcpp::shutdown();
  return 0;
}
