#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "sensor_msgs/msg/image.hpp"

class ZarrWriterNode : public rclcpp::Node
{
public:
  ZarrWriterNode();

private:
  void topic_callback(const sensor_msgs::msg::Image & msg) const;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

