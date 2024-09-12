#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "zarr.h"

class ZarrWriterNode : public rclcpp::Node
{
public:
  ZarrWriterNode();
  ~ZarrWriterNode();
private:
  void settings_from_params();
  void topic_callback(const sensor_msgs::msg::Image & msg) const;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  // unfortunately, this has to be a dumb pointer because of the aligned storage.
  ZarrStreamSettings* zarr_stream_settings_;
  ZarrStream* zarr_stream_;
};

