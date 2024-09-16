#ifndef ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_
#define ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "zarr.h"

namespace acquire_zarr
{

  class ZarrWriterNode : public rclcpp::Node
  {
  public:
    ZarrWriterNode(const rclcpp::NodeOptions);
    ~ZarrWriterNode();
  private:
    void settings_from_params();
    void topic_callback(const sensor_msgs::msg::Image & msg) const;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // unfortunately, this has to be a dumb pointer because of the aligned storage.
    ZarrStreamSettings* zarr_stream_settings_;
    ZarrStream* zarr_stream_;
  };

}  // namespace acquire_zarr

#endif  // ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_