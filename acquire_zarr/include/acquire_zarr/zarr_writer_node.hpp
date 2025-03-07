#ifndef ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_
#define ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "acquire.zarr.h"

namespace acquire_zarr
{
  template <typename T>
  class ZarrWriterNode : public rclcpp::Node
  {
  public:
    ZarrWriterNode(const rclcpp::NodeOptions);
    ~ZarrWriterNode();
  private:
    void settings_from_params();

    void topic_callback(const T & msg) const;

    rclcpp::Subscription<T>::SharedPtr image_sub_;

    ZarrStreamSettings zarr_stream_settings_ = {};
    ZarrCompressionSettings zarr_compression_settings_ = {};
    ZarrStream* zarr_stream_ = nullptr;

    // string variables which are members so the memory can persist 
    // throughout the lifetime of the node
    std::string store_path_;
    std::vector<std::string> dimension_names_;
  };

}  // namespace acquire_zarr

#endif  // ACQUIRE_ZARR__ZARR_WRITER_NODE_HPP_