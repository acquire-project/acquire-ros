
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "acquire_zarr/zarr_writer_node.hpp"
#include "zarr.h"

namespace acquire_zarr
{
  ZarrWriterNode::ZarrWriterNode(const rclcpp::NodeOptions node_options): Node("zarr_writer_node", node_options)
  {

    settings_from_params();


    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the collection window and publish period (default 1s)
    options.topic_stats_options.publish_period = std::chrono::seconds(5);

    // configure the topic name (default '/statistics')
    // options.topic_stats_options.publish_topic = "/topic_statistics";

    auto callback = [this](const sensor_msgs::msg::Image & msg) {
        this->topic_callback(msg);
      };

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, callback, options);


  }

  ZarrWriterNode::~ZarrWriterNode()
  {
    ZarrStreamSettings_destroy(zarr_stream_settings_);
  }

  void ZarrWriterNode::settings_from_params()
  {

    zarr_stream_settings_ = ZarrStreamSettings_create();

    this->declare_parameter<std::string>("store_path", "out.zarr");
    auto store_path = this->get_parameter("store_path").as_string();
    ZarrStreamSettings_set_store(this->zarr_stream_settings_, store_path.c_str(), store_path.size()+1, nullptr);

    this->declare_parameter<int>("data_type", (int)ZarrDataType_uint8);
    auto data_type = this->get_parameter("data_type").as_int();
    ZarrStreamSettings_set_data_type(this->zarr_stream_settings_, (ZarrDataType)data_type);


    this->declare_parameter("dimension_names", std::vector<std::string>{"t", "y", "x"});
    auto dimension_names = this->get_parameter("dimension_names").as_string_array();
    auto n_dimensions = dimension_names.size();

    this->declare_parameter("dimension_types", std::vector<int>{(int)ZarrDimensionType_Time, (int)ZarrDimensionType_Space, (int)ZarrDimensionType_Space});
    auto dimension_types = this->get_parameter("dimension_types").as_integer_array();

    this->declare_parameter("dimension_sizes", std::vector<int>{2, 480, 640});
    auto dimension_sizes = this->get_parameter("dimension_sizes").as_integer_array();

    this->declare_parameter("dimension_chunk_px", std::vector<int>{1, 480, 640});
    auto dimension_chunk_px_sizes = this->get_parameter("dimension_chunk_px").as_integer_array();

    this->declare_parameter("dimension_shard_chunks", std::vector<int>{1, 1, 1});
    auto dimension_shard_chunks = this->get_parameter("dimension_shard_chunks").as_integer_array();

    ZarrStreamSettings_reserve_dimensions(zarr_stream_settings_, n_dimensions);


    for(size_t i = 0; i < n_dimensions; i++)
    {

      ZarrDimensionProperties dim_props = {
        dimension_names[i].c_str(),
        dimension_names[i].size()+1,
        (ZarrDimensionType)dimension_types[i],
        (uint32_t)dimension_sizes[i],
        (uint32_t)dimension_chunk_px_sizes[i],
        (uint32_t)dimension_shard_chunks[i]
      };
      ZarrStreamSettings_set_dimension(zarr_stream_settings_, i, &dim_props);
    }

    zarr_stream_ = ZarrStream_create(zarr_stream_settings_, ZarrVersion_2);

  }

  void ZarrWriterNode::topic_callback(const sensor_msgs::msg::Image & img) const
  {
    size_t size_out = 0;
    ZarrStream_append(zarr_stream_, img.data.data(), img.data.size(), &size_out);
  }
}  // namespace acquire_zarr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(acquire_zarr::ZarrWriterNode)