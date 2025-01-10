
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "acquire_zarr/zarr_writer_node.hpp"

using std::placeholders::_1;


namespace acquire_zarr
{
  template <typename T>
  ZarrWriterNode<T>::ZarrWriterNode(const rclcpp::NodeOptions node_options) : Node("zarr_writer_node", node_options)
  {

    settings_from_params();

    image_sub_ = this->create_subscription<T>(
      "image_data", 
      10, 
      std::bind(&ZarrWriterNode::topic_callback, this, _1));

  }

  template <typename T>
  ZarrWriterNode<T>::~ZarrWriterNode()
  {
    ZarrStreamSettings_destroy_dimension_array(&zarr_stream_settings_);
    if (zarr_stream_ != nullptr)
    {
      ZarrStream_destroy(zarr_stream_);
    }
  }

  template <typename T> 
  void ZarrWriterNode<T>::settings_from_params()
  {

    zarr_stream_settings_.version = ZarrVersion_2;

    this->declare_parameter<std::string>("zarr_out_path", "out.zarr");
    store_path_ = this->get_parameter("zarr_out_path").as_string();
    zarr_stream_settings_.store_path = store_path_.c_str();

    this->declare_parameter<int>("data_type", (int)ZarrDataType_uint8);
    auto data_type = this->get_parameter("data_type").as_int();
    this->zarr_stream_settings_.data_type = (ZarrDataType)data_type;

    this->declare_parameter("dimension_names", std::vector<std::string>{"t", "y", "x"});
    dimension_names_ = this->get_parameter("dimension_names").as_string_array();
    auto n_dimensions = dimension_names_.size();

    this->declare_parameter("dimension_types", std::vector<int>{(int)ZarrDimensionType_Time, (int)ZarrDimensionType_Space, (int)ZarrDimensionType_Space});
    auto dimension_types = this->get_parameter("dimension_types").as_integer_array();

    this->declare_parameter("dimension_sizes", std::vector<int>{2, 480, 640});
    auto dimension_sizes = this->get_parameter("dimension_sizes").as_integer_array();

    this->declare_parameter("dimension_chunk_px", std::vector<int>{1, 480, 640});
    auto dimension_chunk_px_sizes = this->get_parameter("dimension_chunk_px").as_integer_array();

    this->declare_parameter("dimension_shard_chunks", std::vector<int>{1, 1, 1});
    auto dimension_shard_chunks = this->get_parameter("dimension_shard_chunks").as_integer_array();

    ZarrStreamSettings_create_dimension_array(&zarr_stream_settings_, n_dimensions);
    for (size_t i = 0; i < n_dimensions; i++)
    {

      zarr_stream_settings_.dimensions[i] = {
          dimension_names_[i].c_str(),
          (ZarrDimensionType)dimension_types[i],
          (uint32_t)dimension_sizes[i],
          (uint32_t)dimension_chunk_px_sizes[i],
          (uint32_t)dimension_shard_chunks[i]};
    }

    zarr_stream_ = ZarrStream_create(&zarr_stream_settings_);
  }

  template <typename T> 
  void ZarrWriterNode<T>::topic_callback(const T & msg) const
  {
    size_t size_out = 0;
    ZarrStream_append(zarr_stream_, msg.data.data(), msg.data.size(), &size_out);
  }

} // namespace acquire_zarr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(acquire_zarr::ZarrWriterNode<sensor_msgs::msg::Image>)