
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "acquire_zarr/zarr_writer_node.hpp"

using std::placeholders::_1;

namespace
{
  std::string
  to_lowercase(const std::string &s)
  {
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
  }
}

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
    zarr_stream_settings_.version = ZarrVersion_3;

    this->declare_parameter<std::string>("zarr_out_path", "out.zarr");
    store_path_ = this->get_parameter("zarr_out_path").as_string();
    zarr_stream_settings_.store_path = store_path_.c_str();

    // infer zarr data type from template type
    if constexpr (std::is_same_v<T, sensor_msgs::msg::Image>)
    {
      // todo: add support for 16 bit images
      zarr_stream_settings_.data_type = ZarrDataType_uint8;
    }
    else if constexpr (std::is_same_v<T, std_msgs::msg::Float32MultiArray>)
    {
      zarr_stream_settings_.data_type = ZarrDataType_float32;
    }
    else
    {
      throw std::runtime_error("Unsupported data type");
    }

    this->declare_parameter("zarr_format", 3);
    const auto zarr_format = this->get_parameter("zarr_format").as_int();
    switch (zarr_format)
    {
    case 2:
      zarr_stream_settings_.version = ZarrVersion_2;
      break;
    case 3:
      zarr_stream_settings_.version = ZarrVersion_3;
      break;
    default:
      throw std::runtime_error("Unsupported Zarr format version: " + std::to_string(zarr_format));
    }

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

    this->declare_parameter("compression_codec", "none");
    const auto compression_codec = to_lowercase(this->get_parameter("compression_codec").as_string());

    this->declare_parameter("compression_level", 1);
    const auto compression_level = this->get_parameter("compression_level").as_int();

    this->declare_parameter("compression_shuffle", 1);
    const auto compression_shuffle = this->get_parameter("compression_shuffle").as_int();

    this->declare_parameter("multiscale", false);
    const auto multiscale = this->get_parameter("multiscale").as_bool();
    zarr_stream_settings_.multiscale = multiscale;

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

    if (compression_codec != "none")
    {
      zarr_compression_settings_.compressor = ZarrCompressor_Blosc1;
      zarr_compression_settings_.level = compression_level;
      zarr_compression_settings_.shuffle = compression_shuffle;

      if (compression_codec == "lz4")
      {
        zarr_compression_settings_.codec = ZarrCompressionCodec_BloscLZ4;
      }
      else if (compression_codec == "zstd")
      {
        zarr_compression_settings_.codec = ZarrCompressionCodec_BloscZstd;
      }
      else
      {
        throw std::runtime_error("Unsupported compression codec: '" + compression_codec + "'");
      }

      zarr_stream_settings_.compression_settings = &zarr_compression_settings_;
    }

    zarr_stream_ = ZarrStream_create(&zarr_stream_settings_);
  }

  template <>
  void ZarrWriterNode<sensor_msgs::msg::Image>::topic_callback(const sensor_msgs::msg::Image& msg) const
  {
    size_t size_out = 0;
    ZarrStream_append(zarr_stream_, msg.data.data(), msg.data.size(), &size_out);
  }

  template <typename T>
  void ZarrWriterNode<T>::topic_callback(const T& msg) const
  {
    if (msg.layout.dim.size() != zarr_stream_settings_.dimension_count - 1)
    {
      throw std::runtime_error("MultiArray topic dimensions do not match Zarr dimensions");
    }

    size_t size_in = sizeof(msg.data[0]), size_out = 0;
    for (size_t i = 0; i < msg.layout.dim.size(); i++)
    {
      if (msg.layout.dim[i].size != zarr_stream_settings_.dimensions[i + 1].array_size_px)
      {
        throw std::runtime_error("MultiArray topic dimensions do not match Zarr dimensions");
      }
      size_in *= msg.layout.dim[i].size;
    }
    ZarrStream_append(zarr_stream_, msg.data.data(), size_in, &size_out);

    if (size_out != size_in)
    {
      throw std::runtime_error("ZarrStream_append did not write the correct number of bytes");
    }
  }

  using ImageZarrWriterNode = ZarrWriterNode<sensor_msgs::msg::Image>;
  using Float32MultiArrayZarrWriterNode = ZarrWriterNode<std_msgs::msg::Float32MultiArray>;
} // namespace acquire_zarr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

RCLCPP_COMPONENTS_REGISTER_NODE(acquire_zarr::ImageZarrWriterNode)
RCLCPP_COMPONENTS_REGISTER_NODE(acquire_zarr::Float32MultiArrayZarrWriterNode)
