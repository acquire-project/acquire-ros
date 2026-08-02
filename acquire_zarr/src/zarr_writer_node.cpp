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
to_lowercase(const std::string & s)
{
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(), ::tolower);
  return result;
}

void
check_zarr_status(ZarrStatusCode status, const std::string & what)
{
  if (status != ZarrStatusCode_Success) {
    throw std::runtime_error(what + ": " + Zarr_get_status_message(status));
  }
}
}

namespace acquire_zarr
{
template<typename T>
ZarrWriterNode<T>::ZarrWriterNode(const rclcpp::NodeOptions node_options)
: Node("zarr_writer_node", node_options)
{

  settings_from_params();

  image_sub_ = this->create_subscription<T>(
        "image_data",
        10,
        std::bind(&ZarrWriterNode::topic_callback, this, _1));
}

template<typename T>
ZarrWriterNode<T>::~ZarrWriterNode()
{
  if (zarr_stream_settings_.arrays != nullptr) {
    ZarrArraySettings_destroy_dimension_array(zarr_stream_settings_.arrays);
    ZarrStreamSettings_destroy_arrays(&zarr_stream_settings_);
  }
  if (zarr_stream_ != nullptr) {
    ZarrStream_destroy(zarr_stream_);
  }
}

template<typename T>
void ZarrWriterNode<T>::settings_from_params()
{
  zarr_stream_settings_.version = ZarrVersion_3;

  this->declare_parameter<std::string>("zarr_out_path", "out.zarr");
  store_path_ = this->get_parameter("zarr_out_path").as_string();
  zarr_stream_settings_.store_path = store_path_.c_str();

  // This node streams a single array, so all the per-array settings below
  // hang off arrays[0].
  check_zarr_status(ZarrStreamSettings_create_arrays(&zarr_stream_settings_, 1),
                      "Failed to allocate Zarr array settings");
  ZarrArraySettings & array = zarr_stream_settings_.arrays[0];

  // infer zarr data type from template type
  if constexpr (std::is_same_v<T, sensor_msgs::msg::Image>) {
    // todo: add support for 16 bit images
    array.data_type = ZarrDataType_uint8;
  } else if constexpr (std::is_same_v<T, std_msgs::msg::Float32MultiArray>) {
    array.data_type = ZarrDataType_float32;
  } else {
    throw std::runtime_error("Unsupported data type");
  }

  this->declare_parameter("dimension_names", std::vector<std::string>{"t", "y", "x"});
  dimension_names_ = this->get_parameter("dimension_names").as_string_array();
  auto n_dimensions = dimension_names_.size();

  this->declare_parameter("dimension_types",
      std::vector<int>{(int)ZarrDimensionType_Time, (int)ZarrDimensionType_Space,
      (int)ZarrDimensionType_Space});
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
  array.multiscale = multiscale;

  check_zarr_status(ZarrArraySettings_create_dimension_array(&array, n_dimensions),
                      "Failed to allocate Zarr dimension array");
  for (size_t i = 0; i < n_dimensions; i++) {

    array.dimensions[i] = {
      dimension_names_[i].c_str(),
      (ZarrDimensionType)dimension_types[i],
      (uint32_t)dimension_sizes[i],
      (uint32_t)dimension_chunk_px_sizes[i],
      (uint32_t)dimension_shard_chunks[i],
      nullptr,
      1.0};
  }

  if (compression_codec != "none") {
    zarr_compression_settings_.compressor = ZarrCompressor_Blosc1;
    zarr_compression_settings_.level = compression_level;
    zarr_compression_settings_.shuffle = compression_shuffle;

    if (compression_codec == "lz4") {
      zarr_compression_settings_.codec = ZarrCompressionCodec_BloscLZ4;
    } else if (compression_codec == "zstd") {
      zarr_compression_settings_.codec = ZarrCompressionCodec_BloscZstd;
    } else {
      throw std::runtime_error("Unsupported compression codec: '" + compression_codec + "'");
    }

    array.compression_settings = &zarr_compression_settings_;
  }

  zarr_stream_ = ZarrStream_create(&zarr_stream_settings_);
  if (zarr_stream_ == nullptr) {
    throw std::runtime_error("Failed to create Zarr stream at '" + store_path_ + "'");
  }
}

template<>
void ZarrWriterNode<sensor_msgs::msg::Image>::topic_callback(
  const sensor_msgs::msg::Image & msg) const
{
  size_t size_out = 0;
  // A null key appends to the stream's single default array.
  check_zarr_status(
        ZarrStream_append(zarr_stream_, msg.data.data(), msg.data.size(), &size_out, nullptr),
        "Failed to append frame to Zarr stream");
}

template<typename T>
void ZarrWriterNode<T>::topic_callback(const T & msg) const
{
  const ZarrArraySettings & array = array_settings();
  if (msg.layout.dim.size() != array.dimension_count - 1) {
    throw std::runtime_error("MultiArray topic dimensions do not match Zarr dimensions");
  }

  size_t size_in = sizeof(msg.data[0]), size_out = 0;
  for (size_t i = 0; i < msg.layout.dim.size(); i++) {
    if (msg.layout.dim[i].size != array.dimensions[i + 1].array_size_px) {
      throw std::runtime_error("MultiArray topic dimensions do not match Zarr dimensions");
    }
    size_in *= msg.layout.dim[i].size;
  }
  // A null key appends to the stream's single default array.
  check_zarr_status(
        ZarrStream_append(zarr_stream_, msg.data.data(), size_in, &size_out, nullptr),
        "Failed to append frame to Zarr stream");

  if (size_out != size_in) {
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
