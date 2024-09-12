
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "acquire_zarr/zarr_writer_node.hpp"
#include "zarr.h"

ZarrWriterNode::ZarrWriterNode(): Node("zarr_writer_node")
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
  ZarrStreamSettings_set_store_path(this->zarr_stream_settings_, store_path.c_str(), store_path.size()+1);

  this->declare_parameter<int>("data_type", (int)ZarrDataType_uint8);
  auto data_type = this->get_parameter("data_type").as_int();
  ZarrStreamSettings_set_data_type(this->zarr_stream_settings_, (ZarrDataType)data_type);


  ZarrStreamSettings_reserve_dimensions(zarr_stream_settings_, 3);
  ZarrStreamSettings_set_dimension(zarr_stream_settings_, 0, "t", 2, ZarrDimensionType_Time, 0, 1, 1);
  ZarrStreamSettings_set_dimension(zarr_stream_settings_, 1, "y", 2, ZarrDimensionType_Space, 480, 480, 1);
  ZarrStreamSettings_set_dimension(zarr_stream_settings_, 2, "x", 2, ZarrDimensionType_Space, 640, 640, 1);
  

  zarr_stream_ = ZarrStream_create(zarr_stream_settings_, ZarrVersion_2);

}

void ZarrWriterNode::topic_callback(const sensor_msgs::msg::Image & img) const
{
  size_t size_out = 0;
  ZarrStream_append(zarr_stream_, img.data.data(), img.data.size(), &size_out);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZarrWriterNode>());
  rclcpp::shutdown();
  return 0;
}
