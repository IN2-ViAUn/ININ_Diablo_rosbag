#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sys/stat.h>

#define rgb_folder "/home/yangdianyi/colcon_ws/rosbag_save/test/rgb_images"
#define depth_folder "/home/yangdianyi/colcon_ws/rosbag_save/test/depth_images"

class ImageSyncNode : public rclcpp::Node
{
public:
  ImageSyncNode() : Node("image_sync_node")
  {
    image_sub1_.subscribe(this, "/depth_to_rgb/image_raw");
    image_sub2_.subscribe(this, "/rgb/image_raw");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::Image>
        MySyncPolicy;

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub1_, image_sub2_));
    sync_->registerCallback(std::bind(&ImageSyncNode::callback, this, std::placeholders::_1, std::placeholders::_2));
  }
  ~ImageSyncNode()
  {
    SaveJSON();
  }

  void SaveJSON()
  {
    std::cout << "saving..." << std::endl;
    std::ofstream file("timestamps.json");
    if (file)
    {
      file << json_array_.dump(4);
      file.close();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open timestamps.json for writing.");
    }
  }

private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr &image1,
                const sensor_msgs::msg::Image::ConstSharedPtr &image2)
  {
    static int index = 0;
    std::cout << index << std::endl;
    if (index % 1 == 0)
    {
      // down sample
      save_image(image1, depth_folder, index);
      save_image(image2, rgb_folder, index);
    }
    // SaveJSON();
    index++;
  }
  void save_image(const sensor_msgs::msg::Image::ConstSharedPtr msg, const std::string &folder, int index)
  {
    ensure_directory_exists(folder);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (msg->encoding == "16UC1")
      {
        // 将 ROS 图像消息转换为 OpenCV 的 CV_16UC1 格式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        // 如果深度值不是以毫米为单位，需要进行转换
        // 例如，如果深度值以米为单位，可以将其乘以 1000 转换为毫米
        // cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, 1000.0);
      }
      else if (msg->encoding == "32FC1")
      {
        // 处理以 32 位浮点数表示的深度图，通常深度值以米为单位
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // 将深度值从米转换为毫米，并转换为 16 位无符号整数
        cv::Mat depth_mm;
        cv_ptr->image.convertTo(depth_mm, CV_16UC1, 1000.0);
        cv_ptr->image = depth_mm;
      }
      else
      {
        // 对于其他编码格式，按照彩色图像处理
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    std::string filename = folder + "/" + std::to_string(index) + ".png";
    cv::imwrite(filename, cv_ptr->image);
    json_array_.push_back({{"timestamp_sec", msg->header.stamp.sec}, {"timestamp_nsec", msg->header.stamp.nanosec}, {"filename", filename}});
  }
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub2_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                                                sensor_msgs::msg::Image>>>
      sync_;
  nlohmann::json json_array_;
  void ensure_directory_exists(const std::string &name)
  {
    struct stat buffer;
    if (stat(name.c_str(), &buffer) != 0)
    {
      mkdir(name.c_str(), 0777);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
