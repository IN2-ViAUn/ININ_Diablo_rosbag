#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sys/stat.h>

// #define POSE_TOPIC1 "/robot_0/camera_pose"
// #define RGB_TOPIC "/rgb/image_raw"
// #define DEPTH_TOPIC "/depth_to_rgb/image_raw"

#define POSE_TOPIC1 "/state_estimation"
#define RGB_TOPIC "/rgb/image_raw"
#define DEPTH_TOPIC "/depth_to_rgb/image_raw"

#define SAVE_PATH_ROOT "/home/yangdianyi/colcon_ws/rosbag_save_3_29/test"
#define POSE_SAVE_QUAT "/home/yangdianyi/colcon_ws/rosbag_save_3_29/test/pose_quat.txt"
#define POSE_SAVE_RT "/home/yangdianyi/colcon_ws/rosbag_save_3_29/test/pose_rt.txt"

// 设置保存depth和rgb图像的路径，再root路径下分别建立depth_images和rgb_images文件夹
# define SAVE_DEPTH_IMAGE_PATH "/home/yangdianyi/colcon_ws/rosbag_save_3_29/test/depth_images"
# define SAVE_RGB_IMAGE_PATH "/home/yangdianyi/colcon_ws/rosbag_save_3_29/test/rgb_images"

std::string save_path = SAVE_PATH_ROOT;

class PoseImageSyncNode : public rclcpp::Node
{
public:
  PoseImageSyncNode() : Node("pose_image_sync_node")
  {
    // Set up synchronized subscribers
    pose_sub1_.subscribe(this, POSE_TOPIC1);
    image_sub1_.subscribe(this, DEPTH_TOPIC);
    image_sub2_.subscribe(this, RGB_TOPIC);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped,
                                                            sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::Image>
        MySyncPolicy;

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(300), pose_sub1_, image_sub1_, image_sub2_));
    sync_->registerCallback(std::bind(&PoseImageSyncNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  ~PoseImageSyncNode()
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
  void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose1,
                const sensor_msgs::msg::Image::ConstSharedPtr &image1,
                const sensor_msgs::msg::Image::ConstSharedPtr &image2)
  {
    static int index = 0;
    RCLCPP_INFO(this->get_logger(), "Received synchronized data");

    // Process pose1
    double RT1[16];
    create_RT(pose1, RT1);

    // Save quaternion data
    save_pose_quaternion(pose1);

    // Save RT matrix data
    save_pose_rt(RT1);

    // Save RGB and depth images
    if (index % 1 == 0)
    {
      save_image(image1, SAVE_DEPTH_IMAGE_PATH, index);
      save_image(image2, SAVE_RGB_IMAGE_PATH, index);
    }

    index++;
  }

  void save_pose_quaternion(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose)
  {
    std::ofstream file(POSE_SAVE_QUAT, std::ios_base::app);
    if (!file)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file for quaternion saving");
      return;
    }

    // Write quaternion data: w qx qy qz x y z
    file << pose->pose.orientation.w << " "
         << pose->pose.orientation.x << " "
         << pose->pose.orientation.y << " "
         << pose->pose.orientation.z << " "
         << pose->pose.position.x << " "
         << pose->pose.position.y << " "
         << pose->pose.position.z << "\n";

    file.close();
  }

  void save_pose_rt(const double *RT)
  {
    std::ofstream file(POSE_SAVE_RT, std::ios_base::app);
    if (!file)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file for RT matrix saving");
      return;
    }

    // Write RT matrix data
    for (int i = 0; i < 16; i++)
    {
      file << RT[i] << " ";
    }
    file << "\n";

    file.close();
  }

  void save_image(const sensor_msgs::msg::Image::ConstSharedPtr msg, const std::string &folder, int index)
  {
    ensure_directory_exists(folder);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (msg->encoding == "16UC1")
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      else if (msg->encoding == "32FC1")
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_mm;
        cv_ptr->image.convertTo(depth_mm, CV_16UC1, 1000.0);
        cv_ptr->image = depth_mm;
      }
      else
      {
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

  void create_RT(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose, double *RT)
  {
    double position_x = pose->pose.position.x;
    double position_y = pose->pose.position.y;
    double position_z = pose->pose.position.z;
    double orientation_x = pose->pose.orientation.x;
    double orientation_y = pose->pose.orientation.y;
    double orientation_z = pose->pose.orientation.z;
    double orientation_w = pose->pose.orientation.w;

    // Generate extrinsic matrix
    getExtrinsicMatrix(position_x, position_y, position_z, orientation_x,
                       orientation_y, orientation_z, orientation_w, RT);
  }

  void getExtrinsicMatrix(const double position_x, const double position_y, const double position_z,
                          const double orientation_x, const double orientation_y, const double orientation_z,
                          const double orientation_w, double *extrinsicMatrix)
  {
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    quat2rot(orientation_x, orientation_y, orientation_z, orientation_w, r11, r12, r13, r21, r22, r23, r31, r32, r33);

    extrinsicMatrix[0] = r11;
    extrinsicMatrix[1] = r12;
    extrinsicMatrix[2] = r13;
    extrinsicMatrix[3] = position_x;

    extrinsicMatrix[4] = r21;
    extrinsicMatrix[5] = r22;
    extrinsicMatrix[6] = r23;
    extrinsicMatrix[7] = position_y;

    extrinsicMatrix[8] = r31;
    extrinsicMatrix[9] = r32;
    extrinsicMatrix[10] = r33;
    extrinsicMatrix[11] = position_z;

    extrinsicMatrix[12] = 0;
    extrinsicMatrix[13] = 0;
    extrinsicMatrix[14] = 0;
    extrinsicMatrix[15] = 1;
  }

  void quat2rot(const double qx, const double qy, const double qz, const double qw,
                double &r11, double &r12, double &r13, double &r21, double &r22, double &r23, double &r31, double &r32, double &r33)
  {
    double sqx = qx * qx;
    double sqy = qy * qy;
    double sqz = qz * qz;
    double sqw = qw * qw;

    r11 = sqx - sqy - sqz + sqw;
    r21 = 2 * (qx * qy - qz * qw);
    r31 = 2 * (qx * qz + qy * qw);

    r12 = 2 * (qx * qy + qz * qw);
    r22 = -sqx + sqy - sqz + sqw;
    r32 = 2 * (qy * qz - qx * qw);

    r13 = 2 * (qx * qz - qy * qw);
    r23 = 2 * (qy * qz + qx * qw);
    r33 = -sqx - sqy + sqz + sqw;
  }

  void ensure_directory_exists(const std::string &name)
  {
    struct stat buffer;
    if (stat(name.c_str(), &buffer) != 0)
    {
      mkdir(name.c_str(), 0777);
    }
  }

  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub2_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      geometry_msgs::msg::PoseStamped, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
      sync_;
  nlohmann::json json_array_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseImageSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}