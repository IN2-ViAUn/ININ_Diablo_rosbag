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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <iomanip> // Add this include for std::fixed and std::setprecision

#define POSE_TOPIC1 "/vrpn_mocap/Azure_Cam0/pose"
#define RGB_TOPIC "/rgb/image_raw"
#define DEPTH_TOPIC "/depth_to_rgb/image_raw"
#define SAVE_PATH_ROOT "/home/bobh/diablo/dataset_tum_format"
#define POSE_SAVE_QUAT "/home/bobh/diablo/dataset_tum_format/pose_quat.txt"
#define POSE_SAVE_RT "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"

// 设置保存depth和rgb图像的路径，再root路径下分别建立depth_images和rgb_images文件夹
# define SAVE_DEPTH_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/depth"
# define SAVE_RGB_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/rgb"


std::string save_path = SAVE_PATH_ROOT;

class PoseImageSyncNode : public rclcpp::Node
{
public:
  PoseImageSyncNode() : Node("pose_image_sync_node")
  {

    // 创建 QoS 设置
    // rclcpp::QoS qos_settings(10);
    // qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    rclcpp::QoS qos = rclcpp::QoS(10);
    // 创建rmw_qos_profile_t
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    custom_qos_profile.avoid_ros_namespace_conventions = false;

    // Set up synchronized subscribers, 指定qos
    // pose_sub1_ = message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(this, POSE_TOPIC1, qos_settings);
    // pose_sub1_.subscribe(this, POSE_TOPIC1, qos);
    pose_sub1_.subscribe(this, POSE_TOPIC1, custom_qos_profile);
    image_sub1_.subscribe(this, DEPTH_TOPIC);
    image_sub2_.subscribe(this, RGB_TOPIC);

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PoseStamped,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>
        MySyncPolicy;

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(30), pose_sub1_, image_sub1_, image_sub2_));

    sync_->registerCallback(std::bind(&PoseImageSyncNode::callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));

    // Open text files for writing
    rgb_file_.open(SAVE_PATH_ROOT + std::string("/rgb.txt"));
    depth_file_.open(SAVE_PATH_ROOT + std::string("/depth.txt"));

    if (rgb_file_.is_open())
    {
      rgb_file_ << "# color images\n";
      rgb_file_ << "# file: 'rgbd_dataset_freiburg1_desk.bag'\n";
      rgb_file_ << "# timestamp filename\n";
    }

    if (depth_file_.is_open())
    {
      depth_file_ << "# depth maps\n";
      depth_file_ << "# file: 'rgbd_dataset_freiburg1_desk.bag'\n";
      depth_file_ << "# timestamp filename\n";
    }
  }

  ~PoseImageSyncNode()
  {
    SaveJSON();
    if (rgb_file_.is_open())
    {
      rgb_file_.close();
    }
    if (depth_file_.is_open())
    {
      depth_file_.close();
    }
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
    RCLCPP_INFO(this->get_logger(), "Received synchronized pose and images");

    // Save pose quaternion
    save_pose_quaternion(pose1);

    // Save RGB and depth images using the RGB image's timestamp
    if (index % 1 == 0)
    {
        save_image(image1, SAVE_DEPTH_IMAGE_PATH, index, depth_file_, image2->header.stamp);
        save_image(image2, SAVE_RGB_IMAGE_PATH, index, rgb_file_, image2->header.stamp);
    }

    index++;
  }

  void save_pose_quaternion(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose)
  {
    //创建pose_quat.txt文件
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

  void save_image(const sensor_msgs::msg::Image::ConstSharedPtr msg, const std::string &folder, int index, std::ofstream &file, const rclcpp::Time &reference_stamp)
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

    // Use emplace_back for JSON array
    json_array_.emplace_back(nlohmann::json{
        {"timestamp", reference_stamp.seconds()},
        {"filename", filename}});

    if (file.is_open())
    {
        // Write timestamp in fixed-point format with 6 decimal places
        file << std::fixed << std::setprecision(6) << reference_stamp.seconds() << " " << filename << "\n";
    }
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
              double &r11, double &r12, double &r13, 
              double &r21, double &r22, double &r23, 
              double &r31, double &r32, double &r33)
{
    double sqx = qx * qx;
    double sqy = qy * qy;
    double sqz = qz * qz;
    double sqw = qw * qw;

    r11 = sqw + sqx - sqy - sqz;
    r12 = 2.0 * (qx * qy - qz * qw);
    r13 = 2.0 * (qx * qz + qy * qw);

    r21 = 2.0 * (qx * qy + qz * qw);
    r22 = sqw - sqx + sqy - sqz;
    r23 = 2.0 * (qy * qz - qx * qw);

    r31 = 2.0 * (qx * qz - qy * qw);
    r32 = 2.0 * (qy * qz + qx * qw);
    r33 = sqw - sqx - sqy + sqz;
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
  std::ofstream rgb_file_;
  std::ofstream depth_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseImageSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}