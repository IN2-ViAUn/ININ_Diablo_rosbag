#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <filesystem>

#define POSE_TOPIC "/state_estimation"
#define RGB_TOPIC "/rgb/image_raw"
#define DEPTH_TOPIC "/depth_to_rgb/image_raw"

#define SAVE_ROOT_DIR "/home/yangdianyi/colcon_ws/rosbag_save_3_29"
#define POSE_FILE SAVE_ROOT_DIR "/trajectory.txt"
#define RGB_DIR SAVE_ROOT_DIR "/rgb"
#define DEPTH_DIR SAVE_ROOT_DIR "/depth"

class BagExtractor : public rclcpp::Node {
public:
    BagExtractor() : Node("bag_extractor") {
        // 确保输出目录存在
        ensure_directory(SAVE_ROOT_DIR);
        ensure_directory(RGB_DIR);
        ensure_directory(DEPTH_DIR);
        
        // 初始化轨迹文件
        init_trajectory_file();
        
        // 创建订阅器
        pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            POSE_TOPIC, 10, std::bind(&BagExtractor::pose_callback, this, std::placeholders::_1));
        
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            RGB_TOPIC, 10, std::bind(&BagExtractor::rgb_callback, this, std::placeholders::_1));
        
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            DEPTH_TOPIC, 10, std::bind(&BagExtractor::depth_callback, this, std::placeholders::_1));
    }

private:
    void init_trajectory_file() {
        std::ofstream file(POSE_FILE);
        if (file) {
            file << "# ground truth trajectory\n";
            file << "# timestamp tx ty tz qx qy qz qw\n";
            file.close();
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to create trajectory file");
        }
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        std::ofstream file(POSE_FILE, std::ios::app);
        if (!file) {
            RCLCPP_ERROR(get_logger(), "Failed to open trajectory file for writing");
            return;
        }
        
        // 获取时间戳(秒.纳秒)并格式化为6位小数
        double timestamp = pose->header.stamp.sec + pose->header.stamp.nanosec * 1e-9;
        
        // 写入姿态数据，统一使用6位小数精度
        file << std::fixed << std::setprecision(6) << timestamp << " "
             << pose->pose.pose.position.x << " "
             << pose->pose.pose.position.y << " "
             << pose->pose.pose.position.z << " "
             << pose->pose.pose.orientation.x << " "
             << pose->pose.pose.orientation.y << " "
             << pose->pose.pose.orientation.z << " "
             << pose->pose.pose.orientation.w << "\n";
    }

    void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
        save_image(img, RGB_DIR, "rgb");
    }

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
        save_image(img, DEPTH_DIR, "depth");
    }

    void save_image(const sensor_msgs::msg::Image::ConstSharedPtr& img, 
                   const std::string& dir, const std::string& prefix) {
        try {
            // 转换时间戳为文件名，统一使用6位小数
            double timestamp = img->header.stamp.sec + img->header.stamp.nanosec * 1e-9;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6) << timestamp;
            std::string filename = dir + "/" + oss.str() + ".png";
            
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr;
            if (img->encoding == "16UC1" || img->encoding == "32FC1") {
                // 深度图像处理
                if (img->encoding == "32FC1") {
                    // 32位浮点深度图转换为16位
                    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
                    cv::Mat depth_mm;
                    cv_ptr->image.convertTo(depth_mm, CV_16UC1, 1000.0); // 转换为毫米单位
                    cv_ptr->image = depth_mm;
                } else {
                    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
                }
            } else {
                // RGB图像
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            }
            
            // 保存图像
            if (!cv::imwrite(filename, cv_ptr->image)) {
                RCLCPP_ERROR(get_logger(), "Failed to save image: %s", filename.c_str());
            } else {
                RCLCPP_INFO(get_logger(), "Saved image: %s", filename.c_str());
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void ensure_directory(const std::string& path) {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
            RCLCPP_INFO(get_logger(), "Created directory: %s", path.c_str());
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    std::mutex file_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}