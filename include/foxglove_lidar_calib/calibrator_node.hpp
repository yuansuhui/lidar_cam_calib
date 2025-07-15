#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class CalibratorNode : public rclcpp::Node
{
public:
    explicit CalibratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // 回调函数
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void translation_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // 更新旋转矩阵
    void update_rotation();

    // 定时器周期调用更新RPY（也可以只在收到twist时调用update_rotation）
    void update_rpy();

    // 订阅器和发布器
    Eigen::Vector3f translation_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr translation_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgb_pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr projected_img_pub_;


    rclcpp::TimerBase::SharedPtr timer_;

    // 参数及状态
    std::vector<double> camera_matrix_;
    double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
    Eigen::Matrix3f rotation_;

    cv::Mat latest_image_;

    rclcpp::Time last_twist_time_;
};
