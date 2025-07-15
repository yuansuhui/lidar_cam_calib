#include "foxglove_lidar_calib/calibrator_node.hpp"
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_field_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "foxglove_lidar_calib/calibrator_node.hpp"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

CalibratorNode::CalibratorNode(const rclcpp::NodeOptions & options)
: Node("calibrator_node", options),
  roll_(0.0), pitch_(0.0), yaw_(0.0)
{
    declare_parameter<std::vector<double>>("camera_matrix", {306.01, 0.0, 317.20, 0.0, 306.00, 183.83, 0.0, 0.0, 1.0});
    get_parameter("camera_matrix", camera_matrix_);

        // 点云订阅
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->twist_callback(msg);
        });

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->pointcloud_callback(msg);
        });

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->image_callback(msg);
        });
    translation_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/translation_adjust", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->translation_callback(msg);
    });


        std::bind(&CalibratorNode::twist_callback, this, std::placeholders::_1);

    // 点云发布
    projected_img_pub_ = create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);

    rgb_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/colored_pointcloud", 10);

    last_twist_time_ = this->now();

    // 初始化旋转矩阵
    update_rotation();
}
void CalibratorNode::translation_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    translation_ = Eigen::Vector3f(-0.08,-0.05,-0.03);
    // translation_ += Eigen::Vector3f(msg->linear.x, msg->linear.y, msg->linear.z);
    RCLCPP_WARN(this->get_logger(), "Updated translation: [%f, %f, %f]",
                translation_.x(), translation_.y(), translation_.z());
}


void CalibratorNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    rclcpp::Time current_time = this->now();  // this 指针生效的成员函数
    double dt = 1.0/180 * 3.14159;

    roll_  += msg->angular.x * dt;
    pitch_ += msg->angular.y * dt;
    yaw_   += msg->angular.z * dt;

    update_rotation();
}

void CalibratorNode::update_rotation()
{
    Eigen::AngleAxisf Rx(roll_, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf Ry(pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf Rz(yaw_, Eigen::Vector3f::UnitZ());
    roll_=0.034907;
    pitch_=4.707149;
    yaw_=1.534143;

    // ZYX 顺序（先绕X轴转，后绕Y，最后绕Z）
    rotation_ = Rz * Ry * Rx;
     RCLCPP_WARN(get_logger(), "update rotation %f %f %f", roll_, pitch_,yaw_);
}

void CalibratorNode::image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try {
        latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void CalibratorNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{

    double k1 = -0.0320265479385853;
    double k2 =  0.0358092933893204;
    double p1 =  0.000427532620960847;
    double p2 = -0.000534701568540186;
    double k3 = -0.0124573204666376;

    std::vector<double> dist_coeffs = {k1, k2, p1, p2, k3};



    if (latest_image_.empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *input);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());

    Eigen::Matrix3f K;
    K << camera_matrix_[0], camera_matrix_[1], camera_matrix_[2],
         camera_matrix_[3], camera_matrix_[4], camera_matrix_[5],
         camera_matrix_[6], camera_matrix_[7], camera_matrix_[8];
    float fx = K(0,0);
    float fy = K(1,1);
    float cx = K(0,2);
    float cy = K(1,2);
    for (auto& pt : input->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f p_cam = rotation_ * (p+translation_);

        if (p_cam.z() <= 0) continue;

        Eigen::Vector3f uv = K * p_cam;
        // float x = p_cam.x() / p_cam.z();
        // float y = p_cam.y() / p_cam.z();
        // float r2 = x*x + y*y;
        // float r4 = r2 * r2;
        // float r6 = r2 * r4;

        // float x_d = x * (1 + k1*r2 + k2*r4 + k3*r6) + 2*p1*x*y + p2*(r2 + 2*x*x);
        // float y_d = y * (1 + k1*r2 + k2*r4 + k3*r6) + 2*p2*x*y + p1*(r2 + 2*y*y);

        // int u = static_cast<int>(fx * x_d + cx);
        // int v = static_cast<int>(fy * y_d + cy);

        int u = static_cast<int>(uv.x() / uv.z());
        int v = static_cast<int>(uv.y() / uv.z());

        if (u < 0 || v < 0 || u >= latest_image_.cols || v >= latest_image_.rows) continue;

        cv::Vec3b color = latest_image_.at<cv::Vec3b>(v, u);

        pcl::PointXYZRGB p_colored;
        p_colored.x = pt.x;
        p_colored.y = pt.y;
        p_colored.z = pt.z;
        p_colored.r = color[2];
        p_colored.g = color[1];
        p_colored.b = color[0];

        output->points.push_back(p_colored);
    }

    output->header = input->header;
    sensor_msgs::msg::PointCloud2 colored_msg;
    pcl::toROSMsg(*output, colored_msg);
    rgb_pc_pub_->publish(colored_msg);
    // 复制图像，避免修改原始图像
    cv::Mat image_with_points = latest_image_.clone();

    for (auto& pt : input->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f p_cam = rotation_ * p;

        if (p_cam.z() <= 0) continue;

        Eigen::Vector3f uv = K * p_cam;
        int u = static_cast<int>(uv.x() / uv.z());
        int v = static_cast<int>(uv.y() / uv.z());

        if (u < 0 || v < 0 || u >= image_with_points.cols || v >= image_with_points.rows)
            continue;

        // 在图像上画一个绿色的小圆点表示该点
        cv::circle(image_with_points, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
    }

    // 发布新的图像消息
    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(
        msg->header,  // 与点云同步
        "bgr8",
        image_with_points
    ).toImageMsg();

    projected_img_pub_->publish(*msg_out);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
