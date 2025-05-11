#include <ros/ros.h>
#include <data_listener/keyframe.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

// CV-Bridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于 fromMsg
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h> // For tf2::convert
#include <Eigen/Geometry> // 对于 Eigen::Affine3d

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// 坐标变换相关
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

const double fx = 514.0707397460938;
const double fy = 514.3394165039062;
const double cx = 320.46221923828125;
const double cy = 234.55703735351562;
const double depth_scale = 1000.0;

ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;
tf2_ros::TransformBroadcaster *tfb;

void keyframeCallback(const data_listener::keyframe::ConstPtr &msg) {

    cv_bridge::CvImageConstPtr cv_ptr_rgb;
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    cv_ptr_rgb = cv_bridge::toCvCopy(msg->rgb_image, sensor_msgs::image_encodings::BGR8);
    cv_ptr_depth = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_image = cv_ptr_depth->image;
    cv::Mat rgb_image = cv_ptr_rgb->image;

    // fill the transformStamped message body
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "camera0";
    transformStamped.transform.translation.x = msg->pose.position.x;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;

    transformStamped.transform.rotation.x = msg->pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.orientation.w;

    // 将RGBD关键帧信息转化为点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 遍历RGBD帧，添加彩色点云点
    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            // 获取深度值
            uint16_t depth_value_raw = depth_image.at<uint16_t>(v, u);
            double z_camera = static_cast<double>(depth_value_raw) / depth_scale;
            if (z_camera <= 0) {continue;} // 忽略无效深度或太远/太近的点
            
            // 计算相机坐标系下的三维坐标
            double x_camera = (static_cast<double>(u) - cx) * z_camera / fx;
            double y_camera = (static_cast<double>(v) - cy) * z_camera / fy;

            pcl::PointXYZRGB pt;
            pt.x = x_camera;
            pt.y = y_camera;
            pt.z = z_camera;
            // 获取RGB颜色 (OpenCV Mat 是 BGR 顺序)
            cv::Vec3b color = rgb_image.at<cv::Vec3b>(v, u);
            pt.b = color[0];
            pt.g = color[1];
            pt.r = color[2];
            
            cloud->push_back(pt);
        }
    }

    // 发布变换位姿后的点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "camera0"; // 设置坐标系名称
    output.header.stamp = ros::Time::now(); // 设置时间戳
    transformStamped.header.stamp = output.header.stamp;
    tfb->sendTransform(transformStamped);

    pub.publish(output);

}


int main(int argc, char **argv) {
    // 初始化ROS节点 (ROS 1)
    std::cout << "Starting kf2pcl node..." << std::endl;

    ros::init(argc, argv, "kf2cpp"); // 节点名
    ros::NodeHandle nh; // 创建节点句柄

    tf2_ros::TransformBroadcaster t;
    tfb = &t;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/rgbd/pointcloud", 10);
    // 创建一个订阅者
    ros::Subscriber sub = nh.subscribe<data_listener::keyframe>("/rgbd/keyframe", 10, keyframeCallback);

    ROS_INFO("C++ Keyframe listener node started, waiting for messages...");

    // 进入ROS自旋，等待回调函数被触发
    ros::spin();

    return 0;
}