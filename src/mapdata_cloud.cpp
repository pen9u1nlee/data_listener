// #include <iostream>
#include <vector>
#include <httplib.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <rtabmap_msgs/MapData.h>
#include <rtabmap_msgs/Node.h>
#include <rtabmap_msgs/SensorData.h>
#include <sensor_msgs/Image.h>
#include <rtabmap/core/Compression.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <data_listener/keyframe.h>
#include <signal.h>

ros::Publisher keyframe_pub;

httplib::Server svr;


// 处理接收到的数据
void handle_receive(const httplib::Request& req, httplib::Response& res) {
    std::vector<uint8_t> received_data(req.body.begin(), req.body.end());
    rtabmap_msgs::MapData map_data;
    
    try {
        // 反序列化 MapData
        ros::serialization::IStream stream(received_data.data(), received_data.size());
        ros::serialization::deserialize(stream, map_data);

        std::cout << "\n=== Received MapData ===" << std::endl;
        std::cout << "Frame ID: " << map_data.header.frame_id << std::endl;
        std::cout << "Timestamp: " << map_data.header.stamp << std::endl;
        std::cout << "Nodes count: " << map_data.nodes.size() << std::endl;

        // 处理每个节点
        for (size_t i = 0; i < 1; ++i) { // map_data.nodes.size(), here we only pick the first kf
            const rtabmap_msgs::Node& node = map_data.nodes[i];
            const auto& pose = node.pose;

            // 打印位姿信息
            std::cout << "[Position] x: " << pose.position.x 
                      << ", y: " << pose.position.y 
                      << ", z: " << pose.position.z << std::endl;
            
            std::cout << "[Orientation] qx: " << pose.orientation.x 
                      << ", qy: " << pose.orientation.y 
                      << ", qz: " << pose.orientation.z 
                      << ", qw: " << pose.orientation.w << std::endl;

            // 创建keyframe.msg话题
            data_listener::keyframe kfmsg;

            // 设置位姿信息
            kfmsg.pose.position.x = pose.position.x;
            kfmsg.pose.position.y = pose.position.y;
            kfmsg.pose.position.z = pose.position.z;
            kfmsg.pose.orientation.x = pose.orientation.x;
            kfmsg.pose.orientation.y = pose.orientation.y;
            kfmsg.pose.orientation.z = pose.orientation.z;
            kfmsg.pose.orientation.w = pose.orientation.w;

            // 发布keyframe消息
            

            std::cout << "Published keyframe message to ROS topic" << std::endl;

            // 处理左图像（RGB）
            if (!node.data.left_compressed.empty()) {
                cv::Mat leftImage = rtabmap::uncompressImage(node.data.left_compressed);
                std::cout << "[Left Image] Size: " << leftImage.cols << "x" << leftImage.rows 
                          << std::endl;
                kfmsg.rgb_image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
            } else {
                std::cout << "[Left Image] No data available" << std::endl;
            }

            // 处理右图像（深度）
            if (!node.data.right_compressed.empty()) {
                cv::Mat depthImage = rtabmap::uncompressImage(node.data.right_compressed);
                std::cout << "[Right Image] Size: " << depthImage.cols << "x" << depthImage.rows 
                         << std::endl; 

                // 发布深度图
                kfmsg.depth_image = *cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImage).toImageMsg();
                
                std::cout << "Published depth image to ROS topic" << std::endl;
            } else {
                std::cout << "[Right Image] No data available" << std::endl;
            }

            keyframe_pub.publish(kfmsg);
            std::cout << "Published keyframe message to ROS topic" << std::endl;
        }

        res.set_content("MapData received and processed successfully", "text/plain");
        
    } catch (const std::exception& e) {
        std::cerr << "Error processing MapData: " << e.what() << std::endl;
        res.status = 400;
        res.set_content("Error: " + std::string(e.what()), "text/plain");
    }
}

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    if (svr.is_running()) {
        svr.stop();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "rtabmap_server");
    ros::NodeHandle nh;

    signal(SIGINT, signalHandler);
    
    // 创建深度图发布器
    keyframe_pub = nh.advertise<data_listener::keyframe>("/rgbd/keyframe", 10);

    // 创建HTTP服务器
    svr.Post("/receive", handle_receive);
    
    std::cout << "RTAB-Map Server running on port 5000" << std::endl;
    std::cout << "Waiting for MapData messages..." << std::endl;
    
    if (!svr.listen("127.0.0.1", 5000)) {
        std::cerr << "Failed to start server on port 5000" << std::endl;
        return 1;
    }
    
    return 0;
}