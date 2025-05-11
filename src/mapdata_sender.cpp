#include <ros/ros.h>
#include <rtabmap_msgs/MapData.h>
#include <ros/serialization.h>
#include <vector>
#include <httplib.h>
#include <signal.h>

class MapDataSender {
public:
    MapDataSender() {
        // 初始化 httplib 客户端
        cli = std::make_unique<httplib::Client>("http://127.0.0.1:5000");
        // 订阅话题
        sub_ = nh_.subscribe("/rtabmap/mapData", 10, &MapDataSender::mapDataCallback, this);
    }

    void mapDataCallback(const rtabmap_msgs::MapData::ConstPtr& msg) {
        std::vector<uint8_t> buffer;
        uint32_t buffer_size = ros::serialization::serializationLength(*msg);
        buffer.resize(buffer_size);

        ros::serialization::OStream stream(buffer.data(), buffer_size);
        ros::serialization::serialize(stream, *msg);

        ROS_INFO("Serialized MapData size: %zu bytes", buffer.size());
        sendDataToCloud(buffer);
    }

    void sendDataToCloud(const std::vector<uint8_t>& serialized_data) {
        try {
            // 发送 POST 请求
            auto res = cli->Post("/receive", 
                                 reinterpret_cast<const char*>(serialized_data.data()), 
                                 serialized_data.size(), 
                                 "application/octet-stream");

            if (res) {
                if (res->status == 200) {
                    ROS_INFO("Server response: %s", res->body.c_str());
                } else {
                    ROS_ERROR("Error status %d: %s", res->status, res->body.c_str());
                }
            } else {
                ROS_ERROR("No response from server");
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::unique_ptr<httplib::Client> cli; // 使用智能指针管理 httplib 客户端
};

// 信号处理
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapdata_sender");
    MapDataSender sender;

    signal(SIGINT, signalHandler);
    ros::spin();
    return 0;
}
