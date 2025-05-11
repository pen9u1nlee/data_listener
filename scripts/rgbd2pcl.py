#!/usr/bin/env python
import rospy
from data_listener.msg import keyframe
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import tf.transformations

import cv2
import numpy as np

# 相机内参
fx = 514.0707397460938
fy = 514.3394165039062
cx = 320.46221923828125
cy = 234.55703735351562
depth_scale_ = 1000.0

def transform_point(position, orientation, point_camera):
    """
    point_camera: np.array
    """

    # 将四元数转换为旋转矩阵
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]

    # 将位置转换为numpy数组
    translation = np.array([position.x, position.y, position.z])

    # 执行变换
    point_world = np.dot(rotation_matrix, point_camera) + translation

    return point_world

def keyframe_callback(msg):
    

    bridge = CvBridge()
    try:
        # 将RGB和深度图像消息转换为OpenCV图像
        rgb_image = bridge.imgmsg_to_cv2(msg.rgb_image, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(msg.depth_image, "mono16")

        # 获取位姿信息
        position = msg.pose.position
        orientation = msg.pose.orientation
        print(f"orientation: {[orientation.x, orientation.y, orientation.z, orientation.w]}, position: {[position.x, position.y, position.z]}")

        # 初始化点云数据
        points = []
        height, width = depth_image.shape

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u] * 1.0 / depth_scale_
                if z > 0:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    # 相机坐标系下的点
                    point_camera = np.array([x, y, z])

                    # 变换到世界坐标系
                    point_world = transform_point(position, orientation, point_camera)

                    r, g, b = rgb_image[v, u]
                    rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                    points.append([point_world[0], point_world[1], point_world[2], rgb])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
        

        # build a header for the point cloud
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pclmsg = pc2.create_cloud(header, fields, points)

        # 发布点云消息
        pointcloud_pub.publish(pclmsg)
        
        # 计算start到end之间的秒数并且打印
        all_interval = all_end - all_start
        print(f"Processing time all: {all_interval:.4f} seconds")
        for_interval = for_end - for_start
        print(f"Processing time for: {for_interval:.4f} seconds")

    except Exception as e:
        rospy.logerr("Error: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('rgbd2pcl', anonymous=True)

    # 订阅 keyframe 话题
    rospy.Subscriber('/rgbd/keyframe', keyframe, keyframe_callback)

    # 发布点云话题
    pointcloud_pub = rospy.Publisher('/rgbd/pcl', PointCloud2, queue_size=10)

    rospy.spin()
