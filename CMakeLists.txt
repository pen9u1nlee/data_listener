cmake_minimum_required(VERSION 3.16.0)
set(CMAKE_CXX_STANDARD 17)
project(data_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  cv_bridge
  rtabmap_ros
  tf2_ros
  std_msgs
  geometry_msgs
  rtabmap_msgs
  sensor_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(RTABMAP_INCLUDE_DIRS "/opt/ros/noetic/include/rtabmap-0.21")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RTABMAP_INCLUDE_DIRS}
  "/usr/local/include"
)

# add message file
add_message_files(
  FILES
  keyframe.msg
)

generate_messages(
  DEPENDENCIES
  rtabmap_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Python dependencies

catkin_install_python(PROGRAMS scripts/rgbd2pcl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(mapdata_sender src/mapdata_sender.cpp)
target_link_libraries(mapdata_sender ${catkin_LIBRARIES})

add_executable(mapdata_cloud src/mapdata_cloud.cpp)
target_link_libraries(mapdata_cloud ${catkin_LIBRARIES} 
                                    ${rtabmap_LIBRARIES}
                                    ${OpenCV_LIBRARIES}
                                    /opt/ros/noetic/lib/x86_64-linux-gnu/librtabmap_core.so
                                    /opt/ros/noetic/lib/libcv_bridge.so)

add_executable(kf2pcl src/kf2pcl.cpp)
target_link_libraries(kf2pcl ${catkin_LIBRARIES} 
                             ${PCL_LIBRARIES}
                             ${OpenCV_LIBRARIES}
                             /opt/ros/noetic/lib/libcv_bridge.so)