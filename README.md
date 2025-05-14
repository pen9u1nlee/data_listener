# rgbd2octo

RGBD frame to Octomap

**Requirements:**

```
rtabmap_ros
octomap_server (should be recompiled with macro `COLOR_OCTOMAP_SERVER`)
```

**Usage:**

```
# Step 1. Start a RGBD camera. e.g. 
roslaunch orbbec_camera gemini2.launch

# Step 2. Start rtabmap_ros. Remember to remap the rtabmap RGBD topic subscriptions to align with the cameras'. e.g.
 roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--delete_db_on_start"   rgb_topic:="/camera/rgb/image_raw"   depth_topic:="/camera/depth/image_raw"   camera_info_topic:="/camera/rgb/camera_info"   depth_camera_info_topic:="/camera/depth/camera_info";

# Step 3. Start kf sender.
rosrun data_listener mapdata_sender

# Step 4. (In the cloud) start kf receiver and the octomap node
roslaunch data_listener map_cloud.launch
```

**Usage:update**

# step 1.搭建orbbec_camera相机环境
## 环境搭建

本节介绍如何在自己的主板上，搭建相机运行的环境。以工作空间orbbec_ws为例子，工作空间目录是在~目录下，根据实际情况进行修改。

### 1、安装相关依赖

```
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

```
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

### 2、编译功能包

解压文件orbbec_ws_src.zip，得到一个src文件夹，然后把orbbec-ros-sdk复制到orbbec_ws/src目录下（前边的初始化工作空间的操作这里不在赘述），然后打开终端，输入以下指令进行编译，

```bash
cd ~/orbbec_ws
catkin_make
echo "source ~/orbbec_ws/devel/setup.bash" >> ~/.bashrc
```

### 3、创建串口规则文件

输入以下指令进行创建，

```bash
cd ~/orbbec_ws/src/orbbec-ros-sdk/script
sudo chmod 777 *
sudo sh install.sh
```

然后重新拔插相机，输入以下指令检查是否创建成功，

```
ll /dev/OrbbecGemini2
```

![image-20230612211359686](image-20230612211359686.png)

出现以上画面则表示创建成功，只要是出现有ll /dev/OrbbecGemin2就表示成功绑定了。

### 4、运行相机

Gemini2相机运行，

```bash
roslaunch orbbec_camera gemini2.launch
```

![image-20230612211813826](image-20230612211813826.png)

输入rqt_image_view查看是否有图像数据，

![image-20230302212106594](image-20230302212106594.png)

有图像显示则表示环境搭建成功。

### 注：如果使用的是虚拟机，需要在左上角点击#虚拟机#可移动设备，将orbbec_camera与虚拟机相连，断开与主机的连接。




# step 2. 安装和启动rtabmap_ros

### 1.首先安装依赖库，在终端执行以下命令
sudo apt-get install -y \
>     ros-$ROS_DISTRO-rtabmap-ros \
>     ros-$ROS_DISTRO-navigation \
>     ros-$ROS_DISTRO-hector-slam \
>     ros-$ROS_DISTRO-gmapping \
>     libopencv-dev \
>     python3-catkin-tools \
>     python3-rosdep

### 2.启动rtabmap_ros
roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--delete_db_on_start"   rgb_topic:="/camera/rgb/image_raw"   depth_topic:="/camera/depth/image_raw"   camera_info_topic:="/camera/rgb/camera_info"   depth_camera_info_topic:="/camera/depth/camera_info";




# step 3. 在github上拉取相应代码并进行编译

### 1.首先在终端输入以下命令
git clone https://github.com/pen9u1nlee/data_listener.git

### 2.构建工作空间xxx_ws，这里名字可以自取
在主目录下新建工作空间，并在工作空间中新建src文件，同时将第一步克隆的代码放进src文件中。

### 3.对代码进行编译，在终端输入以下命令
cd xxx_ws 首先进入工作空间目录
catkin_make
#### 这里编译的过程可能会报错，说是找不到httplib.h，解决方法是在data_listener文件夹下新建一个include文件，并在github中拉取httplib.h头文件，放在include中，便可解决这一报错。





#  step 4. 运行kf sender,receiver并启动cotomap节点

### 1. 在运行以上节点之前，需要先安装octomap_server，在终端中输入：
sudo apt install ros-noetic-pcl-conversions ros-noetic-pcl-ros
sudo apt install ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros
#### 上面两行是安装octomap_server的依赖
sudo apt install ros-noetic-octomap-server

### 2. 因为程序中使用到的是彩色octomap地图构建，因此需要执行：
将launch中的第一行代码改为<node pkg="octomap_server" type="octomap_color_server_node" name="octomap_server">，即将octomap_server_node改为octomap_color_server_node，否则会报错。

### 3. 运行kf sender，receiver并启动cotomap节点,打开一个终端，输入：
cd xxx_ws
source devel/setup.bash
如果不想每次都输这句话，可以在终端输入echo "source ~/xxx_ws/devel/setup.bash" >> ~/.bashrc，这样的话重启一下系统，之后就不需要每次都输一遍source devel/setup.bash了。
rosrun data_listener mapdata_sender
新建一个终端，输入：
cd xxx_ws
source devel/setup.bash
roslaunch data_listener map_cloud.launch

### 4.在rviz中安装插件，使得可以直接查看octomap地图
sudo apt install ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins
运行之后便可以在rviz的coloroccupancygrid中订阅octomap_full的话题。



**update**
在launch文件中加入<param name="sensor_model/max_range" value="5.0" />，增加深度限制，限制在5米内。

<param name="resolution" value="0.05" />，分辨率改为0.05米，也就是5厘米。

**在kf2pcl中加入图像旋转，同时调整点云坐标**这个地方我也没搞懂为什么，我是根据rviz中的图像进行调整的，具体原理不清楚。

// 图像逆时针旋转90度函数
cv::Mat rotateImageCounterClockwise90(const cv::Mat& image) {
    cv::Mat rotated;
    cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    return rotated;
}

// 注意：旋转后坐标系发生变化，需要调整点云坐标
pt.x = z_camera;
pt.y = y_camera;
pt.z = x_camera;