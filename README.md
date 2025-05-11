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
