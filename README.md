# Particle Filter Localization  
Particle Filter Localization  is a ros2 package of Particle Filter Based Localization in 3D using GNSS/IMU/Odometry(Visual Odometry/Lidar Odometry).

## node
pf_localization_node
- input  
/initial_pose  (geometry_msgs/PoseStamed) 
/cloud  (sensor_msgs/PointCloud2) 
/map  (sensor_msgs/PointCloud2) 
/imu  (sensor_msgs/Imu)
/odom (nav_msgs/Odometry)
/tf(/base_link(robot frame) → /imu_link(imu frame))  
- output  
/curent_pose (geometry_msgs/PoseStamped)

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|pub_period|int|10|publish period[ms]|
|sigma_gnss_xy|double|0.1|variance of a gnss receiver about position xy[m^2]|
|sigma_gnss_z|double|0.15|variance of a gnss receiver about position z[m^2]|
|sigma_odom_xyz|double|0.1|variance of an odometry[m^2]|
|sigma_imu_w|double|0.01|variance of an angular velocity sensor[(deg/sec)^2]|
|sigma_imu_acc|double|0.01|variance of an accelerometer[(m/sec^2)^2]|
|use_gnss|bool|true|whether gnss is used or not |
|use_odom|bool|false|whether odom(lo/vo) is used or not |

## demo



```
ros2 launch particle_filter_localization pf.launch.py
```

```
ros2 topic pub pf_localization/initial_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 1532228824, nanosec: 55000000}, frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {z: 1, w: 0}}}' --once
```

```
ros2 bag play -s rosbag_v2 test.bag
```


![demo](./images/demo_pfl.gif)    