# Particle Filter Localization  
Particle Filter Localization  is a ros2 package of Particle Filter Based Localization in 3D using LiDAR/IMU.

## node
pf_localization_node
- input  
/initial_pose  (geometry_msgs/PoseStamed)  
/cloud  (sensor_msgs/PointCloud2)  
/map  (sensor_msgs/PointCloud2)  
/imu  (sensor_msgs/Imu)  
/tf 
- output  
/curent_pose (geometry_msgs/PoseStamped)  
/particles (geometry_msgs/PoseArray)

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|pub_period|int|10|publish period[ms]|
|var_initial_pose|double|0.2|variance of an initial_pose[m^2]|
|var_imu_w|double|0.01|variance of an angular velocity sensor[(deg/sec)^2]|
|var_imu_acc|double|0.01|variance of an accelerometer[(m/sec^2)^2]|
|num_particles|int|100|numbers of particels|
|selected_estimator|std::string|"WeightedAverage"|"MAP" or "WeightedAverage"|
|voxel_leaf_size|double|0.2|a down sample size of a input cloud[m]|

## demo

```
wget https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test.bag
```

```
rviz2 -d src/particle_filter_localization/config/pfl_demo.rviz
```

```
ros2 launch particle_filter_localization pf.launch.py
```

```
ros2 topic pub pf_localization/initial_pose geometry_msgs/PoseStamped '{pose: {position: {x: 0.075, y: -0.019}, orientation: {z: 0.9998, w: 0.0143}}}' --once
```

```
ros2 bag play -s rosbag_v2 short_test.bag
```


![demo](./images/demo_pfl.gif)    

## Used Libraries

- Eigen
- PCL(Point Cloud Library)
