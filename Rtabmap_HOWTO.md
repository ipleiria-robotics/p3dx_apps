Using RTABMAP with the Pioneer and the Advanced Robotics and Smatr Factories lab

We assume here to be using the Intel Realsense D435i and the Pioneer P3-DX. Make sure you went through the README.md file first.

Make sure rtabmap was installed first, using:
$ sudo apt install ros-melodic-rtabmap-ros 

The p3dx_real.launch file already launches the laser, with the remaining nodes being launchedusing the rtabmap.launch file. To launch rtabmap do:
`roslaunch p3dx_apps rtabmap.launch`

The commands below are left herein case,for some reason, you need to launch part or all applications related with rtabmapfrom the command line, without using the provided launch files. Note that the commands below do not include the robot namespace.

# Realsense camera
```bash
roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
    enable_accel:=true
```

# Imu filter
```bash
rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
```

# Mapping
```
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu
```
