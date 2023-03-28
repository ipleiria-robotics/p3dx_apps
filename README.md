# p3dx_apps

Applications using the Pioneer P3-DX in the Advanced Robotics and Smart Factories Laboratory.

These notes are currently for ROS 1, and should work (if slight adaptations) for both the melodic and noetic versions in Ubuntu 18.04 and Ubuntu 20.04.

The main source of information is http://wiki.ros.org/Robots/AMR_Pioneer_Compatible.

## Setting up the workspace

Create the workspace folder, if it does not exist yet, then clone our repo into it:
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ipleiria-robotics/p3dx_apps.git
```

## Needed packages

### Pioneer P3-DX driver

We are using p2os, namely the Purdue branch from http://wiki.ros.org/p2os-purdue, which is available in the repositories. To install it, do:
`$ sudo apt install ros-melodic-p2os-doc ros-melodic-p2os-driver ros-melodic-p2os-launch ros-melodic-p2os-msgs ros-melodic-p2os-teleop ros-melodic-p2os-urdf`

### Laser scanner

We are using the sick safety laser scanner, so make sure the corresponding package is installed:
`$ sudo apt install ros-melodic-sick-safetyscanners`

### Wiimote controller

To use the wiimote as a controller,makesure the corresponding package is installed:
`$ sudo apt-get install ros-melodic-wiimote`

### Intel Realsense

We are using the Intel Realsense D435i sensor, namely the [ROS-related package](httpp://wiki.ros.org/realsense2_camera). The easiest way to get it working is to follow the guidelines available [here](https://github.com/intel-ros/realsense/#installation-instructions). We are using the Intel repository library packages (asdescribed [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))but, for the ros package, we are cloning localy in the catkin workspace the original package.

The guide above should advise you to install the following packages:
`$ sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg`

The you clone the realsense-ros repository:
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/IntelRealSense/realsense-ros
```

### RTabMap

We re using the rtabmap package to build a 3D model of the environment,somakesure it is installed:

`$ sudo apt install ros-melodic-rtabmap-ros`


## Lauching the nodes

To launch the robot base plus the sick laser scanner and the intel realsense, use the following command:

TO BE CHECKED:

Make sure your user is part of the dialout group in order to use the Kokuyo laser. For instance, you could use the following for the linux user:
 sudo usermod -a -G dialout linux

```
sudo apt-get install ros-indig-urg-node

sudo apt-get install ros-indigo-p2os-*

sudo apt-get install ros-indigo-wiimote


p2os_driver
```

Remote rviz:

In the remote host do

> export ROS_MASTER_URI=http://10.0.0.1:11311/
>  ≳ export ROS_IP=10.0.0.2
> rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100
