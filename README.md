# p3dx_apps
Applications used in the Robotics Laboratory

Make sure your user is part of the dialout group in order to use the Kokuyo laser. For instance, you could use the following for the linux user:
    sudo usermod -a -G dialout linux

    
    
    sudo apt-get install ros-indig-urg-node
    
    sudo apt-get install ros-indigo-p2os-*
    
    sudo apt-get install ros-indigo-wiimote
    
    
    p2os_driver
    
Remote rviz:

 In the remote host do
  > export ROS_MASTER_URI=http://10.0.0.1:11311/
  ≳ export ROS_IP=10.0.0.2
  > rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100