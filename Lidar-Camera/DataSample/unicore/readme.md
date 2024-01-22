# unicore
## Instruction

This package mainly includes three main information formats in the integrated navigation equipment of xinxingtong, which are analyzed and sent to the corresponding ROS node for output.The three formats are:
Inspva: Integrated Navigation
Rawimu: pure IMU data type: sensor_msgs/Imu
Gpgga: GPS data



## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) . Set CMakelist.txt to the corresponding version


### 1.2 Install the package **nmea-navsat-drive**
If use Ubuntu16.04 , Input： sudo apt-get install ros-kinetic-nmea-navsat-drive libgps-dev    
If use Ubuntu18.04 , Input： sudo apt-get install ros-melodic-nmea-navsat-drive libgps-dev 



## 2. Build unicore
catkin_make:

```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
```



## 3. Run unicore

```
    roslaunch unicore startgnssimu.launch
```


