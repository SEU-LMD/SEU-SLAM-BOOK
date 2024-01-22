#DataSample
## Instruction

In this package, the data of images (Zed, Mynteye (20Hz)), 16 line lidar (10Hz), IMU (200Hz) and GPS (gpgga, inspva (1Hz)) are collected by soft synchronization and saved in the data folder. Users can collect the data of a single sensor or their own required sensor data according to their needs.



## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.

ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2 **OpenCV**
Dowload and install instructions can be found at: [OpenCV Installation](http://opencv.org). Tested with OpenCV 3.2.0 .


### 1.3 **PCL**
Follow [PCL Installation](https://github.com/PointCloudLibrary/pcl/releases). Tested with PCL 1.12.0 .


### 1.4 **Eigen3**
Follow [Eigen Installation](http://eigen.tuxfamily.org) Tested with Eigen 3.3 .


### 1.5 **MYNT-EYE-S-SDK-master**
Follow [Mynteye Installation](https://github.com/slightech/MYNT-EYE-S-SDK) . Usage in short,

 make init          init project
   
 make ros          build ros wrapper

 roslaunch mynt_eye_ros_wrapper mynteye.launch           ROS starts the camera 
 
 
### 1.6 **Zed**(Optional)
Follow [Zed Installation](https://github.com/stereolabs/zed-ros-wrapper) . Tested with zed-ros-wrapper-3.4 . For more information, please refer to the blog:
(https://blog.csdn.net/weixin_42148238/article/details/121018271?spm=1001.2014.3001.5501) .

Usage in short,

 roslaunch zed_wrapper zed2.launch           ROS starts the camera 
 
 
### 1.7 **velodyne**
Follow [Velodyne Installation](Velodyne: http://www.ros.org/wiki/velodyne) to download the corresponding version of velodyne driver . Operate:
1. Connect lidar and computer through network cable .
2. Set wired IP address

   Settings - > Network - > wired connection - > IPv4 - > manual
   
   Address: 192.168.1.77
   
   Mask: 255.255.255.0
   
   Gateway: 192.168.1.1
   
   Click Apply
   
3. Open the browser and enter 192.168.1.201 to see the configuration file of lidar

Usage in short, 

 roslaunch velodyne_pointcloud VLP16_points.launch           ROS starts 16 wire lidar
 
 
### 1.8 **minicom**
1.  Installing serial port tool minicom in Ubuntu environment 

Input:    sudo apt-get install minicom

(1)  Serial port configuration

```
sudo minicom -s
```

(2)  Use the arrow keys to select "Serial port setup" and press enter to enter the setting environment ,
Because the USB to RS232 serial port is used,

Set "Serial Device"  /dev/ttyusb0 

Set "Bps/Par/Bits"   115200 8N1

(3)  After configuration, press Enter to return to the previous interface, select "Save setup as dfl", and then Exit to close minicom .

```
sudo chmod 777 /dev/ttyUSB0
```

```
sudo usermod -aG dialout user(There is self named)
```

2. Configure GNSS topic. Read the topic information "/nmea_sentence" of integrated navigation with ROS.

(1)  Install the package nmea-navsat-drive:


If use Ubuntu16.04 , 

```
sudo apt-get install ros-kinetic-nmea-navsat-drive libgps-dev    
```

If use Ubuntu18.04 ,

```
sudo apt-get install ros-melodic-nmea-navsat-drive libgps-dev 
```

(2)  Start roscore

```
 roscore
```
 

(3)  Release location module information:

```
rosrun nmea_navsat_driver nmea_topic_serial_reader _port:=/dev/ttyUSB0 _baud:=115200
```

(4)  To verify that the location data is published successfully

```
  rostopic list
  rostopic echo /nmea_sentence
```

#### minicom has become a communication tool for unicore package to analyze the three main information formats in the integrated navigation equipment of xinxingtong



## 2. Build DataSample
catkin_make:

```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
```



## 3. Run DataSample

```
    roslaunch lidar_camera_sample_data start.launch
```

User can run a single sensor. For example:

### 3.1 Use MYNT-EYE-S-SDK only:

```
 roslaunch mynt_eye_ros_wrapper mynteye.launch           
 ```


### 3.2 Use Zed only:

```
 roslaunch zed_wrapper zed2.launch
```

 
### 3.3 Use 16 line velodyne only:

```
  roslaunch velodyne_pointcloud VLP16_points.launch
```

  
### 3.4 Use GPS only:

```
  roslaunch unicore startgnssimu.launch
 ```




## 4. Pay attention：
By modifying start.launch,users can collect the data of a single sensor or their own required sensor data according to their needs.Pay attention：

1. In start.launch , modifying "path" and launch .
2. In main.cpp , modifying Subscriber .
3. In visualizer.cpp , modifying "mynteye angle" or "zed angle" .
4. In Drawing.h , modifying "cv::cvtColor" .
5. In CMakelist.txt of unicore , modifying ubuntu version .
 
 
 


