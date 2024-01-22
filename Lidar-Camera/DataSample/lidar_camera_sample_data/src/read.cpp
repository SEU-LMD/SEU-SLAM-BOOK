#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>
#include "sensor_msgs/Imu.h"
#include "novatel_gps_msgs/Inspva.h"


#include "camera_lidar_sample_data/Sampledata.h"
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/gpgga.h>

#include <opencv2/opencv.hpp> 

#include <string.h>
#include <chrono>//用于测试时间
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <thread>

#include "FileIO.h"
#include "Types.h"
#include "DrawImg.h"
#include "Tic_Toc.h"
using namespace std;
using namespace sample_data;
sensor_msgs::Image rosData_leftimg;
sensor_msgs::Image rosData_rightimg;
sensor_msgs::PointCloud2 rosData_lidar;
sensor_msgs::Imu rosData_imu;
novatel_gps_msgs::Inspva rosData_inspva;
novatel_gps_msgs::Gpgga rosData_gpgga;
sensor_msgs::Imu rosData_rawimu;
string root_path;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read");
  ros::NodeHandle n;
  n.getParam("/read/root_path", root_path);//加入详细日期

  //ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/mynteye/imu/data_raw",1);
 
  //GetGnssLidardata(root_path+"data/names.txt",root_path,rosData_lidar,rosData_gpgga);
  //GetGnssCameredata(root_path+"data/names.txt",root_path,rosData_leftimg, rosData_rightimg, rosData_gpgga);
  //GetLidarCameredata(root_path+"data/names.txt",root_path,rosData_leftimg, rosData_rightimg, rosData_lidar);   
  //GetGnssLidarCameredata(root_path+"data/names.txt",root_path,rosData_leftimg, rosData_rightimg, rosData_lidar,rosData_gpgga);
  //GetInspvadata(root_path+"data/gnssimu/inspva.txt",root_path,rosData_inspva);
  GetRawImudata(root_path+"data/gnssimu/rawimu.txt",root_path,rosData_rawimu);
  //GetImudata(root_path+"data/gnssimu/imu.txt",root_path,rosData_imu);
  

    
}


