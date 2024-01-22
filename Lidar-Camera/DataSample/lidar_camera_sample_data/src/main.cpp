#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/gpgga.h>

#include "sensor_msgs/PointField.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "camera_lidar_sample_data/Order.h"

#include <opencv2/opencv.hpp> 

#include <string.h>
#include <stdio.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "camera_lidar_sample_data/Sampledata.h"
#include <swri_roscpp/publisher.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "FileIO.h"
#include "Types.h"
#include "DrawImg.h"
#include "Tic_Toc.h"

using namespace std;
using namespace sample_data;
using namespace message_filters;

camera_lidar_sample_data::Sampledata sampledata;
ros::Publisher sampledata_pub;
void callback_sampledata(const sensor_msgs::PointCloud2ConstPtr& lidar_msg,
		         const sensor_msgs::ImageConstPtr& leftimg_msg,
			const sensor_msgs::ImageConstPtr& rightimg_msg
			//const novatel_gps_msgs::GpggaConstPtr&  gpgga_msgs)
//			const novatel_gps_msgs::InspvaConstPtr& inspva_msgs
                         )
{ 
  sampledata.lidarpoints = *lidar_msg;
  sampledata.leftimg = *leftimg_msg;
  sampledata.rightimg = *rightimg_msg; 
//  sampledata.inspva =*inspva_msgs;
 // sampledata.gpgga = *gpgga_msgs;
  sampledata_pub.publish(sampledata);
  cout<<"************************************************************************************"<<endl;
  cout<<"发送topic的时间戳 = "<<to_string(ros::Time::now().toSec())<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_lidar_sample_data");
  ros::NodeHandle n;

  sampledata_pub  = n.advertise<camera_lidar_sample_data::Sampledata>("/camera_and_lidar", 5);
   message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/velodyne_points", 5);//5表示缓存数据个数
   //message_filters::Subscriber<sensor_msgs::Image> leftimg_sub(n,"/zed2/zed_node/left/image_rect_color", 50);//原始图像topic = /mynteye/left/image_raw
   //  message_filters::Subscriber<sensor_msgs::Image> leftimg_sub(n,"/mynteye/left_rect/image_rect", 50);//矫正图像topic = /mynteye/left_rect/image_rect
      message_filters::Subscriber<sensor_msgs::Image> leftimg_sub(n,"/mynteye/left/image_raw", 50);
 // message_filters::Subscriber<sensor_msgs::Image> rightimg_sub(n,"/zed2/zed_node/right/image_rect_color", 50);//原始图像 topic = /mynteye/right/image_raw
    // message_filters::Subscriber<sensor_msgs::Image> rightimg_sub(n,"/mynteye/right_rect/image_rect", 50);//矫正图像topic = /mynteye/right_rect/image_rect
      message_filters::Subscriber<sensor_msgs::Image> rightimg_sub(n,"/mynteye/right/image_raw", 50);
//   message_filters::Subscriber<novatel_gps_msgs::Inspva> inspva_sub(n,"/inspva", 100);
   
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
    //10表示缓存队列大小，如果订阅消息发布速度太快超过处理速度，则在队列中最old的消息会被舍弃，使队列一直维持在10个大小
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, leftimg_sub, rightimg_sub);
    sync.registerCallback(boost::bind(&callback_sampledata, _1, _2, _3));   
 
   
/*    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image,sensor_msgs::Image,novatel_gps_msgs::Inspva> MySyncPolicy;
    //10表示缓存队列大小，如果订阅消息发布速度太快超过处理速度，则在队列中最old的消息会被舍弃，使队列一直维持在10个大小
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, leftimg_sub, rightimg_sub,inspva_sub);
    sync.registerCallback(boost::bind(&callback_sampledata, _1, _2, _3, _4));     */
    
    
    
 
  ROS_INFO("数据采集节点启动");
  ros::spin();

  return 0;
}



