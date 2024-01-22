#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/gpgga.h>

#include "sensor_msgs/PointField.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>
#include "camera_lidar_sample_data/Order.h"



#include <opencv2/opencv.hpp> 

#include <string.h>
#include <chrono>//用于测试时间
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>

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

float PI = 3.1415926;
int ifGNSS;

Eigen::Vector3f t_lidarTocamera;
Eigen::Matrix3f R_lidarTocamera;
sample_data::CAMERAINFO camerainfo;

//int skip = 0;//每多少个数据进行一次显示
//int count_received=0;
string saveroot_path,root_path;

ros::Publisher colpoints_pub,savepoints_pub;//发布所有点的topic
ros::Subscriber imu_sub,inspva_sub,gpgga_sub,rawimu_sub;

float start_img_angle = -1.0, end_img_angle = 1.0;//图像对应38.146度。0.7854
bool quit = false;//当用户想要主动停止数据集采集时这个标志位为true
queue<camera_lidar_sample_data::Sampledata> lidar_camera_datas;
queue<camera_lidar_sample_data::Sampledata> current_lidar_camera_datas;
queue<sensor_msgs::Imu> imu_datas;
queue<novatel_gps_msgs::Inspva> inspva_datas;
//queue<nmea_msgs::Gpgga> gpgga_datas;
queue<novatel_gps_msgs::Gpgga> gpgga_datas;
queue<sensor_msgs::Imu> rawimu_datas;

std::mutex mu_save;
std::mutex mu_vis;

std::condition_variable cond_save;
std::condition_variable cond_vis;

//显示耗费时间大约是120-130ms之间，我们订阅的消息发送的频率是在10hz，因此我们隔一个数据进行一次显示
void dataCallback(const camera_lidar_sample_data::Sampledata::ConstPtr& data_msg)       
{ 	   
	   if(!quit)
	   {
	      std::unique_lock<std::mutex> locker_save(mu_save);
  	      lidar_camera_datas.push(*data_msg);//每一帧的数据都进行保存
	      cout<<"接受到topic的时间戳 = "<<to_string(ros::Time::now().toSec())<<endl;
	      locker_save.unlock();
	      cond_save.notify_all();
	      
	      std::unique_lock<std::mutex> locker_vis(mu_vis);
	      current_lidar_camera_datas.push(*data_msg);
	      if(current_lidar_camera_datas.size()==2)
		current_lidar_camera_datas.pop();
	      locker_vis.unlock();
	      cond_vis.notify_all();
	   }
}
void callback_imu(const sensor_msgs::ImuConstPtr&   imu_msg)
{
 imu_datas.push(*imu_msg);

  while(imu_datas.size()!=0)
  {
  sensor_msgs::Imu ros_imu =imu_datas.front();
  imu_datas.pop();
  double timestamp_imu = ros_imu.header.stamp.toSec();
   Write_imu(root_path+"data/gnssimu/"+"imu.txt",ros_imu);
  }
}
  void callback_inspva(const novatel_gps_msgs::InspvaConstPtr& inspva_msgs)
{
  inspva_datas.push(*inspva_msgs);
  while(inspva_datas.size()!=0)
  { 
  novatel_gps_msgs::Inspva ros_ins_pva = inspva_datas.front();
  inspva_datas.pop();
  double timestamp_inspva=ros_ins_pva.header.stamp.toSec();
   Write_inspva(root_path+"data/gnssimu/"+"inspva.txt",ros_ins_pva); 
  }
}  
  void callback_gpgga(const novatel_gps_msgs::GpggaConstPtr&  gpgga_msgs)
{
  gpgga_datas.push(*gpgga_msgs);
  while(gpgga_datas.size()!=0)
  {
  novatel_gps_msgs::Gpgga ros_gpgga =gpgga_datas.front();
  gpgga_datas.pop();
  double timestamp_gpgga=ros_gpgga.header.stamp.toSec();
  Write_gpgga(root_path+"data/gnssimu/"+"gpgga.txt",ros_gpgga);
  }
}  
  void callback_rawimu(const sensor_msgs::ImuConstPtr& rawimu_msgs)
{
  rawimu_datas.push(*rawimu_msgs);
   while(rawimu_datas.size()!=0)
     {
   sensor_msgs::Imu ros_raw_imu = rawimu_datas.front();   
   rawimu_datas.pop();
   double timestamp_rawimu=ros_raw_imu.header.stamp.toSec();
   Write_rawimu(root_path+"data/gnssimu/"+"rawimu.txt",ros_raw_imu); 
     }
}  

void save_process()
{
  
  cv::Mat leftimg,rightimg;
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;//全局 保存点云数据

  while(1)
  {
    std::unique_lock<std::mutex> locker(mu_save);
    while(lidar_camera_datas.size()==0)
    {
      cond_save.wait(locker);
    }
    if(lidar_camera_datas.size()!=0)
    {
	
        sensor_msgs::Image ros_left_img = lidar_camera_datas.front().leftimg;
	sensor_msgs::Image ros_right_img = lidar_camera_datas.front().rightimg;
	sensor_msgs::PointCloud2 ros_lidr = lidar_camera_datas.front().lidarpoints;
	novatel_gps_msgs::Inspva ros_inspva = lidar_camera_datas.front().inspva;
	
	lidar_camera_datas.pop();
	locker.unlock();
	
	//将左右图像数据转换成opencv格式
	cv_bridge::CvImagePtr cv_ptrLeft;
	cv_ptrLeft = cv_bridge::toCvCopy(ros_left_img);
	leftimg =  (cv_ptrLeft->image).clone();
	cv_bridge::CvImagePtr cv_ptrRight;
	cv_ptrRight = cv_bridge::toCvCopy(ros_right_img);
	rightimg = (cv_ptrRight->image).clone();
	//将接收到的激光数据变换到pcl数据下
	pcl::fromROSMsg(ros_lidr, laserCloudIn); //pcl和sensormsg之间的转换，转换为模板点云laserCloudIn
	//
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn_;
	for(int j=0;j<laserCloudIn.size();j++)
	{
    
		pcl::PointXYZ temp;
		temp.x = laserCloudIn[j].x; 
		temp.y = laserCloudIn[j].y;
		temp.z = laserCloudIn[j].z;
    float horizonAngle = -atan2(temp.y, temp.x)*180/PI+180;//输出范围0-360度
		//if((horizonAngle>=315)||(horizonAngle<=225))        //zed angle
                  if((horizonAngle>=45)&&(horizonAngle<=315))                      //  mynteye angle
		{
			laserCloudIn_.push_back(temp);
		}	
	}
  laserCloudIn=laserCloudIn_;
  //

	//获得时间戳
	iTicToc costime_total;
	double timestamp_rightimg = ros_right_img.header.stamp.toSec();
	double timestamp_leftimg = ros_left_img.header.stamp.toSec();
	double timestamp_lidar = ros_lidr.header.stamp.toSec();
	double timestamp_inspva=ros_inspva.header.stamp.toSec();
	cv::imwrite(root_path+"data/leftImg/left_"+to_string(timestamp_leftimg)+".png",leftimg); 
	cv::imwrite(root_path+"data/rightImg/right_"+to_string(timestamp_rightimg)+".png",rightimg);
	Write_lidarpoints(root_path+"data/lidar/"+to_string(timestamp_lidar)+".txt",laserCloudIn.makeShared());
	Write_inspva(root_path+"data/gnssimu/"+"inspva.txt",ros_inspva);
	Write_names(root_path+"data/names.txt",timestamp_lidar,timestamp_leftimg,timestamp_inspva);
	   cout<<"保存耗费时间(ms) = "<<costime_total.toc()<<endl;   	     
       }
      else if((lidar_camera_datas.size()==0)&&quit)
      {
	locker.unlock();
	break;
      }
   }
  cout<<"保存数据结束"<<endl;
}

//可视化线程
void visualizer()
{
  cout<<"111111111"<<endl;
  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);//opencv新建一个窗口
  while(1)
  {
    std::unique_lock<std::mutex> locker(mu_vis);
    cond_vis.wait(locker);
    
    sensor_msgs::Image ros_left_img = current_lidar_camera_datas.front().leftimg;
    //sensor_msgs::Image ros_right_img = current_lidar_camera_datas.front().rightimg;
    sensor_msgs::PointCloud2 ros_lidr = current_lidar_camera_datas.front().lidarpoints;
    locker.unlock();
    
    //count_received++;
    //if(count_received<skip)
    //{
      TicToc costime_total;
      cv::Mat leftimg,rightimg;
      pcl::PointCloud<pcl::PointXYZ> laserCloudIn;//全局 保存点云数据
      double timestamp_rightimg,timestamp_leftimg,timestamp_lidar;
      

      //将接收到的左图像变换到opencv数据类型下
      cv_bridge::CvImagePtr cv_ptrLeft;
      cv_ptrLeft = cv_bridge::toCvCopy(ros_left_img);
      leftimg =  (cv_ptrLeft->image).clone();
      
      //将接收到的激光数据变换到pcl数据下
      pcl::fromROSMsg(ros_lidr, laserCloudIn); //pcl和sensormsg之间的转换，转换为模板点云laserCloudIn
      //获得激光时间戳
      timestamp_lidar = ros_lidr.header.stamp.toSec();
      static double previous_lidar_time = timestamp_lidar;//只会执行一次
      previous_lidar_time = timestamp_lidar;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_ptr = laserCloudIn.makeShared();
      //根据角度对点云数据进行删减
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_img_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      for(int i=0;i<cloud_ptr->points.size();i++)
      {
	float x = cloud_ptr->points[i].x;
	float y = cloud_ptr->points[i].y;
	float angle = atan2(y,x);
	cloud_save_ptr->points.push_back(cloud_ptr->points[i]);
	if((angle>start_img_angle)&&(angle<end_img_angle))//只保存正前方的点
	{
	  cloud_img_ptr->points.push_back(cloud_ptr->points[i]);
	}
      }
      //开始绘制图像，并将激光附上颜色
      Eigen::Matrix<float,1,5> dis_zero;
      dis_zero.setZero();
      pcl::PointCloud<pcl::PointXYZRGB> colPoints;//发布和图像融合后的彩色点
      cv::Mat color_img = sample_data::DrawPointCloud(cloud_img_ptr,leftimg, R_lidarTocamera,t_lidarTocamera,camerainfo.P_l,dis_zero,
						      colPoints);
      cv::imshow("Image", color_img); // 显示图像img
      cv::waitKey(1);
      //将彩色点云发布出来
      sensor_msgs::PointCloud2 pub_colorpointcloud,pub_savepointcloud;
      pcl::toROSMsg(colPoints,pub_colorpointcloud);
      pub_colorpointcloud.header.frame_id = "velodyne";
      pub_colorpointcloud.header.stamp = ros_lidr.header.stamp;
      colpoints_pub.publish(pub_colorpointcloud);//color_points
      
      pcl::toROSMsg(*cloud_save_ptr,pub_savepointcloud);
      pub_savepointcloud.header.frame_id = "velodyne";
      pub_savepointcloud.header.stamp = ros_lidr.header.stamp;
      savepoints_pub.publish(pub_savepointcloud);
      cout<<"显示耗费时间(ms) = "<<costime_total.toc()<<endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle n;
  
  int save_data =0;
  n.getParam("/visualizer/rootpath", root_path);
  //n.getParam("/visualizer/skip", skip);
  //n.getParam("/visualizer/save_data", save_data);
  
  cout<<"root path = "<<root_path<<endl;
  Delete_Mk(root_path+"data");//删除并新建文件夹
  ofstream f_name;
  f_name.open(root_path+"data/names.txt");//相当于删除内容,因为后面对names.txt都是增加写
  f_name.close();
  
  std::thread save_thread(save_process);//开启保存数据线程
  std::thread vis_thread(visualizer);//开启显示线程
 // std::thread baocu_thread(save_msg);//开启保存其他数据
  //读取标定
  read_lidartocamera(root_path+"/conf/lidar_to_camera.txt",t_lidarTocamera,R_lidarTocamera);
  camerainfo = read_cameraInfo(root_path+"/conf/camera.txt");
  
  
  ros::Subscriber sub_data = n.subscribe("/camera_and_lidar", 1000, dataCallback);//非常重要的回调函数 
 // imu_sub = n.subscribe("/mynteye/imu/data_raw", 1000, callback_imu);
   imu_sub = n.subscribe("/zed2/zed_node/imu/data_raw", 1000, callback_imu);
   inspva_sub = n.subscribe("/inspva", 1000,callback_inspva);
   gpgga_sub = n.subscribe("/gpgga", 1000,callback_gpgga);
   rawimu_sub = n.subscribe("/rawimu", 1000,callback_rawimu);
 
  
  colpoints_pub = n.advertise<sensor_msgs::PointCloud2>("/color_points",50);
  savepoints_pub = n.advertise<sensor_msgs::PointCloud2>("/save_points",50);
  

  ros::spin();
  quit = true;
  
  return 0;
}



