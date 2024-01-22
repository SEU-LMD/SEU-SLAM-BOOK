#ifndef _FILEIO_
#define _FILEIO_
#include "ros/ros.h"
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <dirent.h>  //用于删除目录
#include <sys/stat.h>//用于新建目录
#include <stdio.h>


#include <opencv2/opencv.hpp>  
#include <opencv2/calib3d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>



#include "sensor_msgs/Imu.h"
#include "novatel_gps_msgs/Inspva.h"
#include "nmea_msgs/Gpgga.h"
#include "camera_lidar_sample_data/Sampledata.h"

#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#include "Types.h"
using namespace std;
//这个文件的作用是读取文件到程序中
namespace sample_data
{       
	
  class Inspva {
  public:

    double time = 0.0;
    int week = 0;
    double second_of_week=0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    double north_velocity = 0.0;
    double east_velocity = 0.0;
    double up_velocity = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;
    string status;
    
    int extended_status = 0;
    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;
    
  };
  
  
    class Imu {
     public:
       
    double time = 0.0; 
    double orientation_x=0.0;
    double orientation_y=0.0;
    double orientation_z=0.0;
    double orientation_w=0.0;
    double angular_velocity_x=0.0;
    double angular_velocity_y=0.0;
    double angular_velocity_z=0.0;
    double linear_acceleration_x=0.0;
    double linear_acceleration_y=0.0;
    double linear_acceleration_z=0.0;
    
  };
  
  
  
  class RawImu {
     public:
       
     double time = 0.0;
  /*   int week = 0;
     double seconds_into_week=0.0;
     int imu_state=0;   */
     double z_accel_output=0.0;
     double y_accel_output=0.0;
     double x_accel_output=0.0;
     double z_gyro_output=0.0;
     double y_gyro_output=0.0;
     double x_gyro_output=0.0;
     
  };
  
  
  
  class Gpgga{
    public:
   double time = 0.0;
   double utc_seconds=0.0;
   double lat=0.0;
   double lon=0.0;
   string lat_dir;
   string lon_dir;
   int gps_qual=0;
   int num_sats=0;
   float hdop=0.0;
   float alt=0.0;
   string altitude_units;
   float undulation=0.0;
   string undulation_units;
   int diff_age=0;
   string station_id;
  }; 
  
  	/*   class multidata
	{
	public:

	 sensor_msgs::Image read_Img(string path,double stamp );
	 sensor_msgs::PointCloud2 read_Lidar(string path,double stamp);
	 novatel_gps_msgs::Gpgga read_Gpgga(string path,double stamp);
	 unicore::RawImu  read_RawImu(string path,double stamp,int l);
	 sensor_msgs::Imu read_Imu(string path,double stamp,int s);
	 novatel_gps_msgs::Inspva read_Inspva(string path,double stamp,int m);
	  novatel_gps_msgs::Gpgga read_Gpgga(string path,double stamp,int w);
	 void GetGnssLidarCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    sensor_msgs::PointCloud2 rosData_lidar, 
				    novatel_gps_msgs::Gpgga rosData_gpgga);
	 void GetLidarCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    sensor_msgs::PointCloud2 rosData_lidar);
	  void GetGnssCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    novatel_gps_msgs::Gpgga rosData_gpgga);
	  void GetGnssLidardata(string path, string root_path, 
				    sensor_msgs::PointCloud2 rosData_lidar, 
				    novatel_gps_msgs::Gpgga rosData_gpgga);
	  void GetInspvadata(string path, string root_path,
			      novatel_gps_msgs::Inspva rosData_inspva);
	  void GetRawImudata(string path, string root_path,
			     unicore::RawImu rosData_rawimu);
	  void GetImudata(string path, string root_path,
			     sensor_msgs::Imu rosData_imu);
	  };   */
    

  //删除文件目录中的所有文件
	void Delete_path(string path)
	{  
          
	  DIR *pDir = NULL;  
	  struct dirent *dmsg;  
	  char szFileName[128];  
	  char szFolderName[128];  
	
	  strcpy(szFolderName, path.c_str());  
	  strcat(szFolderName, "/%s");  
	  if ((pDir = opendir(path.c_str())) != NULL)  
	  {  
	      // 遍历目录并删除文件  
	      while ((dmsg = readdir(pDir)) != NULL)  
	      {  
		  if (strcmp(dmsg->d_name, ".") != 0 && strcmp(dmsg->d_name, "..") != 0)  
		  {  
		      sprintf(szFileName, szFolderName, dmsg->d_name);  
		      string tmp = szFileName;  
		      //如果是文件夹，名称中不包含"."  
		      if (tmp.find(".") == -1){  
			  Delete_path(szFileName);  
		      }  
		      remove(szFileName);  
		  }  
	      }
	  }  
	
	  if (pDir != NULL)  
	  {  
	      closedir(pDir);  
	  }  
	  cout<<"delete "<<path<<" all files"<<endl;
	}  
	
	//删除
	void Delete_Mk(string path)
	{
	  Delete_path(path);
	  //新建文件夹
	  string path_leftimg = path+"/leftImg";
	  string path_rightimg = path+"/rightImg";
	  string path_lidar = path+"/lidar";
	  string path_gnssimu = path+"/gnssimu";
	  string path_seg = path+"/segment_points";
	  mkdir(path_leftimg.c_str(),0777);
	  mkdir(path_rightimg.c_str(),0777);
	  mkdir(path_lidar.c_str(),0777);
	  mkdir(path_gnssimu.c_str(),0777);
	  //mkdir(path_seg.c_str(),0777);
	}
	
	void Write_lidarpoints(string path,pcl::PointCloud<pcl::PointXYZ>::Ptr lidarpoints)
	{
	  ofstream f_lidar;
	  f_lidar.open(path);
	  for(auto i=0;i<lidarpoints->points.size();i++)
	  {
	    f_lidar<<lidarpoints->points[i].x<<" "<<lidarpoints->points[i].y<<" "<<lidarpoints->points[i].z<<endl;
	  }
	  f_lidar.close();
	}
	void Write_inspva(string path,novatel_gps_msgs::Inspva ros_ins_pva)
	{
	  ofstream f_inspva;
	  f_inspva.open(path,ios::app);
	  Inspva inspva;
          inspva.time = ros_ins_pva.header.stamp.toSec();
	  inspva.week = ros_ins_pva.week;
	  inspva.second_of_week=ros_ins_pva.seconds;
          inspva.latitude = ros_ins_pva.latitude;
          inspva.longitude = ros_ins_pva.longitude;
	  inspva.height=ros_ins_pva.height;
	  inspva.north_velocity = ros_ins_pva.north_velocity;
	  inspva.east_velocity=ros_ins_pva.east_velocity;
	  inspva.up_velocity=ros_ins_pva.up_velocity;
	  inspva.roll=ros_ins_pva.roll;
	  inspva.pitch=ros_ins_pva.pitch;
	  inspva.azimuth=ros_ins_pva.azimuth;
	  inspva.status=ros_ins_pva.status;
	//  f_inspva<<"# rostime,  week, second_of_week, latitude, longitude, height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth, status"<<endl;
	//  for(auto i=0;i<lidarpoints->points.size();i++)
	  //{
	  
          //inspva.status = nav_sat_fix_ptr.status.status;
	  f_inspva<<to_string(inspva.time)<<" "<<to_string(inspva.week)<<" "<<to_string(inspva.second_of_week)<<" "<<to_string(inspva.latitude)<<" "<<to_string(inspva.longitude)<<" "<<to_string(inspva.height)<<" "<<to_string(inspva.north_velocity)<<" "<<to_string(inspva.east_velocity)<<" "<<to_string(inspva.up_velocity)<<" "<<to_string(inspva.roll)<<" "<<to_string(inspva.pitch)<<" "<<to_string(inspva.azimuth)<<" "<<inspva.status<<endl;
	//  }
	  f_inspva.close();
	} 
	void Write_imu(string path,sensor_msgs::Imu ros_imu)
	{
	  ofstream f_imu;		
	  f_imu.open(path,ios::app);
	  Imu imu;
	  imu.time=ros_imu.header.stamp.toSec();
          imu.orientation_x = ros_imu.orientation.x;
	  imu.orientation_y = ros_imu.orientation.y;
	  imu.orientation_z = ros_imu.orientation.z;
	  imu.orientation_w = ros_imu.orientation.w;
	  imu.angular_velocity_x = ros_imu.angular_velocity.x;
	  imu.angular_velocity_y = ros_imu.angular_velocity.y;
	  imu.angular_velocity_z = ros_imu.angular_velocity.z;
	  imu.linear_acceleration_x =ros_imu.linear_acceleration.x;
	  imu.linear_acceleration_y =ros_imu.linear_acceleration.y;
	  imu.linear_acceleration_z =ros_imu.linear_acceleration.z;
	  f_imu<<to_string(imu.time)<<" "<<to_string(imu.orientation_x)<<" "<<to_string(imu.orientation_y)<<" "<<to_string(imu.orientation_z)<<" "<<to_string(imu.orientation_w)<<" "<<to_string(imu.angular_velocity_x)<<" "<<to_string(imu.angular_velocity_y)<<" "<<to_string(imu.angular_velocity_z)<<" "<<to_string(imu.linear_acceleration_x)<<" "<<to_string(imu.linear_acceleration_y)<<" "<<to_string(imu.linear_acceleration_z)<<endl;		  
	  f_imu.close();
	}
	void Write_rawimu(string path,sensor_msgs::Imu ros_raw_imu)
	{
	  ofstream f_rawimu;
	  f_rawimu.open(path,ios::app);
	  Imu rawimu;
          rawimu.time = ros_raw_imu.header.stamp.toSec();
	  rawimu.orientation_x = ros_raw_imu.orientation.x;
	  rawimu.orientation_y = ros_raw_imu.orientation.y;
	  rawimu.orientation_z = ros_raw_imu.orientation.z;
	  rawimu.orientation_w = ros_raw_imu.orientation.w;
	  rawimu.angular_velocity_x = ros_raw_imu.angular_velocity.x;
	  rawimu.angular_velocity_y = ros_raw_imu.angular_velocity.y;
	  rawimu.angular_velocity_z = ros_raw_imu.angular_velocity.z;
	  rawimu.linear_acceleration_x = ros_raw_imu.linear_acceleration.x;
	  rawimu.linear_acceleration_y =ros_raw_imu.linear_acceleration.y;
	  rawimu.linear_acceleration_z =ros_raw_imu.linear_acceleration.z;
	  f_rawimu<<to_string(rawimu.time)<<" "<<to_string(rawimu.orientation_x)<<" "<<to_string(rawimu.orientation_y)<<" "<<to_string(rawimu.orientation_z)<<" "<<to_string(rawimu.orientation_w)<<" "<<to_string(rawimu.angular_velocity_x)<<" "<<to_string(rawimu.angular_velocity_y)<<" "<<to_string(rawimu.angular_velocity_z)<<" "<<to_string(rawimu.linear_acceleration_x)<<" "<<to_string(rawimu.linear_acceleration_y)<<" "<<to_string(rawimu.linear_acceleration_z)<<endl;		  
	   f_rawimu.close();
	}
	void Write_gpgga(string path,novatel_gps_msgs::Gpgga ros_gpgga)
	{
	  ofstream f_gpgga;
	  f_gpgga.open(path,ios::app);
	  Gpgga gpgga;
	  gpgga.time = ros_gpgga.header.stamp.toSec();
	  gpgga.utc_seconds=ros_gpgga.utc_seconds;
          gpgga.lat=ros_gpgga.lat;
          gpgga.lon=ros_gpgga.lon;
          gpgga.lat_dir=ros_gpgga.lat_dir;
          gpgga.lon_dir=ros_gpgga.lon_dir;
          gpgga.gps_qual=ros_gpgga.gps_qual;
          gpgga.num_sats=ros_gpgga.num_sats;
          gpgga.hdop=ros_gpgga.hdop;
          gpgga.alt=ros_gpgga.alt;
          gpgga.altitude_units=ros_gpgga.altitude_units;
	  gpgga.undulation=ros_gpgga.undulation;
          gpgga.undulation_units=ros_gpgga.undulation_units;
          gpgga.diff_age=ros_gpgga.diff_age;
          gpgga.station_id=ros_gpgga.station_id;
	 
      //  f_gpgga<<to_string(gpgga.utc_seconds)<<" "<<to_string(gpgga.lat)<<" "<<to_string(gpgga.lon)<<" "<<to_string(gpgga.lat_dir)<<" "<<to_string(gpgga.lon_dir)<<" "<<to_string(gpgga.gps_qual)<<" "<<to_string(gpgga.num_sats)<<" "<<to_string(gpgga.hdop)<<" "<<to_string(gpgga.alt)<<" "<<to_string(gpgga.altitude_units)<<" "<<to_string(gpgga.undulation)<<" "<<to_string(gpgga.undulation_units)<<" "<<to_string(gpgga.diff_age)<<" "<<to_string(gpgga.station_id)<<endl;
	 f_gpgga<<to_string(gpgga.time)<<" "<<to_string(gpgga.utc_seconds)<<" "<<to_string(gpgga.lat)<<" "<<to_string(gpgga.lon)<<" "<<gpgga.lat_dir<<" "<<gpgga.lon_dir<<" "<<to_string(gpgga.gps_qual)<<" "<<to_string(gpgga.num_sats)<<" "<<to_string(gpgga.hdop)<<" "<<to_string(gpgga.alt)<<" "<<gpgga.altitude_units<<" "<<to_string(gpgga.undulation)<<" "<<gpgga.undulation_units<<" "<<to_string(gpgga.diff_age)<<" "<<gpgga.station_id<<endl;

	 // cout<<gpgga.utc_seconds<<" "<<gpgga.lat<<" "<<gpgga.lon<<" "<<gpgga.lat_dir<<" "<<gpgga.lon_dir<<" "<<gpgga.gps_qual<<" "<<gpgga.num_sats<<" "<<gpgga.hdop<<" "<<gpgga.alt<<" "<<gpgga.altitude_units<<" "<<gpgga.undulation<<" "<<gpgga.undulation_units<<" "<<gpgga.diff_age<<" "<<gpgga.station_id<<" "<<endl;
	 
	  f_gpgga.close();
	  
	} 
	void Write_names(string path,double lidar_time,double img_time)
	{
	  ofstream f;
	  f.open(path,ios::app);
	  f<<to_string(lidar_time)<<" "<<to_string(img_time)<<" "<<to_string((abs(img_time-lidar_time))*1000)<<endl;
	  f.close();
	}
      
	void Write_names(string path,double lidar_time,double img_time,double gnssimu_time)
	{
	  ofstream f;
	  f.open(path,ios::app);
	  f<<to_string(lidar_time)<<" "<<to_string(img_time)<<" "<<to_string(gnssimu_time)<<" "<<to_string((abs(img_time-lidar_time))*1000)<<" "<<to_string((abs(gnssimu_time-lidar_time))*1000)<<endl;
	  f.close();
	}
	
	vector<string> read_format(string content,string s)
	{
	  int l=0;
	  vector<string> result;
	  while(content.find(s)!=std::string::npos)
	  {
	    int locate = content.find(s);
	    string s_num= content.substr(0,locate);
	    l = content.length();
	    content = content.substr(locate+1,l-locate-1);
	    result.push_back(s_num);
	  }
	  result.push_back(content);
	  return result;
	}
	
        template <class Type>  
	Type stringToNum(const string& str)  
	{  
	    istringstream iss(str);  
	    Type num;  
	    iss >> num;  
	    return num;      
	} 
	
	
	void read_lidartocamera(string path,Eigen::Vector3f& t_lc, Eigen::Matrix3f& R_lc)
	{
		cout<<"激光到相机的外参数 = "<<endl;
		ifstream f;
		f.open(path.c_str());
		char buffer[500];
		for(int i=0;i<4;i++)
		{
			f.getline(buffer,500);
			string content = buffer;
			vector<string> messages = read_format(content," ");
			if(i<3)
			{
				R_lc(i,0) = std::stof(messages[0]);
				R_lc(i,1) = std::stof(messages[1].c_str());
				R_lc(i,2) = std::stof(messages[2].c_str());
				cout<<R_lc(i,0)<<" "<<R_lc(i,1)<<" "<<R_lc(i,2)<<endl;
			}
			else
			{
				t_lc(0) = std::stof(messages[0]);
				t_lc(1) = std::stof(messages[1]);
				t_lc(2) = std::stof(messages[2]);
				cout<<t_lc(0)<<" "<<t_lc(1)<<" "<<t_lc(2)<<endl;
			}
		}
	}
	
	
	//定位txt文件某一行
	ifstream & seek_to_line(ifstream & in,int w)
	{
	  int i;
	  char buffer[2000];
	  in.seekg(0,ios::beg);
	  for(i=0;i<w;i++)
	  {
	    in.getline(buffer,sizeof(buffer));
	  }
	  return in;
	}
	

	//用于读取如下类型文件：
	//name img size
	//752 480
	//存储结果为key = name img size. value ={752,480}
	std::map<string,vector<float>> read_confs(string path)
	{
	      map<string,vector<float>> res;
	      ifstream f;
	      f.open(path.c_str());
	      char buffer[2000];
	      string name;
	      vector<float> values;
	      while(!f.eof())
	      {
		f.getline(buffer,2000);
		string content = buffer;
		vector<string> messages = read_format(content," ");
		if(messages[0]=="name")
		{
		  if(values.size()!=0)
		  {
		    res.insert(pair<string,vector<float>>(name,values));
		    values.clear();
		  }
		  name = content;
		}
		else
		{
		  for(int i=0;i<messages.size();i++)
		  {
		    values.push_back(std::stof(messages[i]));
		  }
		}
	      }
	      res.insert(pair<string,vector<float>>(name,values));
	      return res;
	}
	
	//读取双目相机的标定参数
	CAMERAINFO read_cameraInfo(string path)
	{
	  cout<<path<<endl;
	  CAMERAINFO camerainfo;
	  std::map<string,vector<float>> configs = read_confs(path);
	  map<string, vector<float>>::iterator iter;
	  vector<float> res;
	  
	  iter = configs.find("name img size");
	  if(iter!=configs.end())
	  {
	    res = iter->second;
	    camerainfo.Imgsize = cv::Size(int(res[0]),int(res[1]));
	    cout<<iter->first<<endl<<camerainfo.Imgsize.width<<" "<<camerainfo.Imgsize.height<<endl;
	  }
	  
	  iter = configs.find("name left camera K");
	  res = iter->second;
	  camerainfo.K_l = Vector_to_Matrix3f(res);
	  cout<<iter->first<<endl<<camerainfo.K_l<<endl;

	  
	  iter = configs.find("name left camera distorion");
	  res = iter->second;
	  camerainfo.Dis_l = Vector_to_Matrix1_5f(res);
	  cout<<iter->first<<endl<<camerainfo.Dis_l<<endl;

	  
	  iter = configs.find("name left camera recitification");
	  res = iter->second;
	  camerainfo.Rcam_l = Vector_to_Matrix3f(res);
	  cout<<iter->first<<endl<<camerainfo.Rcam_l<<endl;

	  
	  iter = configs.find("name left camera P");
	  res = iter->second;
	  camerainfo.P_l = Vector_to_Matrix3_4f(res);
	  cout<<iter->first<<endl<<camerainfo.P_l<<endl;

	  
	  
	  iter = configs.find("name right camera K");
	  res = iter->second;
	  camerainfo.K_r = Vector_to_Matrix3f(res);
	  cout<<iter->first<<endl<<camerainfo.K_r<<endl;

	  
	  iter = configs.find("name right camera distortion");
	  res = iter->second;
	  camerainfo.Dis_r = Vector_to_Matrix1_5f(res);
	 cout<<iter->first<<endl<<camerainfo.Dis_r<<endl;

	  
	  iter = configs.find("name right camera rectification");
	  res = iter->second;
	  camerainfo.Rcam_r = Vector_to_Matrix3f(res);
	  cout<<iter->first<<endl<<camerainfo.Rcam_r<<endl;

	  
	  iter = configs.find("name right camera P");
	  res = iter->second;
	  camerainfo.P_r = Vector_to_Matrix3_4f(res);
	  cout<<iter->first<<endl<<camerainfo.P_r<<endl;

	  
	  return camerainfo;
	}
	
	pcl::PointCloud<pcl::PointXYZI> Read_KITTI_Bin(string path)//从kitti的 代码中拷贝下来的
	{
	   fstream input(path.c_str(), ios::in | ios::binary);
	   input.seekg(0, ios::beg);//调到文件最起始的位置
	   pcl::PointCloud<pcl::PointXYZI> points;
           int i;
	   for (i=0; input.good() && !input.eof(); i++)
	   {
		  pcl::PointXYZI point;
		  input.read((char *) &point.x, 3*sizeof(float));
		  input.read((char *) &point.intensity, sizeof(float));
		  points.push_back(point);
	    }
	    input.close();
	    return points;
	}
	  
	//下面是读取图像
	sensor_msgs::Image read_Img(string path,double stamp )
	{
	  cout<<path.c_str()<<endl;
	  sensor_msgs::Image res;
	  cv::Mat cvimg = cv::imread(path,cv::IMREAD_UNCHANGED);
	  cv_bridge::CvImage cvi;
	  ros::Time img_stamp(stamp);
	  cvi.header.stamp = img_stamp;
	  cvi.header.frame_id = "camera";
	  cvi.encoding = "mono8";
	  cvi.image = cvimg.clone();
	  cvi.toImageMsg(res);
	  return res;
	}
	
	//读取激光
	sensor_msgs::PointCloud2 read_Lidar(string path,double stamp)
	{
	    sensor_msgs::PointCloud2 ros_cloud;
	    pcl::PointCloud<pcl::PointXYZI> res_pcl;
	    ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
// 	    for(int i=0;i<11;i++)//skip前面几个信息有关的行
// 	    {
// 	      f.getline(buffer,2000);
// 	    }
	    cout<<path.c_str()<<endl;
	    while(!f.eof())
	    {
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      //cout<<"1"<<endl;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  pcl::PointXYZI temp;
			  temp.x = stringToNum<float>(messages[0]);
			  temp.y = stringToNum<float>(messages[1]);
			  temp.z = stringToNum<float>(messages[2]);
			  //temp.intensity = stringToNum<float>(messages[3]);
			  //cout<<temp.x<<endl;
			  if(!((temp.x==0)&&(temp.y==0)&&(temp.z==0)))
			  res_pcl.points.push_back(temp);
		      }
	    }
	    f.close();
	    
	    pcl::toROSMsg(res_pcl,ros_cloud);
	    ros_cloud.header.frame_id ="velodyne";
	    ros::Time lidar_stamp(stamp);
	    ros_cloud.header.stamp = lidar_stamp;
	    return ros_cloud;
	}
	   sensor_msgs::Imu  read_RawImu(string path,double stamp,int l)
	{
	  sensor_msgs::Imu ros_raw_imu;
	   sensor_msgs::Imu rawimu;
	   ifstream f;
	   f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,l);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");		 
          rawimu.orientation.x = stringToNum<double>(messages[1]);
	  rawimu.orientation.y = stringToNum<double>(messages[2]);
	  rawimu.orientation.z = stringToNum<double>(messages[3]);
	  rawimu.orientation.w = stringToNum<double>(messages[4]);
	  rawimu.angular_velocity.x = stringToNum<double>(messages[5]);
	  rawimu.angular_velocity.y = stringToNum<double>(messages[6]);
	  rawimu.angular_velocity.z = stringToNum<double>(messages[7]);
	  rawimu.linear_acceleration.x = stringToNum<double>(messages[8]);
	  rawimu.linear_acceleration.y = stringToNum<double>(messages[9]);
	  rawimu.linear_acceleration.z = stringToNum<double>(messages[10]);
		      }	  
           f.close();
	   ros_raw_imu = rawimu;
	   ros_raw_imu.header.frame_id ="imu";
	   ros::Time rawimu_stamp(stamp);
	   ros_raw_imu.header.stamp = rawimu_stamp;
	  return ros_raw_imu;	  
	}   
     	 sensor_msgs::Imu read_Imu(string path,double stamp,int s)
	{
	  sensor_msgs::Imu ros_imu;
	  sensor_msgs::Imu imu;
	   ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,s);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");		  
	  imu.orientation.x = stringToNum<double>(messages[1]);
	  imu.orientation.y = stringToNum<double>(messages[2]);
	  imu.orientation.z = stringToNum<double>(messages[3]);
	  imu.orientation.w = stringToNum<double>(messages[4]);
	  imu.angular_velocity.x = stringToNum<double>(messages[5]);
	  imu.angular_velocity.y = stringToNum<double>(messages[6]);
	  imu.angular_velocity.z = stringToNum<double>(messages[7]);
	  imu.linear_acceleration.x = stringToNum<double>(messages[8]);
	  imu.linear_acceleration.y = stringToNum<double>(messages[9]);
	  imu.linear_acceleration.z = stringToNum<double>(messages[10]);
	    }
	   f.close();
	   ros_imu=imu;
	  ros_imu.header.frame_id ="camera_imu_frame";
	   ros::Time imu_stamp(stamp);
	   ros_imu.header.stamp = imu_stamp;
	  return ros_imu;
	}   
	
	novatel_gps_msgs::Inspva read_Inspva(string path,double stamp,int m)
	{  novatel_gps_msgs::Inspva ros_inspva;
	  novatel_gps_msgs::Inspva inspva;
	  ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,m);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			  
	  inspva.week = stringToNum<int>(messages[1]);
	  inspva.seconds = stringToNum<double>(messages[2]);
          inspva.latitude = stringToNum<double>(messages[3]);
          inspva.longitude = stringToNum<double>(messages[4]);
	  inspva.height= stringToNum<double>(messages[5]);
	  inspva.north_velocity = stringToNum<double>(messages[6]);
	  inspva.east_velocity= stringToNum<double>(messages[7]);
	  inspva.up_velocity= stringToNum<double>(messages[8]);
	  inspva.roll= stringToNum<double>(messages[9]);
	  inspva.pitch= stringToNum<double>(messages[10]);
	  inspva.azimuth= stringToNum<double>(messages[11]);
	  inspva.status= messages[12];
		      }
	    
	     f.close();
	   ros_inspva = inspva;
	  ros_inspva.header.frame_id ="inspva";
	   ros::Time inspva_stamp(stamp);
	   ros_inspva.header.stamp = inspva_stamp;
	  return ros_inspva;
	  }   
			  
         novatel_gps_msgs::Gpgga read_Gpgga(string path,double stamp,int w)
	{ 
	 novatel_gps_msgs::Gpgga gpgga;
	 novatel_gps_msgs::Gpgga rosData_gpgga;
	 
	    ifstream f;
	    f.open(path.c_str());
	    char buffer[2000];
	    seek_to_line(f,w);
		      f.getline(buffer,2000);
		      string content;
		      content = buffer;
		      if(content.length()!=0)
		      {
			  vector<string> messages = read_format(content," ");
			 	  
	  gpgga.utc_seconds = stringToNum<double>(messages[1]);
	  gpgga.lat = stringToNum<int>(messages[2]);
	  gpgga.lon = stringToNum<double>(messages[3]);
          gpgga.lat_dir = messages[4];
          gpgga.lon_dir = messages[5];
	  gpgga.gps_qual= stringToNum<int>(messages[6]);
	  gpgga.num_sats = stringToNum<int>(messages[7]);
	  gpgga.hdop = stringToNum<float>(messages[8]);
	  gpgga.alt = stringToNum<float>(messages[9]);
	  gpgga.altitude_units = stringToNum<double>(messages[10]);
	  gpgga.undulation = stringToNum<float>(messages[11]);
	  gpgga.undulation_units = messages[12];
	  gpgga.diff_age = stringToNum<int>(messages[13]);
	  gpgga.station_id = messages[14];
	  cout<<to_string(gpgga.utc_seconds)<<" "<<to_string(gpgga.lat)<<" "<<to_string(gpgga.lon)<<" "<<gpgga.lat_dir<<" "<<gpgga.lon_dir<<" "<<to_string(gpgga.gps_qual)<<" "<<to_string(gpgga.num_sats)<<" "<<to_string(gpgga.hdop)<<" "<<to_string(gpgga.alt)<<" "<<gpgga.altitude_units<<" "<<to_string(gpgga.undulation)<<" "<<gpgga.undulation_units<<" "<<to_string(gpgga.diff_age)<<" "<<gpgga.station_id<<endl;
      
		      }
	    
	     f.close();
	     rosData_gpgga =gpgga;
	     rosData_gpgga.header.frame_id ="map";
	     rosData_gpgga.message_id ="GPGGA";
	     ros::Time gpgga_stamp(stamp);
	     rosData_gpgga.header.stamp = gpgga_stamp;	  
	  return rosData_gpgga;
	  }  
	  
	   void GetImudata(string path,string root_path,sensor_msgs::Imu rosData_imu)
	 {
            ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期 
	  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu",1);
	     ros::Rate loop_rate(200);
	  vector<string> stamp6;
	  vector<double> r2;
	  int s=0;
	  char buffer[2000];
	  ifstream f;
	 f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp6.push_back(messages[0]);
		     r2.push_back(stringToNum<double>(messages[0]));
		  }
	  }
		   f.close();
	  while(ros::ok())
	{
	  if(s<r2.size())
	  {
	    rosData_imu = read_Imu(path,r2[s],s);
	    imu_pub.publish(rosData_imu);
	    s++;
	     ros::spinOnce();
	 loop_rate.sleep();
	  }  
	 }   
	 }
	  
	  void GetInspvadata(string path,string root_path,novatel_gps_msgs::Inspva rosData_inspva)
	 {
            ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期 
	  ros::Publisher inspva_pub = n.advertise<novatel_gps_msgs::Inspva>("/inspva",1);
	     ros::Rate loop_rate(10);
	  vector<string> stamp5;
	  vector<double> r1;
	  int m=0;
	  char buffer[2000];
	  ifstream f;
	 f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp5.push_back(messages[0]);
		     r1.push_back(stringToNum<double>(messages[0]));
		  }
	  }
		   f.close();
	  while(ros::ok())
	{
	  if(m<r1.size())
	  {
	    rosData_inspva = read_Inspva(path,r1[m],m);
	    inspva_pub.publish(rosData_inspva);
	    m++;
	     ros::spinOnce();
	 loop_rate.sleep();
	  }  
	 }   
	 }
	 	  
	  
	   void GetRawImudata(string path,string root_path,sensor_msgs::Imu rosData_rawimu)
	 {
            ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期 
	  ros::Publisher rawimu_pub = n.advertise<sensor_msgs::Imu>("/rawimu",1);
	     ros::Rate loop_rate(100);
	  vector<string> stamp;
	  vector<double> r0;
	  int l=0;
	  char buffer[2000];
	  ifstream f;
	 f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp.push_back(messages[0]);
		     r0.push_back(stringToNum<double>(messages[0]));
		  }
	  }
		   f.close();
	  while(ros::ok())
	{
	  if(l<r0.size())
	  {
	    rosData_rawimu = read_RawImu(path,r0[l],l);
	    rawimu_pub.publish(rosData_rawimu);
	    l++;
	     ros::spinOnce();
	 loop_rate.sleep();
	  }  
	 }
	  
	}
	  void GetGnssLidarCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    sensor_msgs::PointCloud2 rosData_lidar, 
				    novatel_gps_msgs::Gpgga rosData_gpgga)
{			   
	  
	 
	   ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期
          cout<<"项目目录 = "<<root_path<<endl;
	 ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,gpgga_pub,rawimu_pub;
         vector<string> message,stamp0,stamp1,stamp2;
	 vector<double> s0,s1,s2,s3;
	  lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
	  leftimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/left/image_raw", 1);
	  rightimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/right/image_raw", 1);
	  gpgga_pub = n.advertise<novatel_gps_msgs::Gpgga>("/gpgga",1);
	   ros::Rate loop_rate(10);
	  int i=0,j=0,k=0,w=0;
	    char buffer[2000];
	  ifstream f;
		f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp0.push_back(messages[0]);
		     s0.push_back(stringToNum<double>(messages[0]));
		    stamp1.push_back(messages[1]);
		    s1.push_back(stringToNum<double>(messages[1]));	
		    stamp2.push_back(messages[2]);	
		    s2.push_back(stringToNum<double>(messages[2]));
		  }		  
	  }
         f.close();
	 s3.insert(s3.end(),s0.begin(),s0.end());
	 s3.insert(s3.end(),s1.begin(),s1.end());
	 s3.insert(s3.end(),s2.begin(),s2.end());
	sort(s3.begin(),s3.end());
	sort(s2.begin(),s2.end()); 
	
   while(ros::ok())
	{
	 if(i<s3.size())
	 {
	   if(s0[j]==s3[i])
	   {
	   rosData_lidar = read_Lidar(root_path+"data/lidar/"+stamp0[j]+".txt",s0[j]);
	   lidar_pub.publish(rosData_lidar);
	   j++;
	    ros::spinOnce();
	 loop_rate.sleep();
	   }
	 else if(s1[k]==s3[i])
	   {
	     rosData_leftimg = read_Img(root_path +"data/leftImg/"+"left_"+stamp1[k]+".png",s1[k]);
	     rosData_rightimg = read_Img(root_path+"data/rightImg/"+"right_"+stamp1[k]+".png",s1[k]);
	     leftimg_pub.publish(rosData_leftimg);
	     rightimg_pub.publish(rosData_rightimg);
	     k++;
	      ros::spinOnce();
	 loop_rate.sleep();
	   }
	  else if(s2[w]==s3[i])
	  {
	    rosData_gpgga = read_Gpgga(root_path+"data/gnssimu/"+"gpgga.txt",s2[w],w);
	    gpgga_pub.publish(rosData_gpgga);
	    w++;
	     ros::spinOnce();
	 loop_rate.sleep();
	  }  
	i++;
	 }
	  }
   }
         
   
   	  void GetLidarCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    sensor_msgs::PointCloud2 rosData_lidar)
{			   
	  
	 
	   ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期
          cout<<"项目目录 = "<<root_path<<endl;
	 ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,gpgga_pub;
         vector<string> message,stamp0,stamp1,stamp2;
	 vector<double> s0,s1,s2,s3;
	  lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
	  leftimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/left/image_raw", 1);
	  rightimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/right/image_raw", 1);
		       ros::Rate loop_rate(10);
	  int i=0,j=0,k=0,w=0;
	    char buffer[2000];
	  ifstream f;
		f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp0.push_back(messages[0]);
		     s0.push_back(stringToNum<double>(messages[0]));
		    stamp1.push_back(messages[1]);
		    s1.push_back(stringToNum<double>(messages[1]));	
		    stamp2.push_back(messages[2]);	
		    s2.push_back(stringToNum<double>(messages[2]));
		  }		  
	  }
         f.close();
	 s3.insert(s3.end(),s0.begin(),s0.end());
	 s3.insert(s3.end(),s1.begin(),s1.end());
	// s3.insert(s3.end(),s2.begin(),s2.end());
	sort(s3.begin(),s3.end());
	
   while(ros::ok())
	{
	 if(i<s3.size())
	 {
	   if(s0[j]==s3[i])
	   {
	   rosData_lidar = read_Lidar(root_path+"data/lidar/"+stamp0[j]+".txt",s0[j]);
	   lidar_pub.publish(rosData_lidar);	  
	     j++;
	    ros::spinOnce();
	    loop_rate.sleep();
	   }
	 else if(s1[k]==s3[i])
	   {
	     rosData_leftimg = read_Img(root_path +"data/leftImg/"+"left_"+stamp1[k]+".png",s1[k]);
	     rosData_rightimg = read_Img(root_path+"data/rightImg/"+"right_"+stamp1[k]+".png",s1[k]);
	     leftimg_pub.publish(rosData_leftimg);
	     rightimg_pub.publish(rosData_rightimg);    
	      k++;
	    ros::spinOnce();
	    loop_rate.sleep();
	   }
	i++;
	 }
	  }
   }
   
   void GetGnssCameredata(string path, string root_path, 
				    sensor_msgs::Image rosData_leftimg, 
				    sensor_msgs::Image rosData_rightimg, 
				    novatel_gps_msgs::Gpgga rosData_gpgga)
{			   
	  
	 
	   ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期
          cout<<"项目目录 = "<<root_path<<endl;
	 ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,gpgga_pub;
         vector<string> message,stamp0,stamp1,stamp2;
	 vector<double> s0,s1,s2,s3;
	  leftimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/left/image_raw", 1);
	  rightimg_pub = n.advertise<sensor_msgs::Image>("/mynteye/right/image_raw", 1);
	  gpgga_pub = n.advertise<novatel_gps_msgs::Gpgga>("/gpgga",1);
	      ros::Rate loop_rate(10);
	  int i=0,j=0,k=0,w=0;
	    char buffer[2000];
	  ifstream f;
		f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp0.push_back(messages[0]);
		     s0.push_back(stringToNum<double>(messages[0]));
		    stamp1.push_back(messages[1]);
		    s1.push_back(stringToNum<double>(messages[1]));	
		    stamp2.push_back(messages[2]);	
		    s2.push_back(stringToNum<double>(messages[2]));
		  }		  
	  }
         f.close();
	 s3.insert(s3.end(),s1.begin(),s1.end());
	 s3.insert(s3.end(),s2.begin(),s2.end());
	sort(s3.begin(),s3.end());
	sort(s2.begin(),s2.end());
	
   while(ros::ok())
	{
	 if(i<s3.size())
	 {  
	 if(s1[k]==s3[i])
	   {
	     rosData_leftimg = read_Img(root_path +"data/leftImg/"+"left_"+stamp1[k]+".png",s1[k]);
	     rosData_rightimg = read_Img(root_path+"data/rightImg/"+"right_"+stamp1[k]+".png",s1[k]);
	     leftimg_pub.publish(rosData_leftimg);
	     rightimg_pub.publish(rosData_rightimg);
	      k++;
	      ros::spinOnce();
	      loop_rate.sleep();
	   }
	  else if(s2[w]==s3[i])
	  {
	    rosData_gpgga = read_Gpgga(root_path+"data/gnssimu/"+"gpgga.txt",s2[w],w);
	    gpgga_pub.publish(rosData_gpgga);
	      w++;
	    ros::spinOnce();
	    loop_rate.sleep();	   
	  }  
	i++;
	 }
	  }
   }
   
   void GetGnssLidardata(string path, string root_path, 
				    sensor_msgs::PointCloud2 rosData_lidar, 
				    novatel_gps_msgs::Gpgga rosData_gpgga)
{			   
	  
	 
	   ros::NodeHandle n;
          n.getParam("/read/root_path", root_path);//加入详细日期
          cout<<"项目目录 = "<<root_path<<endl;
	 ros::Publisher lidar_pub,leftimg_pub,rightimg_pub,gpgga_pub;
         vector<string> message,stamp0,stamp1,stamp2;
	 vector<double> s0,s1,s2,s3;
	  lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
	  gpgga_pub = n.advertise<novatel_gps_msgs::Gpgga>("/gpgga",1);
	   ros::Rate loop_rate(10);
	  int i=0,j=0,k=0,w=0;
	    char buffer[2000];
	  ifstream f;
		f.open(path.c_str());       
	  while((!f.eof()))
	  {
		 f.getline(buffer,2000);
		  string content = buffer;
		  if(content.length()!=0)
		  {
		    vector<string> messages = read_format(content," ");
		     stamp0.push_back(messages[0]);
		     s0.push_back(stringToNum<double>(messages[0]));
		    stamp1.push_back(messages[1]);
		    s1.push_back(stringToNum<double>(messages[1]));	
		    stamp2.push_back(messages[2]);	
		    s2.push_back(stringToNum<double>(messages[2]));
		  }		  
	  }
         f.close();
	 s3.insert(s3.end(),s0.begin(),s0.end());
	 //s3.insert(s3.end(),s1.begin(),s1.end());
	 s3.insert(s3.end(),s2.begin(),s2.end());
	sort(s3.begin(),s3.end());
	sort(s2.begin(),s2.end());
   while(ros::ok())
	{
	 if(i<s3.size())
	 {
	   if(s0[j]==s3[i])
	   {
	   rosData_lidar = read_Lidar(root_path+"data/lidar/"+stamp0[j]+".txt",s0[j]);
	   lidar_pub.publish(rosData_lidar);	    
	      j++;
	    ros::spinOnce();
	    loop_rate.sleep();
	   }
	  else if(s2[w]==s3[i])
	  {
	    rosData_gpgga = read_Gpgga(root_path+"data/gnssimu/"+"gpgga.txt",s2[w],w);
	    gpgga_pub.publish(rosData_gpgga);
	      w++;
	    ros::spinOnce();
	    loop_rate.sleep();	   
	  }  
	i++;
	 }
	  }
   }
	  
}
#endif

