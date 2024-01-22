#ifndef _PCLlib_
#define _PCLlib_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include "FileIO.h"
#include "Tic_Toc.h"
#include "Util.h"
 
using namespace std;
pcl::PointCloud<pcl::PointXYZI> Load_PointCloud(string filenames,int ID,float angle1=90,float angle2=270,float range_=0.6)
{
	vector<vector<float>> cloud_points = FileIO::ReadTxt2Float(filenames);
	pcl::PointCloud<pcl::PointXYZI> cloud;
	for(int j=0;j<cloud_points.size();j++)
	{
		float x = cloud_points[j][0]; 
		float y = cloud_points[j][1];
		float z = cloud_points[j][2];
		float range = sqrt(x * x + y * y+ z * z);
		if(range > range_)
		{
			float horizonAngle = -atan2(y, x)*180/PI+180;//输出范围0-360度
			if((horizonAngle>=angle1)&&(horizonAngle<=angle2))//(horizonAngle>=angle1)&&(horizonAngle<=angle2)
			{
				pcl::PointXYZI temp;
				temp.x = x;
				temp.y = y;
				temp.z = z;
				temp.intensity = ID;//i
				cloud.push_back(temp);
			}
		}
	}
	return cloud;
} 

std::vector<pcl::PointCloud<pcl::PointXYZI>> GetScanPointCloud(vector<vector<float>> cloud_points,int N_SCANS,float angle1=90,float angle2=270)
{
	std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);
	for(int j=0;j<cloud_points.size();j++)
	{
		pcl::PointXYZI temp;
		temp.x = cloud_points[j][0]; 
		temp.y = cloud_points[j][1];
		temp.z = cloud_points[j][2];

		float horizonAngle = -atan2(temp.y, temp.x)*180/PI+180;//输出范围0-360度
		float angle = atan(temp.z / sqrt(temp.x * temp.x + temp.y * temp.y)) * 180 / M_PI;
		if((horizonAngle>=angle1)&&(horizonAngle<=angle2))//(horizonAngle>=angle1)&&(horizonAngle<=angle2)
		{
			int ID=int((angle + 15) / 2 + 0.5);
			if(ID>=N_SCANS)
			    continue;

			temp.intensity =ID+(horizonAngle-angle1)/360;//i
			laserCloudScans[ID].push_back(temp);
		}	
	}

	return laserCloudScans;
} 



namespace PCLlib
{
	float GetDistance(pcl::PointXYZ t,pcl::PointXYZI s)
	{
		return sqrt((t.x-s.x)*(t.x-s.x)+(t.y-s.y)*(t.y-s.y)+(t.z-s.z)*(t.z-s.z));
	}
    
    float GetPointPlaneDis(pcl::PointXYZI t,Eigen::Vector4f p)
	{
		return abs(p[0]*t.x+p[1]*t.y+p[2]*t.z+p[3])/sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
	}

    void SearchMostClosePoint(pcl::PointXYZ &t,std::vector<pcl::PointCloud<pcl::PointXYZI>> &laserCloudScans)
	{
		float temp,r=1;
		pcl::PointXYZI p;
		for(int i=0;i<laserCloudScans.size();i++)
		{
			for(int j=0;j<laserCloudScans[i].points.size();j++)
			{
				temp=GetDistance(t,laserCloudScans[i].points[j]);
				if(temp<r)
				{
					p=laserCloudScans[i].points[j];
					r=temp;
				}
			}
		}
	    cout<<"SearchMostClosePointdis:"<<sqrt((t.x-p.x)*(t.x-p.x)+(t.y-p.y)*(t.y-p.y)+(t.z-p.z)*(t.z-p.z))<<endl;
		t.x=p.x;
		t.y=p.y;
		t.z=p.z;
	}
	int SearchCloseIndex(pcl::PointXYZ &t,pcl::PointCloud<pcl::PointXYZI> &s,float r=0.5)//实测数据r=0.1 仿真数据r=0.5
	{
		int index=0;
		float min_dis=r;
		for(int i=0;i<s.size();i++)
		{
				float temp=GetDistance(t,s[i]);
				if(temp<min_dis)
				{
					min_dis=temp;
					index=i;
				}
		}
		//cout<<"close distance:"<<GetDistance(t,s[index])<<endl;
		return index;
	}

	float GetCurvature(pcl::PointCloud<pcl::PointXYZI> &laserCloud,int index,int npoints)
	{
		if(index<npoints||index>(laserCloud.size()-npoints))
		    return 10;
		float diffX=0,diffY=0,diffZ=0;
		for(int i=index-npoints;i<=index+npoints;i++)
		{
			diffX+=laserCloud[i].x;
			diffY+=laserCloud[i].y;
			diffZ+=laserCloud[i].z;
		}
		diffX=diffX-(2*npoints+1)*laserCloud[index].x;
		diffY=diffY-(2*npoints+1)*laserCloud[index].y;
		diffZ=diffZ-(2*npoints+1)*laserCloud[index].z;
	    return diffX * diffX + diffY * diffY + diffZ * diffZ;
	}

	void DetectLine(pcl::PointCloud<pcl::PointXYZI> &laserCloud,pcl::PointCloud<pcl::PointXYZI> &board_points,int index,int npoints,Eigen::Vector4f p_lidar,int diff_=4)//实测数据diff_=2 仿真数据diff_=4
	{

		for(int i=index-1;i>=npoints;i--)
		{
			float diff=GetCurvature(laserCloud,i,npoints);
			if(diff>diff_)
			{
				    break;
			}
			if(GetPointPlaneDis(laserCloud[i],p_lidar)<0.02)
			{
				laserCloud[i].intensity=16;
				board_points.push_back(laserCloud[i]);
			}
		}
		for(int i=index;i<laserCloud.size()-npoints;i++)
		{
			float diff=GetCurvature(laserCloud,i,npoints);
			if(diff>diff_)
			{
				break;
			}
			if(GetPointPlaneDis(laserCloud[i],p_lidar)<0.02)
			{
				laserCloud[i].intensity=16;
				board_points.push_back(laserCloud[i]);
			}
		}
	}
    
    Eigen::Vector4f Calculate_Planar_Model2(pcl::PointCloud<pcl::PointXYZI> &cloudin,float th = 0.02)
	{
		std::vector<int> inliers;
		pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloudin.makeShared()));
		pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
		ransac.setDistanceThreshold (th);//与平面距离小于th的点做为局内点考虑
		ransac.computeModel();
		ransac.getInliers(inliers);//存储估计得到点局内点
		Eigen::VectorXf planar_coefficients;
		ransac.getModelCoefficients(planar_coefficients);//获取平面参数
		Eigen::Vector4f res(planar_coefficients(0),planar_coefficients(1),planar_coefficients(2),planar_coefficients(3));
		return res;
	}

    Eigen::Vector4f Calculate_Planar_Model(pcl::PointCloud<pcl::PointXYZI> &cloud) 
	{
		vector<vector<double>> mat3x4(3, vector<double>(4, 0));
		for (int i=0;i<cloud.size();i++) 
		{
			float p[3];
			p[0]=cloud[i].x;
			p[1]=cloud[i].y;
			p[2]=cloud[i].z;
			// 3x3 方程组参数   p[0]:x, p[1]:y, p[2]:z
			// 对角线对称，只需处理5个
			mat3x4[0][0] += p[0] * p[0];
			mat3x4[0][1] += p[0] * p[1];
			mat3x4[0][2] += p[0];
			mat3x4[1][1] += p[1] * p[1];
			mat3x4[1][2] += p[1];

			// 3x1 方程组值
			mat3x4[0][3] += p[0] * p[2];
			mat3x4[1][3] += p[1] * p[2];
			mat3x4[2][3] += p[2];
		}

		Eigen::MatrixXd A(3, 3);
		A <<
			mat3x4[0][0], mat3x4[0][1], mat3x4[0][2],
			mat3x4[0][1], mat3x4[1][1], mat3x4[1][2],
			mat3x4[0][2], mat3x4[1][2], cloud.size();
		Eigen::MatrixXd B(3, 1);
		B << mat3x4[0][3], mat3x4[1][3], mat3x4[2][3];

		//以下三个方式前14位有效数字基本一致，任选一个
		//Eigen::MatrixXd a = A.bdcSvd(ComputeThinU | ComputeThinV).solve(B);
		Eigen::MatrixXd a = A.colPivHouseholderQr().solve(B);
		//MatrixXd a = A.lu().solve(B);
        Eigen::Vector4f P;
		P[0]=a(0);
		P[1]=a(1);
		P[2]=-1;
		P[3]=a(2);
		// 若需 A*x + B*y + C*z + D = 0 形式,则A,B,C,D为{ a(0), a(1), -1.0, a(2) }
		 return P;
    }
    
	void DetectPlane(Eigen::Vector4f &plane_model,std::vector<pcl::PointCloud<pcl::PointXYZI>> &laserCloudScans,pcl::PointXYZ target_point,Eigen::Vector3f &center,int N_SCANS=16,int npoints=2)//实测数据npoints=5 仿真数据npoints=2
	{
		//float dis=0.6;
		pcl::PointCloud<pcl::PointXYZI> board_points;
		pcl::PointXYZI t;
		t.x=target_point.x;
		t.y=target_point.y;
		t.z=target_point.z;
		t.intensity=20;
		SearchMostClosePoint(target_point,laserCloudScans);
		laserCloudScans[0].push_back(t);
		float angle = atan(target_point.z / sqrt(target_point.x * target_point.x + target_point.y * target_point.y)) * 180 / M_PI;
		int ID=int((angle + 15) / 2 + 0.5);
		//cout<<"ID:"<<ID<<endl;
        int pindex;
		for(int i=ID;i<laserCloudScans.size();i++)
		{
			pindex=SearchCloseIndex(target_point,laserCloudScans[i]);
			//cout<<"getdis:"<<GetDistance(target_point,laserCloudScans[i].points[pindex])<<endl;
			if(GetDistance(target_point,laserCloudScans[i].points[pindex])>0.6)//0.13实测数据0.13 仿真数据0.6
			    break;
			for(int j=pindex-1;j>=0;j--)
			{
				//cout<<"getdis xiangling:"<<GetDistance(target_point,laserCloudScans[i].points[j])<<endl;
				//cout<<"GetCurvature:"<<GetCurvature(laserCloudScans[i],j,npoints)<<endl;
				if(GetDistance(target_point,laserCloudScans[i].points[j])>0.6)
				    break;
				if(GetCurvature(laserCloudScans[i],j,npoints)>4)////实测数据1.5 仿真数据4
				    break;
				board_points.push_back(laserCloudScans[i].points[j]);
			}
			for(int j=pindex;j<=laserCloudScans[i].points.size();j++)
			{
				if(GetDistance(target_point,laserCloudScans[i].points[j])>0.6)
				    break;
				if(GetCurvature(laserCloudScans[i],j,npoints)>4)
				    break;
				board_points.push_back(laserCloudScans[i].points[j]);
			}
		}
		for(int i=ID-1;i>=0;i--)
		{
			pindex=SearchCloseIndex(target_point,laserCloudScans[i]);
			if(GetDistance(target_point,laserCloudScans[i].points[pindex])>0.6)
			    break;
			for(int j=pindex-1;j>=0;j--)
			{
				if(GetDistance(target_point,laserCloudScans[i].points[j])>0.6)
				    break;
				if(GetCurvature(laserCloudScans[i],j,npoints)>4)
				    break;
				board_points.push_back(laserCloudScans[i].points[j]);
			}
			for(int j=pindex;j<=laserCloudScans[i].points.size();j++)
			{
				if(GetDistance(target_point,laserCloudScans[i].points[j])>0.6)
				    break;
				if(GetCurvature(laserCloudScans[i],j,npoints)>4)
				    break;
				board_points.push_back(laserCloudScans[i].points[j]);
			}
		}

		//cout<<"plane_points:"<<board_points.size()<<endl;
		Eigen::Vector4f p_lidar;
		if(board_points.size()>10)//实测数据20 仿真数据10
		    p_lidar = Calculate_Planar_Model(board_points);
		else
		{
			cout<<"plane_points:"<<board_points.size()<<endl;
			cout<<"can not find enough points to caculate plane1!"<<endl;
			plane_model[0]=10;
			plane_model[0]=0;
			plane_model[0]=0;
			plane_model[0]=1;
			return;
		}
        board_points.clear();
		pindex=SearchCloseIndex(target_point,laserCloudScans[ID]);
		pcl::PointXYZ temp=target_point;
        DetectLine(laserCloudScans[ID],board_points,pindex,npoints,p_lidar);
		for(int i=ID+1;i<laserCloudScans.size();i++)
		{
			pindex=SearchCloseIndex(temp,laserCloudScans[i]);
			if(GetDistance(temp,laserCloudScans[i].points[pindex])>0.6)//0.15实测数据0.15 仿真数据d0.6
			    break;
			temp.x=laserCloudScans[i].points[pindex].x;
			temp.y=laserCloudScans[i].points[pindex].y;
			temp.z=laserCloudScans[i].points[pindex].z;
            DetectLine(laserCloudScans[i],board_points,pindex,npoints,p_lidar);
		}
        temp=target_point;
		for(int i=ID-1;i>=0;i--)
		{
			pindex=SearchCloseIndex(temp,laserCloudScans[i]);
			if(GetDistance(temp,laserCloudScans[i].points[pindex])>0.6)
			    break;
			temp.x=laserCloudScans[i].points[pindex].x;
			temp.y=laserCloudScans[i].points[pindex].y;
			temp.z=laserCloudScans[i].points[pindex].z;
            DetectLine(laserCloudScans[i],board_points,pindex,npoints,p_lidar);
		}
		//cout<<board_points.size()<<endl;
		if(board_points.size()>10)//实测数据20 仿真数据10
		    plane_model=Calculate_Planar_Model2(board_points);
		else
		{
			cout<<"can not find enough points to caculate plane2!"<<endl;
			plane_model[0]=10;
			plane_model[1]=0;
			plane_model[2]=0;
			plane_model[3]=1;
		}
		    
		if((board_points[0].x*plane_model[0]+board_points[0].y*plane_model[1]+board_points[0].z*plane_model[2])>0)
		{
			plane_model=plane_model*-1.0;
		}
		cout<<" lidar plane = "<<plane_model[0]<<" "<<plane_model[1]<<" "<<plane_model[2]<<" "<<plane_model[3]<<endl;
        pcl::PointXYZ pc;
		pc.x=0;
		pc.y=0;
		pc.z=0;
		for(int i=0;i<board_points.size();i++)
		{
			pc.x=pc.x+board_points[i].x;
			pc.y=pc.y+board_points[i].y;
			pc.z=pc.z+board_points[i].z;
		}
		center[0]=pc.x/board_points.size();
		center[1]=pc.y/board_points.size();
		center[2]=pc.z/board_points.size();
		cout<<"center:"<<center<<endl;
	}

	bool CheckBoardPlane(vector<Eigen::Vector4f> &p)
	{
		if(p.size()==3)
		{
			Eigen::Vector4f p1=p[0];
			Eigen::Vector4f p2=p[1];
			Eigen::Vector4f p3=p[2];
			if((abs(p1[0]*p2[0]+p1[1]*p2[1]+p1[2]*p2[2])<0.15)&&(abs(p3[0]*p2[0]+p3[1]*p2[1]+p3[2]*p2[2])<0.15)&&(abs(p1[0]*p3[0]+p1[1]*p3[1]+p1[2]*p3[2])<0.15))
			{
				//cout<<(p1[0]*p2[0]+p1[1]*p2[1]+p1[2]*p2[2])<<" "<<(p3[0]*p2[0]+p3[1]*p2[1]+p3[2]*p2[2])<<" "<<(p1[0]*p3[0]+p1[1]*p3[1]+p1[2]*p3[2])<<endl;
				return true;
			}
		}
		cout<<"this frame is not valid"<<endl<<endl;
		return false;
	}

  class NDT
  {
    private:
      pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
      int max_iter;        // Maximum iterations
      double step_size;   // Step size
      double trans_eps;  // Transformation epsilon
      bool initial_scan_loaded = false;//表示是第一帧数据
      map<string,vector<float>> config;
      Eigen::Matrix4f curr_pose;//当前优化得到的激光位姿
      nav_msgs::Path path_msg;//激光的所有轨迹
      pcl::PointCloud<pcl::PointXYZI> global_map;
    public:
      NDT(string config_file)
      {
	 config = FileIO::read_config(config_file);
	 max_iter = 30;        // Maximum iterations
	 step_size = 0.1;   // Step size
	 trans_eps = 0.01;  // Transformation epsilon 
	 
	 ndt.setTransformationEpsilon(trans_eps);
	 ndt.setStepSize(step_size);
	 ndt.setResolution(config["ndt_res"][0]);
	 ndt.setMaximumIterations(max_iter);
      }
      
      //输入最新的点云然后得到当前帧的位姿
      void AddCurrentCloud(pcl::PointCloud<pcl::PointXYZI>& scan,Eigen::Matrix4f& T_wl)
      {
	  //pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
	  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
	  //1.将第一针的数据加入到地图中
	  if (initial_scan_loaded == false)
	  {
	      curr_pose = Eigen::Matrix4f::Identity();
	  }
	  else
	  {
	      //2.输入原始点云并得到滤波之后的点云filtered_scan_ptr
	      //16线激光如果不使用滤波则处理一帧数据时间大约是3秒，进行滤波后将leaf size=0.5时，处理一帧时间大约是300ms
	      if(config["filter"][0]==1)//表示对输入的数据进行滤波
	      {
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(config["leaf_size"][0], config["leaf_size"][0], config["leaf_size"][0]);
		voxel_grid_filter.setInputCloud(scan.makeShared());
		voxel_grid_filter.filter(*filtered_scan_ptr);
	      }
	      
	      //3.设置ndt算法的参数
	      ndt.setInputSource(filtered_scan_ptr);//设置当前帧的扫描数据
	      
	    
	      //4.使用ndt开始拟合，并得到拟合的位姿
	      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	      Eigen::Matrix4f init_guess = curr_pose;//需要由作者自定义计算得到的激光点云位姿
	      TicToc usedtime;
	      ndt.align(*output_cloud, init_guess);
	      curr_pose = ndt.getFinalTransformation();
	      cout<<"******************************"<<endl;
	      cout<<"  lidar position = "<<curr_pose(0,3)<<" "<<curr_pose(1,3)<<" "<<curr_pose(2,3)<<endl;
	      cout<<"  fitness_score = "<<ndt.getFitnessScore()<<endl;
	      cout<<"  has_converged = "<<ndt.hasConverged()<<endl;
	      cout<<"  final_num_iteration = "<<ndt.getFinalNumIteration()<<endl;
	      cout<<"  transformation_probability = "<<ndt.getTransformationProbability()<<endl;
	      cout<<"  ndt used time(ms) = "<<usedtime.toc()<<endl;
	      
	      //FPFH(global_map.makeShared(),filtered_scan_ptr);
	      //ICP(global_map.makeShared(),filtered_scan_ptr);
	  }
	  
	  T_wl = curr_pose;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());//当前帧变换到世界地图中的坐标
	  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, T_wl);//将当前帧的点云scan_ptr变化到世界地图下得到transformed_scan_ptr
	  global_map += *transformed_scan_ptr;
	  //对地图进行滤波
	  //pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
	  //voxel_grid_filter.setLeafSize(config["leaf_size"][0], config["leaf_size"][0], config["leaf_size"][0]);
	  //voxel_grid_filter.setInputCloud(global_map.makeShared());
	  //voxel_grid_filter.filter(*global_map);
	  if(initial_scan_loaded==false)
	  {
	      ndt.setInputTarget(global_map.makeShared());//设置匹配用的地图数据
	      initial_scan_loaded = true;
	  }
      }
  };
  
  class RegionGrow
  {
    private:
      float search_radius;
      map<string,vector<float>> config;
     public:
      RegionGrow(string config_file)
      {
	 config = FileIO::read_config(config_file);
      }
      
      pcl::PointCloud<pcl::PointXYZI> SegmentCloud(pcl::PointCloud<pcl::PointXYZI>& inputcloud)
      {
	pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);//存储法线的数据结构
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;//用于计算法线的函数
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (inputcloud.makeShared());
	normal_estimator.setRadiusSearch (0.1);//setKSearch
	normal_estimator.compute (*normals);
	float min_curv = 100000;
	float max_curv = -100000;
	vector<float> curvatures;
	for(int i=0;i<normals->points.size();i++)
	{
	  //inputcloud.points[i].intensity = normals->points[i].curvature;
	  curvatures.push_back(normals->points[i].curvature);
	  if(min_curv>normals->points[i].curvature) min_curv=normals->points[i].curvature; 
	  if(max_curv<normals->points[i].curvature) max_curv=normals->points[i].curvature; 
	}
	cout<<"min_curv = "<<min_curv<<" max_curv = "<<max_curv<<endl;
	Util::Analyze_Data(curvatures,10,true);
	/*pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (PointXYZI);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);*/

	pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
	reg.setMinClusterSize (500);
	reg.setMaxClusterSize (inputcloud.points.size());
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (inputcloud.makeShared());
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (config["angle_th"][0]/ 180.0 * M_PI);
	reg.setCurvatureThreshold (config["curv_th"][0]);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);
	
	
	
	pcl::PointCloud<pcl::PointXYZI> segment_cloud;
	for(int i=0;i<inputcloud.points.size();i++)
	{
	  pcl::PointXYZI tmp;
	  tmp.x = inputcloud.points[i].x;
	  tmp.y = inputcloud.points[i].y;
	  tmp.z = inputcloud.points[i].z;
	  tmp.intensity = 0;
	  segment_cloud.points.push_back(tmp);
	}
	for(int i=0;i<clusters.size();i++)
	{
	  for(int j=0;j<clusters[i].indices.size();j++)
	  {
	    int indx = clusters[i].indices[j];
	    segment_cloud.points[indx].intensity = i+1;
	  }
	}
	return segment_cloud;
      }
  };

  
}
#endif
