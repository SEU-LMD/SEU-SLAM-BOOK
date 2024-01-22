#ifndef _SEGMENT_
#define _SEGMENT_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
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



#include "Eigen/Dense"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointField.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include "Tic_Toc.h"
#include "FileIO.h"


#define TRACE
#ifndef TRACE
 #define tcout 0 && cout//或者NULL && cout
#else
 #define tcout cout
#endif

using namespace std;
float PI = 3.1415926;

//这个文件的作用是读取文件到程序中
namespace Segment
{
   struct Voxel
    {
      int indx =-1;//-1表示无效， 这个点在有效点云中的序号
      int dis;//距离的voxel坐标
      float yaw_angle;//yaw角度
      float pitch_angle;
    };
    struct PointInfo
    {
      int indx;
      float vertical_angle;
      float horizon_angle;
      float range;
      float x,y,z;
    };
    struct SymCmp
    {
	  bool operator () (const Eigen::Vector2i& x, const Eigen::Vector2i& y) const
	  {
		  bool res;
		  
		  if(x[1]<y[1])
		    res = true;
		  else if(x[1]>y[1])
		    res = false;
		  else
		  {
		    if(x[0]<y[0])
		      res = true;
		    else
		      res = false;
		  }
		  return res;
	  }
    };
    bool LessSort(Voxel a,Voxel b)//升序排序
    {
      return (a.yaw_angle < b.yaw_angle);
    }
    bool GreatSort(Voxel a,Voxel b)//降序排序
    {
      return (a.yaw_angle > b.yaw_angle);
    }
    
    bool LessSort_Vertical(Voxel a,Voxel b)//升序排序
    {
      return (a.pitch_angle < b.pitch_angle);
    }
    bool GreatSort_Vertical(Voxel a,Voxel b)//降序排序
    {
      return (a.pitch_angle > b.pitch_angle);
    }
    
   class BingChaJi {
    public:
      vector<int> father;
      
      int Father(int i)
      {
	  if(father[i]!=i){
	      return father[i]=Father(father[i]);
	  }
	  return i;
      }
      void merge(int x,int y)
      {
	  int px=Father(x),py=Father(y);
	  father[px]=py;
      }
      map<int,vector<int>> countComponents(int n, vector<vector<int>>& edges) //n表示节点的个数
      {
	  father.resize(n);
	  for(int i=0;i<n;++i){
	      father[i]=i;
	  }
	  for(auto& edge:edges){
	      merge(edge[0],edge[1]);
	  }
	  unordered_set<int> tmp;//hash表
	  for(int i=0;i<n;++i)
	  {
	      if(tmp.count(Father(i))==0){//注意这里要调用Father(i)函数，因为可能i的父亲还没有更新为最靠上的祖先。
		  tmp.insert(father[i]);//这里可以直接取father[i]的值，因为上面已经验证过了，当前father[i]的值一定是最靠上的祖先。
	      }
	  }
	  
	  map<int,vector<int>> res;
	  for(int i=0;i<n;++i)
	  {
	    int f = Father(i);
	    map<int,vector<int>>::iterator iter = res.find(f);
	    if(iter==res.end())
	    {
	      vector<int> t;
	      t.push_back(i);
	      res.insert(pair<int,vector<int>>(f,t));
	    }
	    else
	    {
	      iter->second.push_back(i);
	    }
	  }
	  return res;
      }   
  };
    
    
   class Segment
   {
      private:
	//用于计算曲率和聚类的相邻点范围
	vector<Eigen::Vector2i> neighbours,neighbours_dist;
	set<Eigen::Vector2i,SymCmp> valid_voxel;//所有的有效voxel内容,存储的坐标=（pitch和yaw的index）
	vector<vector<vector<Voxel>>> voxels;//根据voxel坐标得到voxel中的坐标点
	vector<Eigen::Vector2i> point_voxles;//有效点云序号对应的voxel坐标
	vector<Voxel> point_info;//每个点的具体信息
	vector<float> curves;//存储的是每个点的曲率,没有曲率的点的值是-1
	vector<Eigen::Vector3f> allnormal;
	map<int,vector<Eigen::Vector3f>> left_right_candidates;//对于有曲率的点，存储最左段和最右端点的坐标
	set<float> ordered_curves;
	int lableCount;
	int maxyawIndx;//yaw的最大voxel坐标序号
	//调试过程中需要输出的信息
	pcl::PointCloud<pcl::PointXYZI> invalid_curvature_cloud,valid_curvature_cloud,sharpCloud,planarCloud,badplane_cloud,project_cloud;
	pcl::PointCloud<pcl::PointXYZI> test_cloud;
	ros::Publisher pointWithLael_inliner_pub,project_cloud_pub,final_three_plan_pub,segment_pub,badplane_pub,withoutsegment_pub,cloudVoxel,originalCloud_pub,planarCloud_pub,sharpCloud_pub,valid_curvature_cloud_pub,invalid_curvature_cloud_pub,pca_area_pub,last_added_pub;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	set<int> valid_curvature_indx,invalid_curvature_indx,sharp_indx,planar_indx,bad_plane_indx;
	vector<vector<int>> labels;
	vector<Eigen::Vector2i> queueInd;//用于bfs搜索用的
	map<int,vector<int>> each_segment_cloud;//第一个元素是分类的序号，第二个元素是有效点云的序号
	map<string,vector<float>> config;
	float unit_d,unit_Yaw,minYaw,maxYaw;//unit_d 单位是米，unit_pitch单位是rad，unit_yaw单位是rad
	vector<Eigen::Vector2i> neighbour_voxles;//判断上下两个线上的点是否为角点
      public:
	  //配置的参数
	  int vertical_scan,yaw_radius,th_cluster;
	  float th_curvature,thresh_cur,th_area,th_plane,show_bias;//第一个是百分比，第二个是阈值
	  pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud; //输入的点云
	  pcl::PointCloud<pcl::PointXYZI> pointWithLael,pointWithLael_inliner;//已经聚类的点云
	  double timestamp;
	  cv::Mat leftimg,rightimg;
// 	  vector<LidarCamera> datas;//存储的是要保存的聚类后的点云
// 	  vector<LidarCamera> all_of_segment_cloud;
	  
	  Segment():queueInd(200000),voxels(100,vector<vector<Voxel>>(5000)),curves(200000,-1),labels(100,vector<int>(5000,-2))
	  {
	      
	  }
	  
	 
	  
	  Eigen::Vector4f Calculate_Planar_Model(pcl::PointCloud<pcl::PointXYZI> cloudin,pcl::PointCloud<pcl::PointXYZI>& outcloud,float th = 0.03)
	  {
	      std::vector<int> inliers;
	      pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloudin.makeShared()));
	      pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
	      ransac.setDistanceThreshold (th);
	      ransac.computeModel();
	      ransac.getInliers(inliers);
	      Eigen::VectorXf planar_coefficients;
	      ransac.getModelCoefficients(planar_coefficients);
	      //cout<<"平面模型= "<<planar_coefficients[0]<<" "<<planar_coefficients[1]<<" "<<planar_coefficients[2]<<" "<<planar_coefficients[3]<<endl;
	      for(int i=0;i<inliers.size();i++)//得到平面ransac估计之后的内点数据
	      {
		//cout<<inliers[i]<<" "<<cloudin.points.size()<<endl;
		outcloud.push_back(cloudin.points[inliers[i]]);
	      }
	      
	      Eigen::Vector4f res(planar_coefficients(0),planar_coefficients(1),planar_coefficients(2),planar_coefficients(3));
	      return res;
	  }
	         
	  void choose_coordinate(vector<Eigen::Vector3f>& orignal,Eigen::Matrix3f& R,Eigen::Vector3f& t,vector<Eigen::Vector3f>& output)//得到的R和t是将输入的坐标变换到新的坐标系中的变换
	  {
	    Eigen::MatrixXf X(3,orignal.size());
	    Eigen::Vector3f average;
	    Eigen::Vector3f sum(0,0,0);
	    for(int i=0;i<orignal.size();i++)
	    {
	      sum = sum + orignal[i];
	    }
	    average = sum/orignal.size();
	    
	    for(int i=0;i<orignal.size();i++)
	    {
	      Eigen::Vector3f a = orignal[i]-average;
	      X(0,i) = a[0];
	      X(1,i) = a[1];
	      X(2,i) = a[2];
	    }
	  
	    Eigen::MatrixXf XTX = X*X.transpose();
	    Eigen::EigenSolver<Eigen::MatrixXf> es1( XTX );
	    Eigen::MatrixXcf evecs = es1.eigenvectors();
	    Eigen::MatrixXcf evals = es1.eigenvalues();
	    //cout<<"特征值 = "<<endl;
	    //cout<<evals<<endl;
	    //cout<<"特征向量 = "<<endl;
	    //cout<<evecs<<endl;
	  
	    for(int i=0;i<3;i++)//使用特征向量更新旋转矩阵R
	    {
	      R(0,i) = evecs.real()(0,i);
	      R(1,i) = evecs.real()(1,i);
	      R(2,i) = evecs.real()(2,i);
	    }
	    if(R.determinant()<0)
	    {
	      R = R*-1.0;
	    }
	    //cout<<"R = "<<endl;
	    //cout<<"R 行列式 = "<<R.determinant()<<endl;
	    //cout<<R<<endl;
	    //R = Eigen::Matrix3d::Identity();
	    t = -R*average;
	    
	    for(int i=0;i<orignal.size();i++)
	    {
	      Eigen::Vector3f coor_new;
	      coor_new = R*(orignal[i]-average);
	      output.push_back(coor_new);
	    }
	  }
	  
	  vector<Eigen::Vector3f> ConvertToEigen(pcl::PointCloud<pcl::PointXYZI> clouds)
	  {
	    vector<Eigen::Vector3f> res;
	    for(int i=0;i<clouds.points.size();i++)
	    {
	      Eigen::Vector3f temp(clouds.points[i].x,clouds.points[i].y,clouds.points[i].z);
	      res.push_back(temp);
	    }
	    return res;
	  }
	  //纯计算平面
	  Eigen::VectorXf Calculate_Planar_Model_Own(pcl::PointCloud<pcl::PointXYZI> cloudin,float& pca)
	  {
	    Eigen::VectorXf model(4);
	    if(cloudin.points.size()==3)
	    {
	      float x1 = cloudin.points[0].x;float y1 = cloudin.points[0].y;float z1 = cloudin.points[0].z;
	      float x2 = cloudin.points[1].x;float y2 = cloudin.points[1].y;float z2 = cloudin.points[1].z;
	      float x3 = cloudin.points[2].x;float y3 = cloudin.points[2].y;float z3 = cloudin.points[2].z;
	      float A = (y3 - y1)*(z3 - z1) - (z2 -z1)*(y3 - y1);
	      float B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
	      float C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
	      float D =  -(A * x1 + B * y1 + C * z1);
	      float norm = sqrt(A*A+B*B+C*C);
	      model[0]= A/norm;
	      model[1]= B/norm;
	      model[2]= C/norm;
	      model[3]= D/norm;
	    }
	    else if(cloudin.points.size()>3)
	    {
		  //1.第一步首先根据pca，将点变换到pca坐标系下
		  vector<Eigen::Vector3f> points = ConvertToEigen(cloudin);
		  Eigen::Matrix3f Rc;
		  Eigen::Vector3f tc;
		  vector<Eigen::Vector3f> points_pca;
		  choose_coordinate(points,Rc,tc,points_pca);
		  //cout<<"pca坐标转换结束"<<endl;
		  
		  //2.计算T_scale变换，并将点变换到new坐标系下
		  Eigen::Vector3f sum(0,0,0);
		  for(int i=0;i<points_pca.size();i++)
		  {
		    float x = abs(points_pca[i][0]);
		    float y = abs(points_pca[i][1]);
		    float z = abs(points_pca[i][2]);
		    sum = sum + Eigen::Vector3f(x,y,z);
		  }
		  Eigen::Matrix3f T_scale = Eigen::Matrix3f::Identity();
		  T_scale(0,0) = (float)points_pca.size()/sum[0];
		  T_scale(1,1) = (float)points_pca.size()/sum[1];
		  T_scale(2,2) = (float)points_pca.size()/sum[2];
		  vector<Eigen::Vector3f> points_new;
		  for(int i=0;i<points_pca.size();i++)
		  {
		    points_new.push_back(T_scale*points_pca[i]);
		  }
		  //cout<<"new坐标系变换结束"<<endl;
		  
		  //3.得到new坐标系下的平面方程
		  Eigen::MatrixXf X(points_new.size(),4);
		  for(int i=0;i<points_new.size();i++)
		  {
		    X(i,0) = points_new[i][0];
		    X(i,1) = points_new[i][1];
		    X(i,2) = points_new[i][2];
		    X(i,3) = 1;
		  }
		  Eigen::MatrixXf XTX = X.transpose()*X;
		  Eigen::EigenSolver<Eigen::MatrixXf> es1( XTX );
		  Eigen::MatrixXcf evecs = es1.eigenvectors();
		  Eigen::MatrixXcf evals = es1.eigenvalues();
		  float a =  evals.real()(1);
		  float b =  evals.real()(2);
		  float c =  evals.real()(3);
		  float norm_eigenvalue = sqrt(a*a+b*b+c*c);
		  
		  float min_eval = abs( evals.real()(1) );
		  int indx = 0;
		  if(abs(evals.real()(0))>abs( evals.real()(1)) )
		  {
		    min_eval = abs( evals.real()(1) );
		    indx = 1;
		  }
		  if( min_eval>abs( evals.real()(2)) )
		  {
		    min_eval = abs( evals.real()(2) );
		    indx = 2;
		  }
		  if( min_eval>abs( evals.real()(3)) )
		  {
		    min_eval = abs( evals.real()(3) );
		    indx = 3;
		  }
		  pca = min_eval/norm_eigenvalue;//最小特征值占整个的百分比
		  Eigen::Vector4f new_model;
		  new_model[0] = evecs.real()(0,indx);
		  new_model[1] = evecs.real()(1,indx);
		  new_model[2] = evecs.real()(2,indx);
		  new_model[3] = evecs.real()(3,indx);
		  //我们希望特征值只有最小的值接近于0 而其他的都远离0 
		  //cout<<"特征值 = "<<endl<<evals.real()<<endl;
		  // cout<<"new坐标系参数计算结束"<<endl;
		  
		  //4.根据new坐标系下得到的参数和之前的变换矩阵得到最终的平面模型
		  Eigen::Matrix4f T_scale_homo = Eigen::Matrix4f::Identity();
		  Eigen::Matrix4f T_pca_homo = Eigen::Matrix4f::Identity();
		  T_scale_homo.topLeftCorner<3,3>() = T_scale;
		  T_pca_homo.topLeftCorner<3,3>() = Rc;
		  T_pca_homo.topRightCorner<3,1>() = tc;
		  
// 		      cout<<"T_scale_homo = "<<endl<<T_scale_homo<<endl;
// 		      cout<<"T_pca_homo = "<<endl<<T_pca_homo<<endl;
		  
		  Eigen::Vector4f final_result = T_pca_homo.transpose()*T_scale_homo*new_model;
		  float norm = sqrt(final_result[0]*final_result[0] + final_result[1]*final_result[1] + final_result[2]*final_result[2]);
		  model[0] = final_result[0]/norm;
		  model[1] = final_result[1]/norm;
		  model[2] = final_result[2]/norm;
		  model[3] = final_result[3]/norm;
	    }
	    return model;
	  }
	  
	  //根据激光射线的平面投影
	  template<class T>
	  pcl::PointCloud<T> Project_Cloud( pcl::PointCloud<T> cloudin,Eigen::Vector3f t, Eigen::Vector4f planar_coefficients,float th = 0.05)
	  {
	      int num = 0;
	      pcl::PointCloud<T> res;
	      for(int i=0;i<cloudin.points.size();i++)
	      {
		float x,y,z,A,B,C,D;
		x = cloudin.points[i].x;
		y = cloudin.points[i].y;
		z = cloudin.points[i].z;
		A = planar_coefficients[0];
		B = planar_coefficients[1];
		C = planar_coefficients[2];
		D = planar_coefficients[3];
		float k = abs( (-D-A*t[0]-B*t[1]-C*t[2])/(A*(x-t[0])+B*(y-t[1])+C*(z-t[2])) );
		if(!isnan(k))
		{
		    T tempXYZI = cloudin.points[i];
		    tempXYZI.x = k*(x-t[0])+t[0];
		    tempXYZI.y = k*(y-t[1])+t[1];
		    tempXYZI.z = k*(z-t[2])+t[2];
		    float dis = (tempXYZI.x-x)*(tempXYZI.x-x)+(tempXYZI.y-y)*(tempXYZI.y-y)+(tempXYZI.z-z)*(tempXYZI.z-z);
		    if(dis<th)//矫正之后不能相距原来的点太远
		    {
		      res.points.push_back(tempXYZI);
		    }
		    else
		      num++;
		}
		//cout<<"残差= "<<planar_coefficients[0]*tempXYZI.x+planar_coefficients[1]*tempXYZI.y+planar_coefficients[2]*tempXYZI.z+planar_coefficients[3]<<endl;
	      }
	      //cout<<"平面投影后距离相差较远的点个数 = "<<num<<" 总点数 = "<<cloudin.points.size()<<endl;
	      return res;
	  }
	  
	  void initialize(ros::NodeHandle& n,string config_file)//对一些参数初始化
	  {
	      
	      segment_pub = n.advertise<sensor_msgs::PointCloud2>("/segment_cloud",1);//聚类的点云
	      cloudVoxel = n.advertise<sensor_msgs::PointCloud2>("/cloud_voxel",1);//孤立的voxel，也就是说上下左右都没有voxel和其相连
	      originalCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/original_cloud",1);//接受到的原始点云，这里publish的目的是因为分割需要时间，导致显示的聚类点云和原始点云不是一帧数据下的
	      planarCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/plannar_cloud",1);//经过曲率计算得到的平面的点云
	      sharpCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/sharp_cloud",1);//经过曲率计算得到的平面的点云
	      valid_curvature_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/valid_curvature_cloud",1);
	      invalid_curvature_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/invalid_curvature_cloud",1);
	      pca_area_pub = n.advertise<sensor_msgs::PointCloud2>("/pca_area",1);
	      last_added_pub = n.advertise<sensor_msgs::PointCloud2>("/last_added_cloud",1);
	      badplane_pub = n.advertise<sensor_msgs::PointCloud2>("/badplane_cloud",1);
	      project_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/project_cloud",1);
	      final_three_plan_pub = n.advertise<sensor_msgs::PointCloud2>("/final_three_plane",1);//test_cloud_pub
	      pointWithLael_inliner_pub =  n.advertise<sensor_msgs::PointCloud2>("/segment_cloud_inliner",1);
	      
	      config = FileIO::read_config(config_file);
	      
	      minYaw = config["minYaw"][0];
	      maxYaw = config["maxYaw"][0];
	      unit_d = config["unit_D"][0];
	      unit_Yaw = config["unit_Yaw"][0];
	      vertical_scan = config["vertical_scan"][0];
	      yaw_radius = config["yaw_radius"][0];
	      th_cluster = config["th_cluster"][0];
	      th_curvature = config["th_curvature"][0];
	      th_area = config["th_area"][0];
	      th_plane = config["th_plane"][0];
	      show_bias = config["show_bias"][0];
	      
	      
	      for(int j=-1;j<2;j++)//pitch
	      {
		for(int k=-1;k<2;k++)//yaw
		{
		    if( !((j==0)&&(k==0)) )
		    {
		      neighbours.push_back(Eigen::Vector2i(j,k));
		    }
		}
	      }
	      for(int j=-1;j<2;j++)//pitch
	      {
		for(int k=-4;k<5;k++)//yaw
		{
		    if( !((j==0)&&(k==0)) )
		    {
		      neighbours_dist.push_back(Eigen::Vector2i(j,k));
		    }
		}
	      }
	      
	      
	      neighbour_voxles.push_back(Eigen::Vector2i(1,0));
	      neighbour_voxles.push_back(Eigen::Vector2i(1,-1));
	      neighbour_voxles.push_back(Eigen::Vector2i(1,1));
	      
	      neighbour_voxles.push_back(Eigen::Vector2i(-1,0));
	      neighbour_voxles.push_back(Eigen::Vector2i(-1,1));
	      neighbour_voxles.push_back(Eigen::Vector2i(-1,-1));
	      
	      cloud.clear();
	      lableCount =1;
	  }//segment 类初始化结束
	  
	   void ResetParameters()
	  {
	    voxels = vector<vector<vector<Voxel>>>(100,vector<vector<Voxel>>(5000));
	    labels = vector<vector<int>>(100,vector<int>(5000,-2));
	    valid_voxel.clear();
	    original_cloud->clear();
	    cloud.clear();
	    curves = vector<float>(200000,-1);
	    lableCount = 1;
	    
	    valid_curvature_cloud.clear();
	    invalid_curvature_cloud.clear();
	    invalid_curvature_indx.clear();
	    valid_curvature_indx.clear();
	    sharpCloud.clear();
	    planarCloud.clear();
	    
	    point_voxles.clear();
	    point_info.clear();
	    test_cloud.clear();
	    
	    sharp_indx.clear();
	    planar_indx.clear();
	    
	    ordered_curves.clear();
	    queueInd.clear();
	    each_segment_cloud.clear();
	    bad_plane_indx.clear();
	    pointWithLael.clear();
	    badplane_cloud.clear();
	    project_cloud.clear();
	    pointWithLael_inliner.clear();
	    
	    left_right_candidates.clear();
	    
	    
	    allnormal.clear();
	      
	  }
	  
	  pcl::PointCloud<pcl::PointXYZI> Obtain_PCLCloud(set<int> indx)//根据点云的序号得到pcl点云
	  {
	    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	    for(set<int>::iterator iter = indx.begin(); iter!= indx.end();iter++)
	    {
	      pcl::PointXYZI tempXYZI = cloud.points[*iter];
	      tempXYZI.intensity = curves[*iter];
	      pcl_cloud.points.push_back(tempXYZI);
	    }
	    return pcl_cloud;
	  }
	  pcl::PointCloud<pcl::PointXYZI> Obtain_PCLCloud(vector<int> indx)//根据点云的序号得到pcl点云
	  {
	    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	    for(vector<int>::iterator iter = indx.begin(); iter!= indx.end();iter++)
	    {
	      pcl::PointXYZI tempXYZI = cloud.points[*iter];
	      tempXYZI.intensity = curves[*iter];
	      pcl_cloud.points.push_back(tempXYZI);
	    }
	    return pcl_cloud;
	  }
	
	  bool Valid_Voxel(Eigen::Vector2i& v)//判断这个坐标是否有效
	  {
	     
	      if(v[0]>=0 && v[0]<vertical_scan && v[1]>=0 && v[1]<=maxyawIndx)
		return true;
	     
	      if((v[1]>maxyawIndx)&&(maxYaw-minYaw==360))
		 v = v-Eigen::Vector2i(0,maxyawIndx+1);   
	     
	      if((v[1]<0)&&(maxYaw-minYaw==360))
		v = v+Eigen::Vector2i(0,maxyawIndx+1);
	      
	       if(v[0]>=0 && v[0]<vertical_scan && v[1]>=0 && v[1]<=maxyawIndx)
		 return true;
	       else
		 return false;
	  }
	  
	  
	  void helper(vector<int> n,int k,int level, vector<int>& out,vector<vector<int>>& res)
	  {
		  if(out.size()==k)
		  {
			  res.push_back(out);
			  return;
		  }
		  for(int i=level;i<n.size();i++)
		  {
			  out.push_back(n[i]);
			  helper(n,k,i+1,out,res);
			  out.pop_back();//删除最后一个元素
		  }
	  }
	  vector<vector<int>> combine(vector<int> n,int k)
	  {
		  vector<vector<int>> res;
		  vector<int> out;
		  helper(n,k,0,out,res);
		  return res;
	  }
	  
	  //m既是输入也是输出
	  map<int,pcl::PointCloud<pcl::PointXYZI>> Fusion_Plane(map<int,Eigen::Vector4f>& m,map<int,pcl::PointCloud<pcl::PointXYZI>> cloud_plane)
	  {
	      //首先
	      pointWithLael.clear();
	      
	      vector<pair<int,Eigen::Vector4f>> m_vect;
	      for(map<int,Eigen::Vector4f>::iterator iter = m.begin();iter!=m.end();iter++)
	      {
		  int label = iter->first;
		  Eigen::Vector4f model = iter->second;
		  pcl::PointCloud<pcl::PointXYZI> cloud_temp = cloud_plane.find(label)->second;
		  Eigen::Vector4f centroid;
		  pcl::compute3DCentroid(cloud_temp, centroid);
		  pcl::PointXYZI temppoint;
		  temppoint.x = centroid[0];
		  temppoint.y = centroid[1];
		  temppoint.z = centroid[2];
		  pcl::flipNormalTowardsViewpoint(temppoint,0,0,0,model);//使法向量的方向朝向viewpoint。
		  iter->second = model;
		  m_vect.push_back(pair<int,Eigen::Vector4f>(label,model));		
	      }
	      
	      
	      int num = m_vect.size();
	      vector<int> data;
	      for(int i=0;i<num;i++)
	      {
		data.push_back(i);
	      }
	      vector<vector<int>> combination =  combine(data,2);//得到所有的两辆排列组合结果
	      vector<vector<int>> fusion;//相当于连通关系
	      for(int i=0;i<combination.size();i++)
	      {
		  int a = combination[i][0];
		  int b = combination[i][1];	
		
		  Eigen::Vector4f modela = m_vect[a].second;
		  Eigen::Vector4f modelb = m_vect[b].second;
		  if(abs(modela[3]-modelb[3])<0.15)
		  {
		      Eigen::Vector3f va(modela[0],modela[1],modela[2]);
		      Eigen::Vector3f vb(modelb[0],modelb[1],modelb[2]);
		      float angle = acos( (modela[0]*modelb[0]+ modela[1]*modelb[1]+ modela[2]*modelb[2])/(va.norm()*vb.norm()) )*180/PI;
		      if((angle<30)&&(angle>0))
		      {
			vector<int> t;
			t.push_back(a);
			t.push_back(b);
			fusion.push_back(t);
		      }
		  }
	      }
	      
	      BingChaJi bcj;
	      map<int,vector<int>> fusion_plane_indx = bcj.countComponents(num,fusion);//非常重要的函数！！！！！！！！！！1
	      map<int,pcl::PointCloud<pcl::PointXYZI>> planar_clouds;
	      map<int,Eigen::Vector4f> model_fusion;
	      map<int,pcl::PointCloud<pcl::PointXYZI>> cloud_plane_fusion;
	      for(map<int,vector<int>>::iterator iter = fusion_plane_indx.begin();iter!=fusion_plane_indx.end();iter++)
	      {
		 
		  pcl::PointCloud<pcl::PointXYZI> temp_cloud;//属于相同平面的所有点云
		  for(int i=0;i<iter->second.size();i++)
		  {
		    int label_fusion = m_vect[iter->second[i]].first;
		    map<int,pcl::PointCloud<pcl::PointXYZI>>::iterator iter_cloud = cloud_plane.find(label_fusion);
		    for(int j=0;j<iter_cloud->second.points.size();j++)
		    {
		      temp_cloud.points.push_back(iter_cloud->second.points[j]);
		    }
		  }
		   //然后对平面进行估计
		   int label = m_vect[iter->first].first;
		   pcl::PointCloud<pcl::PointXYZI> inliner_cloud;
		   Eigen::Vector4f model = Calculate_Planar_Model(temp_cloud,inliner_cloud,th_plane);//经过我们自己的验证 pcl得到的inliner数量和我们自己计算得到的相同
		   if(inliner_cloud.points.size()>th_cluster)
		    {
			for(int i=0;i<inliner_cloud.points.size();i++)
			{
			    pcl::PointXYZI temXYZI = inliner_cloud.points[i];
			    temXYZI.intensity = label;
			    pointWithLael_inliner.points.push_back(temXYZI);
			}
			//cout<<"inliner点云大小 = "<<inliner_cloud.points.size()<<endl;
			model_fusion.insert( pair<int,Eigen::Vector4f>( label,model ) );
			pcl::PointCloud<pcl::PointXYZI> plannar_cloud;
			plannar_cloud = Project_Cloud<pcl::PointXYZI>(inliner_cloud,Eigen::Vector3f(0,0,0),model);//将点投影到我们估计得到的平面模型中
			//plannar_cloud = inliner_cloud;
			planar_clouds.insert( pair<int,pcl::PointCloud<pcl::PointXYZI>>( label,plannar_cloud) );
		   }
		   for(int i=0;i<temp_cloud.points.size();i++)
		   {
		      pcl::PointXYZI temXYZI = temp_cloud.points[i];
		      temXYZI.intensity = label;
		      pointWithLael.points.push_back(temXYZI);
		  }
	      }
	      m = model_fusion;
	      return planar_clouds;
	  }
	  
	  
	  void Obtain_UpDownCloud(int i,vector<int>& upper_indx,vector<int>& down_indx,int& min_indx_upper, int& min_indx_down)
	  {
	      Eigen::Vector2i voxel_ind = point_voxles[i];//当前点所在的voxel坐标
	      //将上面1线和下面1线的定点压入到upper_indx,down_indx;
	      for(int j=0;j<neighbour_voxles.size();j++)
	      {
		Eigen::Vector2i nb = voxel_ind+neighbour_voxles[j];
		if(Valid_Voxel(nb))
		{
			vector<Voxel> points = voxels[nb[0]][nb[1]];//这个voxel中的所有点
			if(points.size()!=0)
			{
			      for(int k=0;k<points.size();k++)
			      {
				if(j<3)
				  upper_indx.push_back(points[k].indx);
				else
				  down_indx.push_back(points[k].indx);
			      }
			}
		}
	      }
	      
	      //上下相邻点搜索完毕，现在开始搜索上下最近点序号min_indx_upper和min_indx_down
	      float min_dist_upper = -1,min_dist_down=-1;
	      min_indx_upper = -1;
	      min_indx_down=-1;
	      Eigen::Vector3f current_point(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
	      if(upper_indx.size()!=0)
	      {
		for(int j=0;j<upper_indx.size();j++)
		{
		    Eigen::Vector3f ng(cloud.points[upper_indx[j]].x,cloud.points[upper_indx[j]].y,cloud.points[upper_indx[j]].z);
		    float dist = (ng-current_point).norm();
		    if(dist>min_dist_upper)
		    {
		      min_dist_upper = dist;
		      min_indx_upper = upper_indx[j];
		    }
		}
	      }
	      if(down_indx.size()!=0)
	      {
		for(int j=0;j<down_indx.size();j++)
		{
		    Eigen::Vector3f ng(cloud.points[down_indx[j]].x,cloud.points[down_indx[j]].y,cloud.points[down_indx[j]].z);
		    float dist = (ng-current_point).norm();
		    if(dist>min_dist_down)
		    {
		      min_dist_down = dist;
		      min_indx_down = down_indx[j];
		    }
		}
	      }
	  }
	  
	  float CalculatePCLCurvatures()
	  {
	      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	      kdtree.setInputCloud(cloud.makeShared());
	      
	      for(int i=0;i<cloud.points.size();i++)
	      {
		  Eigen::Vector3f cur_point(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
		  vector<int> upIndx,downIndx;
		  int upminindx,downminindx;
		  Obtain_UpDownCloud(i,upIndx,downIndx,upminindx,downminindx);//非常重要的函数！！！！
		  float disup=-1,disdown=-1;
		  if(upminindx!=-1)
		  {
		    Eigen::Vector3f tmp(cloud.points[upminindx].x,cloud.points[upminindx].y,cloud.points[upminindx].z);
		    disup = (cur_point-tmp).norm();
		  }
		  if(disdown!=-1)
		  {
		    Eigen::Vector3f tmp(cloud.points[downminindx].x,cloud.points[downminindx].y,cloud.points[downminindx].z);
		    disdown = (cur_point-tmp).norm();
		  }
		  
		  
		  if((disup==-1)&&(disdown==-1))//既没有上点也没有下点
		  {
		    invalid_curvature_indx.insert(i);
		    curves[i]  = -1;
		    invalid_curvature_cloud.points.push_back(cloud.points[i]);
		    allnormal.push_back(Eigen::Vector3f(0,0,0));
		  }
		  else
		  {
		    float th = disdown;
		    if(disup>disdown)
		      th = disup;
		    
		    th = 1.2*th;
		    std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
		    std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
		    kdtree.radiusSearch(cloud.points[i],th,pointIdxRadiusSearch, pointRadiusSquaredDistance);
		    float curvature; 
		    Eigen::Vector4f plane_parameters; 
		    pcl::computePointNormal(cloud,pointIdxRadiusSearch,plane_parameters,curvature);
		    
		    curves[i] = curvature;
		    ordered_curves.insert(curves[i]);
		    valid_curvature_indx.insert(i);
		    pcl::PointXYZI tempXYZI = cloud.points[i];
		    tempXYZI.intensity = curves[i];
		    valid_curvature_cloud.points.push_back(tempXYZI);
		    
		    Eigen::Vector3f n(plane_parameters(0),plane_parameters(1),plane_parameters(2));
		    if(n.adjoint()*cur_point<0)
		      allnormal.push_back(n);
		    else
		      allnormal.push_back(-n);
		    
		  }
	      }
	      
	      /*
	      pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> ne;
	      ne.setInputCloud (cloud.makeShared());
	      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	      ne.setSearchMethod (tree);
	      pcl::PointCloud<pcl::PointNormal> cloud_normals;
	      ne.setRadiusSearch (config["diameter"][0]);//半径搜索范围
	      //ne.setKSearch(10);//设置最近邻需要多少点来模拟平面计算法线
	      ne.compute (cloud_normals);
	      int valid_curvature_num = 0,invalid_curvature_num = 0;
	      for(int i=0;i<cloud_normals.points.size();i++)
	      {
		curves[i] = cloud_normals.points[i].curvature;
		if(curves[i]>0)
		{
		  valid_curvature_num++;
		  ordered_curves.insert(curves[i]);
		  valid_curvature_indx.insert(i);
		  pcl::PointXYZI tempXYZI = cloud.points[i];
		  tempXYZI.intensity = curves[i];
		  valid_curvature_cloud.points.push_back(tempXYZI);
		}
		else
		{
		  invalid_curvature_num++;
		  invalid_curvature_indx.insert(i);
		  curves[i]  = -1;
		  invalid_curvature_cloud.points.push_back(cloud.points[i]);
		}
	      }*/
	      
	      float min_curv = *(ordered_curves.begin());
	      float max_curv = *(ordered_curves.rbegin());
	      float classnum = 8;
	      float delta = (max_curv-min_curv)/classnum;
	      vector<float> classth;
	      vector<int> numofclass;
	      float acc = min_curv;
	      numofclass.push_back(0);
	      for(int i=1;i<(int)classnum;i++)
	      {
		acc = acc + delta;
		classth.push_back(acc);
		numofclass.push_back(0);
	      }
	      classth.push_back(max_curv);
	      for(int i=0;i<cloud.points.size();i++)
	      {
		//curves[i] = cloud_normals.points[i].curvature;
		if(curves[i]>0)
		{
		  for(int j=0;j<(int)classnum;j++)
		  {
		    if(curves[i]<classth[j])
		    {
		      numofclass[j] = numofclass[j]+1;
		      break;
		    }
		  }
		}
	      }
	      for(int i=0;i<numofclass.size();i++)
	      {
		tcout<<" 曲率分类"<<i<<" "<<numofclass[i]<<endl;
	      }
	      float res;
	      for(int i=1;i<numofclass.size();i++)
	      {
		if(numofclass[0]*0.5>numofclass[i])
		{
		  res = classth[i];
		  break;
		}
	      }
	      return res;
	  } 
	  
	  //使用PCA计算最小特正值 norm_vector是最小特征值对应的特征向量
	  float Obtain_PCACurve(vector<Eigen::Vector3f>  points,Eigen::Vector3f& norm_vector)
	  {
	      Eigen::MatrixXf X(3,points.size());
	      Eigen::Vector3f average;
	      Eigen::Vector3f sum(0,0,0);
	      for(int i=0;i<points.size();i++)
	      {
		sum = sum + points[i];
	      }
	      average = sum/float(points.size());
	      for(int i=0;i<points.size();i++)
	      {
		Eigen::Vector3f a = points[i]-average;
		X(0,i) = a[0];
		X(1,i) = a[1];
		X(2,i) = a[2];
	      }
	    
	      Eigen::MatrixXf XTX = X*X.transpose();
	      Eigen::EigenSolver<Eigen::MatrixXf> es1( XTX );
	      Eigen::MatrixXcf evecs = es1.eigenvectors();
	      Eigen::MatrixXcf evals = es1.eigenvalues();
	      vector<float> eigenVaules;
	      eigenVaules.push_back(evals.real()(0));
	      eigenVaules.push_back(evals.real()(1));
	      eigenVaules.push_back(evals.real()(2));//经过测试发现输出的特征值并不是按照大小顺序排列的
	      float min_eval = abs( eigenVaules[0] );//获得最小特征值
	      int min_indx = 0;
	      if(abs(eigenVaules[0])>abs(eigenVaules[1]))
	      {
		min_eval = abs(eigenVaules[1]);
		min_indx = 1;
	      }
	      if( min_eval>abs(eigenVaules[2]))
	      {
		min_eval = abs(eigenVaules[2]);
		min_indx = 2;
	      }
	      float norm = sqrt(eigenVaules[0]*eigenVaules[0]+eigenVaules[1]*eigenVaules[1]+eigenVaules[2]+eigenVaules[2]);
	      //float norm = abs(eigenVaules[0])+abs(eigenVaules[1])+abs(eigenVaules[2]);//pcl书211页
	      //cout<<"特征值 = "<<eigenVaules[0]/norm<<" "<<eigenVaules[1]/norm<<" "<<eigenVaules[2]/norm<<endl;
	      float res = (min_eval)/norm;
	      
	      norm_vector(0) = evecs.real()(0,min_indx);
	      norm_vector(1) = evecs.real()(1,min_indx);
	      norm_vector(2) = evecs.real()(2,min_indx);
	      return res;
	  }
	  
	  
	  
	 void CalculateAgngleCurvatures()
	  {
	    int minnum = config["min_yaw_num"][0];//最小条件
	    for(int i=0;i<cloud.points.size();i++)//遍历不同的点
	    {
		  Eigen::Vector2i voxel_ind = point_voxles[i];//当前点所在的voxel坐标
		  Voxel curr_voxel = point_info[i];
		  Eigen::Vector3f curr_point(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
		  //先在当前的这个voxel中搜索
		  vector<Voxel> left_candidates,right_candidates;
		  if(voxels[voxel_ind[0]][voxel_ind[1]].size()>1)
		  {
		    for(int j=0;j<voxels[voxel_ind[0]][voxel_ind[1]].size();j++)
		    {
		      Voxel neigh_voxel = voxels[voxel_ind[0]][voxel_ind[1]][j];
		      if(neigh_voxel.indx!=i)
		      {
			if(neigh_voxel.yaw_angle>curr_voxel.yaw_angle)
			  right_candidates.push_back(neigh_voxel);
			else if(neigh_voxel.yaw_angle<curr_voxel.yaw_angle)
			  left_candidates.push_back(neigh_voxel);
		      }
		    }
		  }
		  //然后遍历左右voxel并更新right_candidates和left_candidates
		  for(int k=1;k<=yaw_radius;k++)
		  {
		       if(right_candidates.size()<minnum)
		       {
			  Eigen::Vector2i na = voxel_ind+Eigen::Vector2i(0,k);
			  if((na[1]>maxyawIndx)&&(maxYaw-minYaw==360))
			    na = na-Eigen::Vector2i(0,maxyawIndx+1);
			  if(na[1]<=maxyawIndx)
			  {
			      vector<Voxel> right_points = voxels[na[0]][na[1]];//这个voxel中的所有点
			      if(right_points.size()!=0)
			      {
				    for(int j=0;j<right_points.size();j++)
				    {
					right_candidates.push_back(right_points[j]);
				    }
			      }
			  }
		       }
		       else
			 break;
		  }
		  for(int k=1;k<=yaw_radius;k++)
		  {
		       if(left_candidates.size()<minnum)
		       {
			  Eigen::Vector2i na = voxel_ind+Eigen::Vector2i(0,-k);
			  if((na[1]<0)&&(maxYaw-minYaw==360))
			    na = na+Eigen::Vector2i(0,maxyawIndx+1);
			  if(na[1]>=0)
			  {
			      vector<Voxel> left_points = voxels[na[0]][na[1]];//这个voxel中的所有点
			      if(left_points.size()!=0)
			      {
				    for(int j=0;j<left_points.size();j++)
				    {
					left_candidates.push_back(left_points[j]);
				    }
			      }
			  }
		       }
		       else
			 break;
		  } 
		  
		  
		  //判断当前这个点左右的candidate是否超过
		  if((left_candidates.size()<minnum)||(right_candidates.size()<minnum))
		  {
		      curves[i] = -1;//表示这个点的曲率无效，序号是点在点云中的序号，内容是此点的曲率
		      invalid_curvature_indx.insert(i);
		      continue;
		  }
		  
		  sort(left_candidates.begin(),left_candidates.end(),GreatSort);//根据yaw的数值降序
		  sort(right_candidates.begin(),right_candidates.end(),LessSort);//根据yaw的数值升序
		  
		  
		  //相邻的点距离不能超过0.25米
		  bool disconnect = false;
		  Eigen::Vector3f curr_dist(cloud.points[curr_voxel.indx].x,cloud.points[curr_voxel.indx].y,cloud.points[curr_voxel.indx].z);
		  for(int j=0;j<right_candidates.size();j++)
		  {
		    Eigen::Vector3f next_dist(cloud.points[right_candidates[j].indx].x,cloud.points[right_candidates[j].indx].y,cloud.points[right_candidates[j].indx].z);
		    if((curr_dist-next_dist).norm()>0.25)
		    {
		      disconnect = true;
		      break;
		    }
		    curr_dist = next_dist;
		  }
		  if(!disconnect)
		  {
		    curr_dist = Eigen::Vector3f(cloud.points[curr_voxel.indx].x,cloud.points[curr_voxel.indx].y,cloud.points[curr_voxel.indx].z);
		    for(int j=0;j<left_candidates.size();j++)
		    {
		      Eigen::Vector3f next_dist(cloud.points[left_candidates[j].indx].x,cloud.points[left_candidates[j].indx].y,cloud.points[left_candidates[j].indx].z);
		      if((curr_dist-next_dist).norm()>0.25)
		      {
			disconnect = true;
			break;
		      }
		      curr_dist = next_dist;
		    }
		  }
		  if(disconnect)
		  {
		    curves[i] = -1;//表示这个点的曲率无效，序号是点在点云中的序号，内容是此点的曲率
		    invalid_curvature_indx.insert(i);
		    continue;
		  }
		  


		  //下面我们只选择使用最远的两个点计算曲率
		  int id_left  =left_candidates[minnum-1].indx;
		  int id_right  = right_candidates[minnum-1].indx;
		  Eigen::Vector3f neiga_point(cloud.points[id_left].x,cloud.points[id_left].y,cloud.points[id_left].z);
		  Eigen::Vector3f neigb_point(cloud.points[id_right].x,cloud.points[id_right].y,cloud.points[id_right].z);
		  vector<Eigen::Vector3f> tmp;
		  tmp.push_back(neiga_point);
		  tmp.push_back(neigb_point);

		  left_right_candidates.insert(pair<int,vector<Eigen::Vector3f>>(i,tmp));
		  Eigen::Vector3f vectA = neiga_point - curr_point;
		  Eigen::Vector3f vectB = neigb_point - curr_point;
		  float angle =(vectA[0]*vectB[0]+vectA[1]*vectB[1]+vectA[2]*vectB[2])/(vectA.norm()*vectB.norm());
		  float diff  = 180-acos(angle)*180/PI;
		  if(!isnan(diff))
		  {
		      curves[i] = diff;
		      ordered_curves.insert(curves[i]);//用于求取曲率阈值
		      valid_curvature_indx.insert(i);
		  }
		  else
		  {
		    curves[i] = -1;//表示这个点的曲率无效，序号是点在点云中的序号，内容是此点的曲率
		    invalid_curvature_indx.insert(i);
		  }
	    }
	  }
	  
	 
	 
	  void Project_Pointcloud()
	  {
	      tcout<<"********1.原始点云处理*******"<<endl;
	      TicToc tic_temp;
	      int cloudSize = original_cloud->points.size();
	      int invalid_point_num = 0;
	      vector<PointInfo> points;
	      float min_pitch = 200000,min_yaw = 200000,min_d = 200000;
	      float max_pitch = -200000,max_yaw = -200000,max_d = -200000;
	      for (size_t i = 0; i < cloudSize; ++i)
	      {
		  float x = original_cloud->points[i].x;
		  float y = original_cloud->points[i].y;
		  float z = original_cloud->points[i].z;
		  float range = sqrt(x * x + y * y + z * z);
		  if ((range > config["min_d"][0])&&(range<config["max_d"][0]))
		  {
		      float verticalAngle = atan2(z, sqrt(x * x + y * y))*180/PI;
		      float horizonAngle = -atan2(y, x)*180/PI+180;//输出范围0-360度
		      if((horizonAngle>=minYaw)&&(horizonAngle<=maxYaw))
		      {
			    //original_cloud->points[i].intensity = -20000;//-20000表示还没有分类
			    cloud.points.push_back(original_cloud->points[i]);
			
			    PointInfo info;
			    info.vertical_angle = verticalAngle;
			    info.horizon_angle = horizonAngle;
			    info.range = range;
			    info.indx = i;
			    points.push_back(info);
			    
			    if(verticalAngle<min_pitch)
			      min_pitch = verticalAngle;
			    if(verticalAngle>max_pitch)
			      max_pitch = verticalAngle;
			      
			    if(horizonAngle<min_yaw)
			      min_yaw = horizonAngle;
			    if(horizonAngle>max_yaw)
			      max_yaw = horizonAngle;
			    
			    if(range<min_d)
			      min_d = range;
			    if(range>max_d)
			      max_d = range;			    
		      }
		  }
		  else
		  {
		      invalid_point_num++;
		  }
	      }
	      float unit_pitch = (max_pitch-min_pitch)/(vertical_scan-1);
	      
	      tcout<<"原始点云预处理使用时间(ms) = "<<tic_temp.toc()<<endl;
	      tcout<<"原始点云个数 = "<<cloudSize<<endl;
	      tcout<<"无效点的数量 = "<<invalid_point_num<<" 有效点的数量 = "<<cloud.points.size()<<endl;
	      tcout<<"D 范围 = "<<min_d<<" "<<max_d<<endl;
	      tcout<<"pitch 角度范围(度) = "<<min_pitch<<" "<<max_pitch<<" 分辨率= "<<unit_pitch<<endl;
	      tcout<<"yaw 角度范围(度) = "<<min_yaw<<" "<<max_yaw<<" 分辨率 = "<<unit_Yaw<<endl;
	      
	      
	      
	      maxyawIndx = round( (max_yaw-min_yaw)/unit_Yaw);
	      for (int i = 0; i < points.size(); i++)
	      {
			int dInd = round((points[i].range-min_d)/unit_d);
			int pitchInd = round((points[i].vertical_angle-min_pitch)/unit_pitch);
			int yawInd = round( (points[i].horizon_angle-min_yaw)/unit_Yaw );
			
			int b = pitchInd;
			int c = yawInd;
			Eigen::Vector2i indx = Eigen::Vector2i(b,c);
			set<Eigen::Vector2i,SymCmp>::iterator iter = valid_voxel.find(indx);
			if(iter==valid_voxel.end())
			  valid_voxel.insert(indx);
			
			Voxel v;
			v.indx  = i;
			v.dis = dInd;
			v.yaw_angle = points[i].horizon_angle;
			v.pitch_angle = points[i].vertical_angle;
			voxels[b][c].push_back(v);
			point_voxles.push_back(Eigen::Vector2i(b,c));
			point_info.push_back(v);
	      }     
	      int repeat_voxel = 0;
	      for(set<Eigen::Vector2i>::iterator iter = valid_voxel.begin();iter!=valid_voxel.end();iter++)
	      {
		  Eigen::Vector2i z = *iter;
		  if(voxels[z[0]][z[1]].size()>1)
		    repeat_voxel++;
	      }
	       tcout<<"voxel总数 = "<<valid_voxel.size()<<" 重复voxel = "<<repeat_voxel<<endl;
	      
	       tcout<<"*******2.开始计算曲率*******"<<endl;
	       TicToc tic;
	       float thresh_cur;
	       if(config["curvature_method"][0]==0)
	       {
		  thresh_cur = CalculatePCLCurvatures();//搜索半径为1分米
	       }
	       else 
	       {
		  CalculateAgngleCurvatures();
		  thresh_cur = th_curvature;
	       }
	      
	      tcout<<"计算曲率使用时间(ms) = "<<tic.toc()<<endl;
	      tcout<<"曲率阈值 = "<<thresh_cur<<endl;//根据曲率设定的阈值更新哪些点是平面点哪些是角点
	      tcout<<"曲率范围 = "<<*(ordered_curves.begin())<<" "<<*(ordered_curves.rbegin())<<endl;
	      
	      /*
	      for(int i=0;i<cloud.points.size();i++)
	      {
		 int a = point_voxles[i][0];
		 int b = point_voxles[i][1];
		 if(curves[i]>=thresh_cur)//曲率非常大
		 {
		     sharp_indx.insert(i);
		     labels[a][b]=-1;//表示第一次增长时不遍历这个点
		 }
		 else if((curves[i]<thresh_cur)&&(curves[i]!=-1))//曲率较小
		 {
		    planar_indx.insert(i);
		}
	      }*/
	     
	     
	      for(int i=0;i<cloud.points.size();i++)
	      {
		 int a = point_voxles[i][0];
		 int b = point_voxles[i][1];
		 if(curves[i]>=thresh_cur)//曲率非常大
		 {
		     sharp_indx.insert(i);
		     labels[a][b]=-1;//表示第一次增长时不遍历这个点
		 }
		 else if((curves[i]<thresh_cur)&&(curves[i]!=-1))//曲率较小，并且曲率有效
		 {
		    //还要计算和上下线的方向是否一致，如果角度相差太大这个点仍旧还是角点
		    //首先搜索得到上下的相邻voxel
		    Eigen::Vector2i voxel_ind = point_voxles[i];//当前点所在的voxel坐标
		    //判断这个voxel中是否有角点
		    vector<Voxel> curr_voxel_points = voxels[voxel_ind[0]][voxel_ind[1]];
		    bool donotprocess = false;
		    //如果当前点所在的voxel存在曲率比较大的点在跳过这个点
		    for(int j=0;j<curr_voxel_points.size();j++)
		    {
		      if(curves[curr_voxel_points[j].indx]>thresh_cur)
		      {
			  donotprocess = true;
			  sharp_indx.insert(i);
			  labels[a][b]=-1;//表示第一次增长时不遍历这个点
		      }
		    }
		    if(donotprocess)
		      continue;
		    
		    //如果所在的voxel没有曲率特别大的点，则将上面1线和下面1线的定点压入到upper_indx,down_indx;
		    vector<int> upper_indx,down_indx;
		    for(int j=0;j<neighbour_voxles.size();j++)
		    {
		      Eigen::Vector2i nb = voxel_ind+neighbour_voxles[j];
		      if(Valid_Voxel(nb))
		      {
			      vector<Voxel> points = voxels[nb[0]][nb[1]];//这个voxel中的所有点
			      if(points.size()!=0)
			      {
				    for(int k=0;k<points.size();k++)
				    {
				      if(j<3)
					upper_indx.push_back(points[k].indx);
				      else
					down_indx.push_back(points[k].indx);
				    }
			      }
		      }
		    }
		    
		    //上下相邻点搜索完毕，现在开始搜索上下最近点序号min_indx_upper和min_indx_down
		    float min_dist_upper = 100000,min_dist_down=10000;
		    int min_indx_upper = -1,min_indx_down=-1;
		    Eigen::Vector3f current_point(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
		    if(upper_indx.size()!=0)
		    {
		      for(int j=0;j<upper_indx.size();j++)
		      {
			if(curves[upper_indx[j]]!=-1)
			{
			    Eigen::Vector3f ng(cloud.points[upper_indx[j]].x,cloud.points[upper_indx[j]].y,cloud.points[upper_indx[j]].z);
			    float dist = (ng-current_point).norm();
			    if(dist<min_dist_upper)
			    {
			      min_dist_upper = dist;
			      min_indx_upper = upper_indx[j];
			    }
			}
		      }
		    }
		    if(down_indx.size()!=0)
		    {
		      for(int j=0;j<down_indx.size();j++)
		      {
			if(curves[down_indx[j]]!=-1)
			{
			    Eigen::Vector3f ng(cloud.points[down_indx[j]].x,cloud.points[down_indx[j]].y,cloud.points[down_indx[j]].z);
			    float dist = (ng-current_point).norm();
			    if(dist<min_dist_down)
			    {
			      min_dist_down = dist;
			      min_indx_down = down_indx[j];
			    }
			}
		      }
		    }
		    
		    //最近点搜索完毕，然后计算和上面一个线，和下面一个线的向量夹角
		    /*
		    float angle_upper=-10000;
		    if(min_indx_upper!=-1)
		    {
			if((curves[min_indx_upper]!=-1)&&(curves[min_indx_upper]<thresh_cur))//上面线最近的点是有曲率的点且是曲率较小的点
			{
			  vector<Eigen::Vector3f> ngupper = left_right_candidates[min_indx_upper];
			  Eigen::Vector3f v1 = ngupper[0]-ngupper[1];
			  vector<Eigen::Vector3f> curr= left_right_candidates[i];
			  Eigen::Vector3f v2 = curr[0]-curr[1];
			  angle_upper = acos((v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])/(v1.norm()*v2.norm()))*180/PI;
			}
		    }
		    float angle_down=-10000;
		    if(min_indx_down!=-1)
		    {
			if((curves[min_indx_down]!=-1)&&(curves[min_indx_down]<thresh_cur))//表示这个点是有曲率的点且是曲率较小的点
			{
			  vector<Eigen::Vector3f> ngdown= left_right_candidates[min_indx_down];
			  Eigen::Vector3f v1 = ngdown[0]-ngdown[1];
			  vector<Eigen::Vector3f> curr= left_right_candidates[i];
			  Eigen::Vector3f v2 = curr[0]-curr[1];
			  angle_down = acos((v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])/(v1.norm()*v2.norm()))*180/PI;
			}
		    }*/
		    
		    //上下两个线上的点曲率必须都小于阈值
		    if((curves[min_indx_upper]!=-1)&&(curves[min_indx_upper]<thresh_cur)&&(curves[min_indx_down]!=-1)&&(curves[min_indx_down]<thresh_cur))//表示这个点是有曲率的点且是曲率较小的点
		    {
		      planar_indx.insert(i);
		    }
		    else
		    {
		      sharp_indx.insert(i);
		      labels[a][b]=-1;//表示第一次增长时不遍历这个点
		    }
		 }
		 else if(curves[i]==-1)//此点没有曲率
		 {
		    labels[a][b]=-1;//表示第一次增长时不遍历这个点
		 }
	      }//遍历所有点云结束
	      
	      
	      //再遍历一遍点云，如果这个点左右都存在角点，则这个点也改为角点
	      vector<Eigen::Vector2i> recheck_ngbour;
	      int recheck_num = 6;
	      for(int u=0;u<recheck_num;u++)
	      {
		recheck_ngbour.push_back(Eigen::Vector2i(0,u+1));
	      }
	      for(int u=0;u<recheck_num;u++)
	      {
		recheck_ngbour.push_back(Eigen::Vector2i(0,-u-1));
	      }
	      
	      vector<int> fake_planar;
	      for(set<int>::iterator it=planar_indx.begin();it!=planar_indx.end();it++)
	      {
		  bool left_sharp = false;
		  bool right_sharp = false;
		  Eigen::Vector2i voxel_ind = point_voxles[*it];
		  for(int i=0;i<recheck_ngbour.size();i++)
		  {
		    Eigen::Vector2i ng_voxel = voxel_ind + recheck_ngbour[i];
		    if(Valid_Voxel(ng_voxel))
		    {
		      if((labels[ng_voxel[0]][ng_voxel[1]]==-1)&&(i<recheck_num))
			right_sharp = true;
		      else if((labels[ng_voxel[0]][ng_voxel[1]]==-1)&&(i>recheck_num-1))
			left_sharp = true;
		    }
		  }
		  if(left_sharp&&right_sharp)
		  {
		    fake_planar.push_back(*it);
		    labels[voxel_ind[0]][voxel_ind[1]]=-1;
		    sharp_indx.insert(*it);
		  }
	      }
	      //然后从planar中删除
	      for(int i=0;i<fake_planar.size();i++)
	      {
		planar_indx.erase(fake_planar[i]);
	      }
	      tcout<<"有效曲率点云个数 = "<<valid_curvature_indx.size()<<" 无效曲率点云个数 = "<<invalid_curvature_indx.size()<<", 和 = "<<valid_curvature_indx.size()+invalid_curvature_indx.size()<<endl;
	      tcout<<"平面点的个数 = "<<planar_indx.size()<<" 角点个数 = "<<sharp_indx.size()<<", 和 = "<<planar_indx.size()+sharp_indx.size()<<endl;
    
	}//Project_Pointcloud函数结束
	  
	 
	//将曲率无效的点和现有的平面点融合 并使用增长法
	//all_delete_points = 与平面融合的点，是输出变量
	//th是点到平面距离的阈值
        void labelComponentsAccordingDist(Eigen::Vector2i voxelIndex,map<int,Eigen::Vector4f> planes,vector<int>& all_delete_points,float th)
	{
	    //首先找到距离这个voxel最近的那个平面的id
	    int start_plane_index = -100;
	    float first_min_dist = 1000000;
	    //在非平面点的周围搜索那些已经聚类的平面点，如果周围存在多个平面点，则选择距离最近的那个平面点，最近平面的序号为start_plane_index
	    for(int i=0;i<neighbours_dist.size();i++)
	    {
		Eigen::Vector2i neighbour_voxel = voxelIndex+neighbours_dist[i];//得到周围的voxel
		//判断周围的voxel是否存在
		if(Valid_Voxel(neighbour_voxel))
		{
		    int a = neighbour_voxel[0];
		    int b = neighbour_voxel[1];
		    int label = labels[a][b];
		    if(label>0)//表示周围的这个点是聚类的平面点
		    {
			if(planes.find(label)!=planes.end())//并且这个聚类的点属于有效平面中
			{
			    Eigen::Vector4f pc = planes.find(label)->second;
			    float total_dis = 0;
			    for(int k=0;k<voxels[voxelIndex[0]][voxelIndex[1]].size();k++)				  
			    {
			      int id = voxels[voxelIndex[0]][voxelIndex[1]][k].indx;
			      float x=  cloud.points[id].x;
			      float y = cloud.points[id].y;
			      float z = cloud.points[id].z;
			      float dis = abs(pc[0]*x+pc[1]*y+pc[2]*z+pc[3]);
			      total_dis = total_dis + dis;
			    }
			    float avg_dis = total_dis/voxels[voxelIndex[0]][voxelIndex[1]].size();
			    if(avg_dis<th)
			    {
			      if(first_min_dist>avg_dis)
			      {
				start_plane_index = label;
				first_min_dist = avg_dis;
			      }
			    }
			}
		    }
		}
	     }//
	  
	    //如果这个非平面点周围存在已经聚类的平面点
	    //然后再在这个非平面点上进行增长，将
	    if(start_plane_index!=-100)
	    {
			vector<Eigen::Vector2i> allPushedInd;//存储的是根据输入的voxel进行聚类之后的所有的voxel序号
			int queueSize = 1;
			Eigen::Vector2i fromInd,thisInd;
			int queueStartId = 0;
			int queueEndInd = 1;
			queueInd[0] = voxelIndex;
			//allPushedInd.push_back(voxelIndex);
			while(queueSize>0)
			{
				fromInd = queueInd[queueStartId];
				--queueSize;
				++queueStartId;
			    
	    // 		if(labels[fromInd[0]][fromInd[1]]==-1)//表示这个voxel属于角点和无效的点云点
	    // 		if(labels[fromInd[0]][fromInd[1]]<0)//表示这个voxel属于角点和无效的点云点
			    //{
				//在当前voxel周围搜索是否存在已经聚类好平面的voxel
				  map<int,float> candidate_plane;//第一个元素是 已经分类好的平面的序号 第二个内容是当前点到这个平面的距离
				  for(int i=0;i<neighbours_dist.size();i++)
				  {
				      Eigen::Vector2i neighbour_voxel = fromInd+neighbours_dist[i];//得到周围的voxel
				      //判断周围的voxel是否存在
				      if(Valid_Voxel(neighbour_voxel))
				      {
					  int a = neighbour_voxel[0];
					  int b = neighbour_voxel[1];
					  int label = labels[a][b];
  // 					if(label>0)//表示周围的这个点是聚类的平面点
					  if(label==start_plane_index)
					  {
					      if(planes.find(label)!=planes.end())//并且这个聚类的点属于有效平面中
					      {
						  Eigen::Vector4f pc = planes.find(label)->second;
						  float total_dis = 0;
						  for(int k=0;k<voxels[fromInd[0]][fromInd[1]].size();k++)				  
						  {
						    int id = voxels[fromInd[0]][fromInd[1]][k].indx;
						    float x=  cloud.points[id].x;
						    float y = cloud.points[id].y;
						    float z = cloud.points[id].z;
						    float dis = abs(pc[0]*x+pc[1]*y+pc[2]*z+pc[3]);
						    total_dis = total_dis + dis;
						  }
						  float avg_dis = total_dis/voxels[fromInd[0]][fromInd[1]].size();
						  if(avg_dis<th)
						  {
						    candidate_plane.insert(pair<int,float>(label,avg_dis));
						  }
					      }
					  }
				      }
				  }//for结束
				  
				  if(candidate_plane.size()==0)//表示周围没有可用的平面点
				  {
				    continue;
				  }
				  else//匹配的平面数量大于等于1个
				  {
				      allPushedInd.push_back(fromInd);
				      //从候选平面中选择最近的那个平面
// 				      float min_dist =  candidate_plane.begin()->second;
// 				      int min_plane = candidate_plane.begin()->first;
// 				      for(map<int,float>::iterator iter = candidate_plane.begin();iter!=candidate_plane.end();iter++)
// 				      {
// 					if(min_dist>iter->second)
// 					{
// 					  min_dist = iter->second;
// 					  min_plane = iter->first;
// 					}
// 				      }
				    
				      labels[fromInd[0]][fromInd[1]]= start_plane_index;//更新label中这个voxel的聚类信息
				      
				      
				      //将要从角点和无效点中删除点的点云序号
				      for(int i=0;i<voxels[fromInd[0]][fromInd[1]].size();i++)
				      {
					int id = voxels[fromInd[0]][fromInd[1]][i].indx;
					all_delete_points.push_back(id);
				      }
				      
				      
				      for(int j=0;j<neighbours_dist.size();j++)
				      {
					  thisInd =  fromInd + neighbours_dist[j];
					  if(Valid_Voxel(thisInd))
					  { 
					      int label = labels[thisInd[0]][thisInd[1]];
					      if(label<0)
					      {
						  bool valid = false;
						  if(planes.find(start_plane_index)!=planes.end())//并且这个聚类的点属于有效平面中
						  {
						      Eigen::Vector4f pc = planes.find(start_plane_index)->second;
						      float total_dis = 0;
						      for(int k=0;k<voxels[thisInd[0]][thisInd[1]].size();k++)				  
						      {
							int id = voxels[thisInd[0]][thisInd[1]][k].indx;
							float x=  cloud.points[id].x;
							float y = cloud.points[id].y;
							float z = cloud.points[id].z;
							float dis = abs(pc[0]*x+pc[1]*y+pc[2]*z+pc[3]);
							total_dis = total_dis + dis;
						      }
						      float avg_dis = total_dis/voxels[thisInd[0]][thisInd[1]].size();
						      if(avg_dis<th)
						      {
							valid = true;
						      }
						  }
						  if(valid)
						  {
						      queueInd[queueEndInd] = thisInd;//向队列中增加新元素
						      queueSize++;
						      queueEndInd++;
						      labels[thisInd[0]][thisInd[1]] = start_plane_index;
						      allPushedInd.push_back(thisInd);
						  }
					      }
					  }
				      }//for循环结束
				  }
			    //}
			    
			}//while 循环结束
			
			
			//区域增长结束 更新eachsegmentcloud中的内容
			for(int i=0;i<allPushedInd.size();i++)
			{
			    int a = allPushedInd[i][0];
			    int b = allPushedInd[i][1];
			    int label = labels[a][b];
			    for(int j=0;j<voxels[a][b].size();j++)
			    {
			      each_segment_cloud.find(label)->second.push_back(voxels[a][b][j].indx);
			    }
			}
	    }
      }//labelComponentsAccordingDist函数结束
	  
	  
	  void labelComponents(Eigen::Vector2i voxelIndex)
	  {
		vector<Eigen::Vector2i> allPushedInd;//存储的是根据输入的voxel进行聚类之后的所有的voxel序号
		int queueSize = 1;
		Eigen::Vector2i fromInd,thisInd;
		int queueStartId = 0;
		int queueEndInd = 1;
		queueInd[0] = voxelIndex;
		//allPushedInd.push_back(voxelIndex);
		while(queueSize>0)
		{
		    fromInd = queueInd[queueStartId];
		    --queueSize;
		    ++queueStartId;
		    if(labels[fromInd[0]][fromInd[1]]==-2)//表示这个voxel是候选点
		    {
		      labels[fromInd[0]][fromInd[1]] = lableCount;
		      allPushedInd.push_back(fromInd);
		    }
		    
		    for(int i=0;i<neighbours.size();i++)
		    {
			thisInd =  fromInd + neighbours[i];
			if(Valid_Voxel(thisInd))
			{
			      if((labels[thisInd[0]][thisInd[1]]==-2)&&(voxels[thisInd[0]][thisInd[1]].size()!=0))//表示这个点还没有参与聚类 并且允许聚类，并且这个点有对应的坐标点
			      {
// 				int dis1 = voxels[fromInd[0]][fromInd[1]][0].dis;
// 				int dis2 = voxels[thisInd[0]][thisInd[1]][0].dis;
				
				int id1 = voxels[fromInd[0]][fromInd[1]][0].indx;
				int id2 = voxels[thisInd[0]][thisInd[1]][0].indx;
				Eigen::Vector3f fromPoint(cloud.points[id1].x,cloud.points[id1].y,cloud.points[id1].z);
				Eigen::Vector3f thisPoint(cloud.points[id2].x,cloud.points[id2].y,cloud.points[id2].z);
// 				if(abs(dis1-dis2)<5)*
				//float angle = 180.0*(allnormal[id1](0)*allnormal[id2](0)+allnormal[id1](1)*allnormal[id2](1)+allnormal[id1](2)*allnormal[id2](2))/(allnormal[id1].norm()*allnormal[id2].norm())/PI;
				if((fromPoint-thisPoint).norm()<0.25)
				{
				    queueInd[queueEndInd] = thisInd; 
				    queueSize++;
				    queueEndInd++;
				    
				    labels[thisInd[0]][thisInd[1]] = lableCount;
				    allPushedInd.push_back(thisInd);
				}
			      }
			}
		    }//for循环结束
		}//while 循环结束
		int total_num = 0;
		for(int i=0;i<allPushedInd.size();i++)
		{
		    int a = allPushedInd[i][0];
		    int b = allPushedInd[i][1];
		    int num = voxels[a][b].size();
		    total_num = total_num + num;
		}
		
		if(total_num>th_cluster)//聚类成功
		{
		  vector<int> seg_cloud;
		  for(int i=0;i<allPushedInd.size();i++)
		  {
		      int a = allPushedInd[i][0];
		      int b = allPushedInd[i][1];
		      for(int j=0;j<voxels[a][b].size();j++)
		      {
			int indx_cloud = voxels[a][b][j].indx;
			cloud.points[indx_cloud].intensity = lableCount;
			seg_cloud.push_back(indx_cloud);
		      }
		  }
		  each_segment_cloud.insert(pair<int,vector<int>>(lableCount,seg_cloud));
		  ++lableCount;
		}
		else//聚类失败
		{
		    for(int i=0;i<allPushedInd.size();i++)
		    {
			int a = allPushedInd[i][0];
			int b = allPushedInd[i][1];
			labels[a][b]= -3;
			for(int j=0;j<voxels[a][b].size();j++)
			{
			  int indx_cloud = voxels[a][b][j].indx;
			  bad_plane_indx.insert(indx_cloud);//更新失败的平面点
			}
		    }
		}
	      
	  }//labelComponents函数结束
	  
	  void SegmentCloud_PCL()
	  {
		//创建一个指向kd树搜索对象的共享指针
		pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;//创建法线估计对象

		normal_estimator.setSearchMethod (tree);//设置搜索方法

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
		cloud_ptr = cloud.makeShared();
		normal_estimator.setInputCloud (cloud_ptr);//设置法线估计对象输入点集

		normal_estimator.setKSearch (50);// 设置用于法向量估计的k近邻数目

		normal_estimator.compute (*normals);//计算并输出法向量
		
		
		pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;//创建区域生长分割对象
		reg.setMinClusterSize (th_cluster);//最小的聚类数目
		reg.setMaxClusterSize (1000000);//最大的聚类数目
		reg.setSearchMethod (tree);//设置搜索方法
		reg.setNumberOfNeighbours (30);//设置搜索的临近点数目
		reg.setInputCloud (cloud_ptr);//设置输入点云
		reg.setInputNormals (normals);//设置输入点云的法向量
		reg.setSmoothnessThreshold (config["setSmoothnessThreshold"][0] / 180.0 * PI);//设置平滑阈值
		reg.setCurvatureThreshold (config["setCurvatureThreshold"][0]);//设置曲率阈值
		
		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);//获取聚类的结果，分割结果保存在点云索引的向量中。
		
		int index = 0;
		for(int i=0;i<clusters.size();i++)
		{
		  vector<int> seg_index;
		  for(int j=0;j<clusters[i].indices.size();j++)
		  {
		    seg_index.push_back(clusters[i].indices[j]);
		  }
		  each_segment_cloud.insert(pair<int,vector<int>>(index,seg_index));
		  index++;
		}
	  }
	  
	  float Calculate_Area(vector<cv::Point2f> points)
	  {
	    vector<cv::Point2f> cv_points_Hull;
// 	      vector<int> temps;
	  
	    cv::convexHull(points, cv_points_Hull);//第二个输入的参数也可以是vector<int>
// 	      cv::convexHull(cv_points, temps);
// 	      for(int i=0;i<temps.size();i++)
// 	      {
// 		cout<<temps[i]<<" ";
// 	      }
// 	      cout<<endl;
	    float area = (float)cv::contourArea(cv_points_Hull);
	    return area;
	  }
	  
	  template<class T>
	  vector<cv::Point2f> transformToXYPlane(pcl::PointCloud<T> cloud,Eigen::Vector4f model)
	  {
	    Eigen::Vector3f norm_XY(0,0,1);
	    Eigen::Vector3f norm_w(model[0],model[1],model[2]);
	    if(norm_XY.dot(norm_w)<0)
	      norm_w = -norm_w;
	    Eigen::Vector3f axis = norm_XY.cross(norm_w);
	    axis = axis/axis.norm();
	    float theteta = (float)acos((norm_XY.dot(norm_w))/norm_w.norm() );// [0,pi] 
	    
	    Eigen::AngleAxisf angleaxis(-theteta,axis);
	    Eigen::Matrix3f R=angleaxis.toRotationMatrix();
	    vector<cv::Point2f> res;
	    for(int i=0;i<cloud.points.size();i++)
	    {
	      float x=  cloud.points[i].x;
	      float y = cloud.points[i].y;
	      float z = cloud.points[i].z;
	      Eigen::Vector3f p(x,y,z);
	      Eigen::Vector3f temp = R*p;
	      //cout<<temp.transpose()<<endl;
// 		    cout<<model[3]<<" "<<temp[2]<<endl;
	      res.push_back(cv::Point2f(temp[0],temp[1]));
	    }
	    return res;
	  }
	  void SegmentCloud()
	  {
	        //1.首先根据曲率进行区域增长，遇到曲率过大的平面就停止增长
		tcout<<"*******3.开始聚类*******"<<endl;
		set<Eigen::Vector2i,SymCmp>::iterator iter_set;
		for(iter_set=valid_voxel.begin();iter_set!=valid_voxel.end();iter_set++)
		{
		    int a = (*iter_set)[0];
		    int b = (*iter_set)[1];
		    if(labels[a][b]==-2)//这个点允许进行聚类 并且还没有聚类，初始值就是-2
		    {
		      labelComponents(*iter_set);//对某一个的voxel进行聚类，对lables进行更新
		    }
		}
		int totalnum_planar = 0;
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		  totalnum_planar = totalnum_planar + iter->second.size() ;
		}
		tcout<<"好的平面点个数 = "<<totalnum_planar<<" 坏的平面点个数 = "<<bad_plane_indx.size()<<", 和 = "<<totalnum_planar+bad_plane_indx.size()<<endl;
		
		
		
		//2。计算聚类平面的参数
		tcout<<"*******3.1计算平面参数*******"<<endl;
		map<int,Eigen::Vector4f> models;///,models_own;
		map<int,float>  inliner_perct,pca_analysis,pca_analysis_own,areas,density;
		map<int,Eigen::Vector3f> pca_norm;//我们自己计算得到的平面法向量
		map<int,pcl::PointCloud<pcl::PointXYZI>> inliner_clouds;
 		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		    int label = iter->first;
		    pcl::PointCloud<pcl::PointXYZI> clusterCloud = Obtain_PCLCloud(iter->second);
		    //拟合平面的参数
		    pcl::PointCloud<pcl::PointXYZI> inliner_cloud;
		    Eigen::Vector4f model = Calculate_Planar_Model(clusterCloud,inliner_cloud,th_plane);//经过我们自己的验证 pcl得到的inliner数量和我们自己计算得到的相同
		    models.insert( pair<int,Eigen::Vector4f>( label,model) );
		    inliner_clouds.insert(pair<int,pcl::PointCloud<pcl::PointXYZI>>(label,inliner_cloud));
		    
		    //float own_pca;
		    //Eigen::VectorXf model_own = Calculate_Planar_Model_Own(inliner_cloud,own_pca);
		    //pca_analysis_own.insert(pair<int,float>(label,own_pca));
		    //models_own.insert( pair<int,Eigen::Vector4f>( label,Eigen::Vector4f(model_own(0),model_own(1),model_own(2),model_own(3)) ) );		   
		    
		    //参数1：inliner百分比
		    float percentage = (float)inliner_cloud.points.size()/(float)clusterCloud.points.size();
		    inliner_perct.insert(pair<int,float>(label,percentage));

		    //参数3：平面的主方向
		    Eigen::Vector3f norm_vector;
		    float pca_ind = Obtain_PCACurve(ConvertToEigen(inliner_cloud),norm_vector);//计算属于一个类的点云的主方向
		    pca_analysis.insert(pair<int,float>(label,pca_ind));
		    pca_norm.insert(pair<int,Eigen::Vector3f>(label,norm_vector));
		    
		    //参数4 计算面积
		    //首先将所有点的inliner投影到平面上
		    pcl::PointCloud<pcl::PointXYZI> plannar_cloud;
		    plannar_cloud = Project_Cloud<pcl::PointXYZI>(inliner_cloud,Eigen::Vector3f(0,0,0),model);//将点投影到我们估计得到的平面模型中
		    vector<cv::Point2f> xy_points = transformToXYPlane<pcl::PointXYZI>(plannar_cloud,model);//计算平面得到的面积
		    float area = Calculate_Area(xy_points);
		    areas.insert(pair<int,float>(label,area));
		    
		    //参数5 计算inliner密度
		    float den = area/(float)inliner_cloud.points.size();//每个点占有多少面积
		    density.insert(pair<int,float>(label,den));
		}
		//3.对得到的平面进行筛选
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		    int label = iter->first;
		    float perct = inliner_perct[label];
		    float pca = pca_analysis[label];
		    float pca_own = pca_analysis_own[label];
		    int num = inliner_clouds[label].points.size();
		    float area = areas[label];
		    float den = density[label];
		    Eigen::Vector4f temp_plane =  models[label];
		    Eigen::Vector3f pcl_norm(temp_plane(0),temp_plane(1),temp_plane(2));

		    float diff_angle = acos(abs(pcl_norm.transpose()*pca_norm.find(label)->second))*180/PI;
		    //printf("面积= %f 密度 = %f inliner百分比 = %f inliner数量 = %d 法方向夹角 = %f度",area,den,perct,num,diff_angle);
		    //printf("pcl pca系数= %f 自己计算得到的pca系数 = %f",pca,pca_own);
		    //这里选择的条件是面积够大并且inliner数量要大于50个，并且pcl算出来的法向量和我们自己计算得到的角度相差不超过10度，并且inliner百分比要超过百分之80
		    if((area>th_area)&&(num>50)&&(diff_angle<30))//A4的面积为0.06，
		    {
 		        tcout<<" good"<<endl;
		    }
		    else
		    {
		      tcout<<" bad"<<endl;
		      for(int k=0;k<each_segment_cloud[label].size();k++)
		      {
			Eigen::Vector2i voxel_indx = point_voxles[each_segment_cloud[label][k]];
			labels[voxel_indx[0]][voxel_indx[1]] = -3;//-3表示不参与第二次增长
			for(int m=0;m<voxels[voxel_indx[0]][voxel_indx[1]].size();m++)
			{
			  bad_plane_indx.insert(voxels[voxel_indx[0]][voxel_indx[1]][m].indx);
			}
		      }
		      each_segment_cloud.erase(label);
		      models.erase(label);
		    }
		}
		totalnum_planar = 0;
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		  totalnum_planar = totalnum_planar + iter->second.size() ;
		}
		tcout<<"筛选之后好的平面点个数 = "<<totalnum_planar<<" 筛选之后坏的平面点个数 = "<<bad_plane_indx.size()<<", 和 = "<<totalnum_planar+bad_plane_indx.size()<<endl;
	    
	        vector<int> deleted_points;
		set<int> nonplanar_indx;
		pcl::PointCloud<pcl::PointXYZI> allofSegmentCloud;
		//4.对invalid和角点进行区域增长，与现有的平面进行融合
		if(config["grow_second"][0]==1)
		{
		    tcout<<"*******3.2第二次区域增长*******"<<endl;
		    queueInd.clear();//用于bfs搜索的
		    int numofinvalid = 0;
		    //首先遍历曲率无效的点
		    nonplanar_indx.insert(invalid_curvature_indx.begin(),invalid_curvature_indx.end());
		    nonplanar_indx.insert(sharp_indx.begin(),sharp_indx.end());
		    nonplanar_indx.insert(bad_plane_indx.begin(),bad_plane_indx.end());
		    for(set<int>::iterator iter = nonplanar_indx.begin();iter!=nonplanar_indx.end();iter++)
		    {
		      Eigen::Vector2i voxel_coordiante = point_voxles[*iter];//根据点云序号找到voxle
		      int a = voxel_coordiante[0];
		      int b = voxel_coordiante[1];
		      if(labels[a][b]<0)
			labelComponentsAccordingDist(voxel_coordiante,models,deleted_points,th_plane);//deleted_points要从曲率无效的点删除的点
		    }
		    //从invalid点和角点删除又增长的点
		    for(int i=0;i<deleted_points.size();i++)
		    {
		      set<int>::iterator itera = invalid_curvature_indx.find(deleted_points[i]);
		      set<int>::iterator iterb = sharp_indx.find(deleted_points[i]);
		      set<int>::iterator iterc = bad_plane_indx.find(deleted_points[i]);
		      if(itera!=invalid_curvature_indx.end())
		      {
			invalid_curvature_indx.erase(*itera);
		      }
		      if(iterb!=sharp_indx.end())
		      {
			sharp_indx.erase(*iterb);
		      }
		      if(iterc!=bad_plane_indx.end())
		      {
			bad_plane_indx.erase(*iterc);
		      }
		    }
		    tcout<<"第二次成功融合的聚类点个数 = "<<deleted_points.size()<<endl;
		    
		    
		    models.clear();
		    for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		    {
			totalnum_planar = totalnum_planar + iter->second.size() ;
			
			int label = iter->first;
			pcl::PointCloud<pcl::PointXYZI> clusterCloud = Obtain_PCLCloud(iter->second);
			//拟合平面的参数
			pcl::PointCloud<pcl::PointXYZI> inliner_cloud;
			Eigen::Vector4f model = Calculate_Planar_Model(clusterCloud,inliner_cloud,th_plane);//经过我们自己的验证 pcl得到的inliner数量和我们自己计算得到的相同
			models.insert( pair<int,Eigen::Vector4f>( label,model) );
		    }
		    tcout<<"好的平面点个数 = "<<totalnum_planar<<" 坏的平面点个数 = "<<bad_plane_indx.size()<<", 和 = "<<totalnum_planar+bad_plane_indx.size()<<endl;
		}
		
		vector<Eigen::Vector2i> num_and_label;//一个元素是大小
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		    totalnum_planar = totalnum_planar + iter->second.size() ;
		    num_and_label.push_back(Eigen::Vector2i(totalnum_planar,iter->first));
		    for(int i=0;i<iter->second.size();i++)
		    {
		      allofSegmentCloud.points.push_back(cloud.points[iter->second[i]]);
		    }
		}
		
		
		
		// 第三次增长 遍历不是平面的所有点 寻找和他距离最近的平面点 然后判断这个点距离这个平面是否小于阈值如果小于阈值则我们将其归入这个平面中
		if(config["grow_third"][0]==1)
		{
		    tcout<<"*******3.3第三次区域增长*******"<<endl;
		    deleted_points.clear();
		    nonplanar_indx.clear();
		    nonplanar_indx.insert(invalid_curvature_indx.begin(),invalid_curvature_indx.end());
		    nonplanar_indx.insert(sharp_indx.begin(),sharp_indx.end());
		    nonplanar_indx.insert(bad_plane_indx.begin(),bad_plane_indx.end());
		    
		    pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
		    kd_tree.setInputCloud(allofSegmentCloud.makeShared());
		    //遍历所有非平面的点，并在所有平面点中搜索，找到最近的那个点，然后计算这个点到达这个平面的距离，如果比较小则将这个非平面点划归为平面点
		    for(set<int>::iterator iter = nonplanar_indx.begin();iter!=nonplanar_indx.end();iter++)
		    {
		      int id = *iter;
		      pcl::PointXYZI searchpoint;
		      searchpoint.x = cloud.points[id].x;
		      searchpoint.y = cloud.points[id].y;
		      searchpoint.z = cloud.points[id].z;
		      vector<int> pointIdxNKNSearch(1);
		      vector<float> pointIdxNKNSquareDistance(1);
		      if(kd_tree.nearestKSearch(searchpoint,1,pointIdxNKNSearch,pointIdxNKNSquareDistance)>0)
		      {
			int search_indx = pointIdxNKNSearch[0];
			int belong_label;
			for(int i=0;i<num_and_label.size();i++)
			{
			  if(search_indx<num_and_label[i][0])
			  {
			    belong_label = num_and_label[i][1];
			    break;
			  }
			}
			//计算到平面的距离
			float dist = models[belong_label][0]*searchpoint.x+models[belong_label][1]*searchpoint.y+models[belong_label][2]*searchpoint.z+models[belong_label][3];
			if(abs(dist)<th_plane)
			{
			  deleted_points.push_back(id);
			  each_segment_cloud[belong_label].push_back(id);
			}
		      }
		    }
		    tcout<<"第三次成功融合的聚类点个数 = "<<deleted_points.size()<<endl;
		    //从invalid点和角点删除又增长的点
		    for(int i=0;i<deleted_points.size();i++)
		    {
		      set<int>::iterator itera = invalid_curvature_indx.find(deleted_points[i]);
		      set<int>::iterator iterb = sharp_indx.find(deleted_points[i]);
		      set<int>::iterator iterc = bad_plane_indx.find(deleted_points[i]);
		      if(itera!=invalid_curvature_indx.end())
		      {
			invalid_curvature_indx.erase(*itera);
		      }
		      if(iterb!=sharp_indx.end())
		      {
			sharp_indx.erase(*iterb);
		      }
		      if(iterc!=bad_plane_indx.end())
		      {
			bad_plane_indx.erase(*iterc);
		      }
		    }
		    
		    totalnum_planar = 0;
		    models.clear();
		    for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		    {
		      totalnum_planar = totalnum_planar + iter->second.size() ;
		      
		      int label = iter->first;
		      pcl::PointCloud<pcl::PointXYZI> clusterCloud = Obtain_PCLCloud(iter->second);
		      //拟合平面的参数
		      pcl::PointCloud<pcl::PointXYZI> inliner_cloud;
		      Eigen::Vector4f model = Calculate_Planar_Model(clusterCloud,inliner_cloud,th_plane);//经过我们自己的验证 pcl得到的inliner数量和我们自己计算得到的相同
		      models.insert( pair<int,Eigen::Vector4f>( label,model ) );
		    }
		    tcout<<"好的平面点个数 = "<<totalnum_planar<<" 坏的平面点个数 = "<<bad_plane_indx.size()<<", 和 = "<<totalnum_planar+bad_plane_indx.size()<<endl;
		}
		
		/*
		//再重复一遍第二次搜索
		queueInd.clear();//用于bfs搜索的
		deleted_points.clear();
		//首先遍历曲率无效的点
		nonplanar_indx.clear();
		nonplanar_indx.insert(invalid_curvature_indx.begin(),invalid_curvature_indx.end());
		nonplanar_indx.insert(sharp_indx.begin(),sharp_indx.end());
		nonplanar_indx.insert(bad_plane_indx.begin(),bad_plane_indx.end());
		for(set<int>::iterator iter = nonplanar_indx.begin();iter!=nonplanar_indx.end();iter++)
		{
		  Eigen::Vector2i voxel_coordiante = point_voxles[*iter];//根据点云序号找到voxle
		  int a = voxel_coordiante[0];
		  int b = voxel_coordiante[1];
// 		  if(labels[a][b]==-1)
		  if(labels[a][b]<0)
		    labelComponentsAccordingDist(voxel_coordiante,models,deleted_points,th_plane);//deleted_points要从曲率无效的点删除的点
		}
		//从invalid点和角点删除又增长的点
		for(int i=0;i<deleted_points.size();i++)
		{
		  set<int>::iterator itera = invalid_curvature_indx.find(deleted_points[i]);
		  set<int>::iterator iterb = sharp_indx.find(deleted_points[i]);
		  set<int>::iterator iterc = bad_plane_indx.find(deleted_points[i]);
		  if(itera!=invalid_curvature_indx.end())
		  {
		    invalid_curvature_indx.erase(*itera);
		  }
		  if(iterb!=sharp_indx.end())
		  {
		    sharp_indx.erase(*iterb);
		  }
		  if(iterc!=bad_plane_indx.end())
		  {
		    bad_plane_indx.erase(*iterc);
		  }
		}
		cout<<"第四次成功融合的聚类点个数 = "<<deleted_points.size()<<endl;
		
		
		totalnum_planar = 0;
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		  totalnum_planar = totalnum_planar + iter->second.size() ;
		}
		cout<<"7**"<<endl;
	        cout<<"过滤之后点云个数 = "<<cloud.points.size()<<endl;
	        cout<<"有效曲率点云个数 = "<<valid_curvature_indx.size()<<" 无效曲率点云个数 = "<<invalid_curvature_indx.size()<<", 和 = "<<valid_curvature_indx.size()+invalid_curvature_indx.size()<<endl;
	        cout<<"平面点的个数 = "<<planar_indx.size()<<" 角点个数 = "<<sharp_indx.size()<<", 和 = "<<planar_indx.size()+sharp_indx.size()<<endl;
		cout<<"好的平面点个数 = "<<totalnum_planar<<" 坏的平面点个数 = "<<bad_plane_indx.size()<<", 和 = "<<totalnum_planar+bad_plane_indx.size()<<endl;
		// 第三次增长 遍历不是平面的所有点 寻找和他距离最近的平面点 然后判断这个点距离这个平面是否小于阈值如果小于阈值则我们将其归入这个平面中
		*/
	}
	 
	 void Publish_Topic()
	 {
		 //有曲率的点
		 //原始点云
		if(originalCloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 original_cloud_msg;
		  //pcl::toROSMsg(*original_cloud,original_cloud_msg);
		  pcl::toROSMsg(cloud,original_cloud_msg);
		  original_cloud_msg.header.frame_id = "/velodyne";
		  original_cloud_msg.header.stamp = ros::Time(timestamp);
		  originalCloud_pub.publish(original_cloud_msg);
		}
		if(valid_curvature_cloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 valid_curvature_cloud_msg;
		  pcl::toROSMsg(valid_curvature_cloud,valid_curvature_cloud_msg);
		  valid_curvature_cloud_msg.header.frame_id = "/velodyne";
		  valid_curvature_cloud_msg.header.stamp = ros::Time(timestamp);
		  valid_curvature_cloud_pub.publish(valid_curvature_cloud_msg);
		}
		//计算不出来曲率的点
		if(invalid_curvature_cloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 invalid_curvature_cloud_msg;
		  pcl::toROSMsg(invalid_curvature_cloud,invalid_curvature_cloud_msg);
		  invalid_curvature_cloud_msg.header.frame_id = "/velodyne";
		  invalid_curvature_cloud_msg.header.stamp = ros::Time(timestamp);
		  invalid_curvature_cloud_pub.publish(invalid_curvature_cloud_msg);
		}
		//曲率小的点
		if(planarCloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 planarCloud_msg;
		  pcl::toROSMsg(planarCloud,planarCloud_msg);
		  planarCloud_msg.header.frame_id = "/velodyne";
		  planarCloud_msg.header.stamp = ros::Time(timestamp);
		  planarCloud_pub.publish(planarCloud_msg);
		}
		
		//曲率大的点
		if(sharpCloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 sharp_cloud_msg;
		  pcl::toROSMsg(sharpCloud,sharp_cloud_msg);
		  sharp_cloud_msg.header.frame_id = "/velodyne";
		  sharp_cloud_msg.header.stamp = ros::Time(timestamp);
		  sharpCloud_pub.publish(sharp_cloud_msg);
		}
		//聚类失败的平面点
		if(badplane_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 badplanecloud_msg;
		  pcl::toROSMsg(badplane_cloud,badplanecloud_msg);
		  badplanecloud_msg.header.frame_id = "/velodyne";
		  badplanecloud_msg.header.stamp = ros::Time(timestamp);
		  badplane_pub.publish(badplanecloud_msg);
		}
		
		
		
		
		//最后将分割好的原始点云publish出去
		if(segment_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 segmentcloud_msg;
		   for(int i=0;i<pointWithLael.points.size();i++)
		   {
		     pointWithLael.points[i].y = pointWithLael.points[i].y + show_bias;
		   }
		  pcl::toROSMsg(pointWithLael,segmentcloud_msg);
		  segmentcloud_msg.header.frame_id = "/velodyne";
		  segmentcloud_msg.header.stamp = ros::Time(timestamp);
		  segment_pub.publish(segmentcloud_msg);
		}
		//分割好的原始点云的inliner点
		if(pointWithLael_inliner_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 pointWithLael_inliner_msg;
		  for(int i=0;i<pointWithLael_inliner.points.size();i++)
		  {
		    pointWithLael_inliner.points[i].y = pointWithLael_inliner.points[i].y + 2*show_bias;
		  }
		  pcl::toROSMsg(pointWithLael_inliner,pointWithLael_inliner_msg);
		  pointWithLael_inliner_msg.header.frame_id = "/velodyne";
		  pointWithLael_inliner_msg.header.stamp = ros::Time(timestamp);
		  pointWithLael_inliner_pub.publish(pointWithLael_inliner_msg);
		}
		
		//分割好的投影到平面上的点
		if(project_cloud_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 project_cloud_msg;
		  for(int i=0;i<project_cloud.points.size();i++)
		  {
		    project_cloud.points[i].y = project_cloud.points[i].y + 3*show_bias;
		  }
		  pcl::toROSMsg(project_cloud,project_cloud_msg);
		  project_cloud_msg.header.frame_id = "/velodyne";
		  project_cloud_msg.header.stamp = ros::Time(timestamp);
		  project_cloud_pub.publish(project_cloud_msg);
		}
		
		if(final_three_plan_pub.getNumSubscribers()>0)
		{
		  sensor_msgs::PointCloud2 test_cloud_msg;
		  //test_cloud = cloud;
		  pcl::toROSMsg(test_cloud,test_cloud_msg);
		  test_cloud_msg.header.frame_id = "/velodyne";
		  test_cloud_msg.header.stamp = ros::Time(timestamp);
		  final_three_plan_pub.publish(test_cloud_msg);
		}
	}
	 
	  map<int,pcl::PointCloud<pcl::PointXYZI>> Obtain_SegementCloud(pcl::PointCloud<pcl::PointXYZI>& out_cloud,map<int,Eigen::Vector4f>& models)
	  {

		tcout<<endl;
		tcout<<endl;	
	        //1.
		Project_Pointcloud();
 		//2.然后对点云进行分类
  		SegmentCloud();
// 		SegmentCloud_PCL();
		
		//3.获得对应的点云
	        valid_curvature_cloud = Obtain_PCLCloud(valid_curvature_indx);
	        planarCloud = Obtain_PCLCloud(planar_indx);
		
		sharpCloud = Obtain_PCLCloud(sharp_indx);
		badplane_cloud = Obtain_PCLCloud(bad_plane_indx);
		invalid_curvature_cloud = Obtain_PCLCloud(invalid_curvature_indx);
		
		
		//4.计算每个平面的参数
		map<int,pcl::PointCloud<pcl::PointXYZI>> planar_clouds;//投影到平面上的点
		map<int,pcl::PointCloud<pcl::PointXYZI>> non_plannar_cloud;//还没有进行投影的平面点
		for(map<int,vector<int>>::iterator iter=each_segment_cloud.begin(); iter!=each_segment_cloud.end();iter++)
		{
		    vector<int> p=iter->second;
		    int label = iter->first;
		    
		    pcl::PointCloud<pcl::PointXYZI> clusterCloud = Obtain_PCLCloud(iter->second);
		    non_plannar_cloud.insert( pair<int,pcl::PointCloud<pcl::PointXYZI>>(label,clusterCloud) );
		    for(int i=0;i<clusterCloud.points.size();i++)
		    {
		      pcl::PointXYZI temppointYXZI;
		      temppointYXZI = clusterCloud.points[i];
		      temppointYXZI.intensity = label;
		      pointWithLael.points.push_back(temppointYXZI);
		    }
		    //拟合平面的参数
		    pcl::PointCloud<pcl::PointXYZI> inliner_cloud;
		    Eigen::Vector4f model = Calculate_Planar_Model(clusterCloud,inliner_cloud,th_plane);//经过我们自己的验证 pcl得到的inliner数量和我们自己计算得到的相同
		    for(int i=0;i<inliner_cloud.points.size();i++)
		    {
		      pcl::PointXYZI temppointYXZI;
		      temppointYXZI = inliner_cloud.points[i];
		      temppointYXZI.intensity = label;
		      pointWithLael_inliner.points.push_back(temppointYXZI);
		    }
		    models.insert( pair<int,Eigen::Vector4f>( label, model ) );
		    
		    pcl::PointCloud<pcl::PointXYZI> plannar_cloud;
		    plannar_cloud = Project_Cloud<pcl::PointXYZI>(clusterCloud,Eigen::Vector3f(0,0,0),model);//将点投影到我们估计得到的平面模型中
		    planar_clouds.insert( pair<int,pcl::PointCloud<pcl::PointXYZI>>(label,plannar_cloud) );
		}
		
		//5.将相似的平面进行融合
		planar_clouds = Fusion_Plane(models,non_plannar_cloud);//非常重要!!!!!!!!!!!!!!！！！！！！！！！！！！	
		//更新投影之后的点
		for(map<int,pcl::PointCloud<pcl::PointXYZI>>::iterator iter =planar_clouds.begin();iter!=planar_clouds.end();iter++)
		{
		  int label = iter->first;
		  for(int i=0;i<iter->second.points.size();i++)
		  {
		    pcl::PointXYZI tempXYZI = iter->second.points[i];
		    tempXYZI.intensity = label;
		    project_cloud.points.push_back(tempXYZI);
		  }
		}
		
		//6.找到三个相互垂直的平面
		map<int,Eigen::Vector3f> centroids;
		for(map<int,pcl::PointCloud<pcl::PointXYZI>>::iterator iter =planar_clouds.begin();iter!=planar_clouds.end();iter++)
		{
		  Eigen::Vector4f centroid;
		  pcl::compute3DCentroid(iter->second, centroid);
		  centroids.insert(pair<int,Eigen::Vector3f>(iter->first,Eigen::Vector3f(centroid[0],centroid[1],centroid[2])));
		}

		vector<int> data;
		for(map<int,Eigen::Vector4f>::iterator iter =models.begin();iter!=models.end();iter++)
		{
		   data.push_back(iter->first);
		}
		vector<vector<int>> valid_combination;
		if(data.size()>2)
		{
		  vector<vector<int>> combination =  combine(data,3);//得到所有的3个排列组合结果
		  for(int i=0;i<combination.size();i++)//遍历所有的三个平面的组合
		  {
		    Eigen::Vector3f a = Eigen::Vector3f(models[combination[i][0]][0],models[combination[i][0]][1],models[combination[i][0]][2]);
		    Eigen::Vector3f b = Eigen::Vector3f(models[combination[i][1]][0],models[combination[i][1]][1],models[combination[i][1]][2]);
		    Eigen::Vector3f c = Eigen::Vector3f(models[combination[i][2]][0],models[combination[i][2]][1],models[combination[i][2]][2]);
		    Eigen::Vector3f a_center = centroids[combination[i][0]];
		    Eigen::Vector3f b_center = centroids[combination[i][1]];
		    Eigen::Vector3f c_center = centroids[combination[i][2]];
		    if(a_center.adjoint()*a>0)
		      a = -1.0*a;
		    if(b_center.adjoint()*b>0)
		      b = -1.0*b;
		    if(c_center.adjoint()*c>0)
		      c = -1.0*c;
		
		    if((a.adjoint()*b<0.2)&&(a.adjoint()*c<0.2)&&(b.adjoint()*c<0.2))//对应81度
		    {
			int ground = combination[i][0];
			int left = combination[i][1];
			int right = combination[i][2];
			float min_z = a_center[2];
			if(min_z>b_center[2]) 
			{
			  ground = combination[i][1];
			  left = combination[i][0];
			  right = combination[i][2];
			  min_z = b_center[2];
			}
			if(min_z>c_center[2])
			{
			  ground = combination[i][2];
			  left = combination[i][0];
			  right = combination[i][1];
			}
			if(centroids[left][1]<centroids[right][1])
			{
			  int tmp = left;
			  left = right;
			  right = tmp;
			}
			
			//然后判断是否为凹平面关系
			Eigen::Vector3f left_center = centroids[left];
			Eigen::Vector3f right_center = centroids[right];
			Eigen::Vector3f left_normal = Eigen::Vector3f(models[left][0],models[left][1],models[left][2]);
			if(left_center.adjoint()*left_normal>0)
			  left_normal = -1.0*left_normal;
			Eigen::Vector3f right_normal = Eigen::Vector3f(models[right][0],models[right][1],models[right][2]);
			if(right_center.adjoint()*right_normal>0)
			  right_normal = -1.0*right_normal;
			
			if( ((right_center-left_center).adjoint()*left_normal>0)&&((left_center-right_center).adjoint()*right_normal>0) )
			{
			  vector<int> temp;
			  temp.push_back(right);
			  temp.push_back(left);
			  temp.push_back(ground);
			  valid_combination.push_back(temp);
			}
		    }
		  }
		}
		for(int i=0;i<valid_combination.size();i++)
		{
		  int indx_a = valid_combination[i][0];
		  int indx_b = valid_combination[i][1];
		  int indx_c = valid_combination[i][2];
		  cout<<"  "<<indx_a<<" "<<indx_b<<" "<<indx_c<<endl;
		  float offset = (i+1)*2;
		  for(int j=0;j<planar_clouds[indx_a].points.size();j++)
		  {
		    pcl::PointXYZI temp = planar_clouds[indx_a].points[j];
		    temp.z = temp.z+offset;
		    temp.intensity = indx_a;
		    test_cloud.push_back(temp);
		  }
		  for(int j=0;j<planar_clouds[indx_b].points.size();j++)
		  {
		    pcl::PointXYZI temp = planar_clouds[indx_b].points[j];
		    temp.z = temp.z+offset;
		    temp.intensity = indx_b;
		    test_cloud.push_back(temp);
		  }
		  for(int j=0;j<planar_clouds[indx_c].points.size();j++)
		  {
		    pcl::PointXYZI temp = planar_clouds[indx_c].points[j];
		    temp.z = temp.z+offset;
		    temp.intensity = indx_c;
		    test_cloud.push_back(temp);
		  }
		}
		
		if(valid_combination.size()==1)
		{
		    Eigen::Vector3f a = Eigen::Vector3f(models[valid_combination[0][0]][0],models[valid_combination[0][0]][1],models[valid_combination[0][0]][2]);
		    Eigen::Vector3f b = Eigen::Vector3f(models[valid_combination[0][1]][0],models[valid_combination[0][1]][1],models[valid_combination[0][1]][2]);
		    Eigen::Vector3f c = Eigen::Vector3f(models[valid_combination[0][2]][0],models[valid_combination[0][2]][1],models[valid_combination[0][2]][2]);
		    float a_D = models[valid_combination[0][0]][3];
		    float b_D = models[valid_combination[0][1]][3];
		    float c_D = models[valid_combination[0][2]][3];
		    
		    Eigen::Vector3f a_center = centroids[valid_combination[0][0]];
		    Eigen::Vector3f b_center = centroids[valid_combination[0][1]];
		    Eigen::Vector3f c_center = centroids[valid_combination[0][2]];
		    if(a_center.adjoint()*a>0)
		    {
		      a = -1.0*a;
		      a_D = -1.0*a_D;
		    }
		    if(b_center.adjoint()*b>0)
		    {
		      b = -1.0*b;
		      b_D = -1.0*b_D;
		    }
		    if(c_center.adjoint()*c>0)
		    {
		      c = -1.0*c;
		      c_D = -1.0*c_D;
		    }
		    Eigen::Vector4f right_plane,left_plane,ground_plane;
		    right_plane[0] = a[0];
		    right_plane[1] = a[1];
		    right_plane[2] = a[2];
		    right_plane[3] = a_D;
		    
		    left_plane[0] = b[0];
		    left_plane[1] = b[1];
		    left_plane[2] = b[2];
		    left_plane[3] = b_D;
		    
		    ground_plane[0] = c[0];
		    ground_plane[1] = c[1];
		    ground_plane[2] = c[2];
		    ground_plane[3] = c_D;
		    models.insert(pair<int,Eigen::Vector4f>(0,right_plane));
		    models.insert(pair<int,Eigen::Vector4f>(1,left_plane));
		    models.insert(pair<int,Eigen::Vector4f>(2,ground_plane));
		}
		else
		{
		  models.clear();
		}
                Publish_Topic();
		
		out_cloud = cloud;
		ResetParameters();
		return planar_clouds;
	  }//Obtain_SegementCloud函数结束
     
	
   };

}

#endif
