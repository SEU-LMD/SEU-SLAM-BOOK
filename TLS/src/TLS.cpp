#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <cmath>
#include<time.h>//用于生成随机数
#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构
#include <Eigen/QR> //用于矩阵求逆
#include <random>
/*
#include <g2o/core/base_vertex.h>//顶点的数据类型
#include <g2o/core/block_solver.h>//块求解器
#include <g2o/core/optimization_algorithm_levenberg.h>//非线性求解方法
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>//线性求解器的头文件
#include <g2o/solvers/dense/linear_solver_dense.h>//线性求解器的头文件
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>//定义了我们需要的边
#include <g2o/core/robust_kernel_impl.h>//用于设置鲁邦核函数
*/

#include <opencv2/core/core.hpp>//为了使用它的读取yaml文件函数
#include <opencv2/calib3d/calib3d.hpp>//这个函数的作用引入罗德里格斯公式和pnp的代码
#include <opencv/cv.h>   
#include<opencv2/opencv.hpp>

#include "sophus/so3.h"//李群李代数的库函数
#include "sophus/se3.h"

#include "ceres/ceres.h"//ceres的头文件
#include "ceres/rotation.h"

//下面是我们自己定义的头文件
#include "deal_string.h"
#include "icp.h"
#include "icpTLS.h"
#include "FileIO.h"
#define PI (3.1415926535897932346f)

using namespace std;
using namespace cv;
using namespace TLS;//我们自己定义的状态空间

//计算两个姿态的误差
void ICP_Error(double& error_R,double& error_t,Eigen::Matrix3d Ra,Eigen::Vector3d ta,Eigen::Matrix3d Rb,Eigen::Vector3d tb)
{
     Eigen::Matrix3d R_delta = Ra.transpose()*Rb;
     double d = 0.5*(R_delta(0,0)+R_delta(1,1)+R_delta(2,2)-1);
     error_R = acos( max( min(d,1.0),-1.0 ) );//输出范围位0 pi
     if(error_R<0)
     {
      cout<<"error!"<<endl;
     }
     error_t = (ta-tb).norm();
}
//计算得到icp的残差
double Residual_Error(Eigen::Matrix3d R,Eigen::Vector3d t,vector<Eigen::Vector3d> slam_points,vector<Eigen::Vector3d> gps_points)
{
  double sumerror=0;
  for(int i=0;i<slam_points.size();i++)
  {
    sumerror= sumerror+( R*slam_points[i]+t-gps_points[i] ).norm();
  }
  return sumerror;
}
//points没有噪声的点 a为噪声的幅度，
void add_Noise(vector<Eigen::Vector3d>& slampoints_,vector<Eigen::Vector3d>& gpspoints_,vector<double> a,string noisemodel = "Uniform")
{
  if(noisemodel == "Uniform")//平均分布 噪声的范围为-a到a
  {
      vector<double> noise_x;
      vector<double> noise_y;
      vector<double> noise_z;
      for(int i=0;i<3;i++)
      { 
	  int dis = 2*a[i];
	  for(int j=0;j<slampoints_.size()*2;j++)
	  {
	    double noise =  -a[i] + dis * (rand() / (RAND_MAX + 1.0));
	    if(i==0)
	    {
	      noise_x.push_back(noise);
	    }
	    else if(i==1)
	    {
	      noise_y.push_back(noise);
	    }
	    else
	      
	    {
	      noise_z.push_back(noise);
	    }
          }
       }
      for(int i=0;i<slampoints_.size();i++)
      {
	    double x_slam_noise = slampoints_[i][0]+noise_x[i];
	    double y_slam_noise = slampoints_[i][1]+noise_y[i];
	    double z_slam_noise = slampoints_[i][2]+noise_z[i];
	    slampoints_[i] = Eigen::Vector3d(x_slam_noise,y_slam_noise,z_slam_noise);
	    
	    double x_gps_noise = gpspoints_[i][0]+noise_x[2*i];
	    double y_gps_noise = gpspoints_[i][1]+noise_y[2*i];
	    double z_gps_noise = gpspoints_[i][2]+noise_z[2*i];
	    gpspoints_[i] = Eigen::Vector3d(x_gps_noise,y_gps_noise,z_gps_noise);
      }
  }
  else if(noisemodel == "Normal")//高斯分布 期望为0 方差为a
  {
    vector<double> noise_x;
    vector<double> noise_y;
    vector<double> noise_z;
    static std::random_device _randomdevice;
    static std::mt19937 _randomGen(_randomdevice());
   //std::default_random_engine generator;
    for(int i=0;i<3;i++)
    {
      std::normal_distribution<double> dist(0,a[i]);
      for(int j=0;j<slampoints_.size()*2;j++)
      {
	double noise = dist(_randomGen);
	if(i==0)
	{
	  noise_x.push_back(noise);
	}
	else if(i==1)
	{
	  noise_y.push_back(noise);
	}
	else
	  
	{
	  noise_z.push_back(noise);
	}
      }
    }
    for(int i=0;i<slampoints_.size();i++)
    {
      double x_slam_noise = slampoints_[i][0]+noise_x[i];
      double y_slam_noise = slampoints_[i][1]+noise_y[i];
      double z_slam_noise = slampoints_[i][2]+noise_z[i];
      slampoints_[i] = Eigen::Vector3d(x_slam_noise,y_slam_noise,z_slam_noise);
      
      double x_gps_noise = gpspoints_[i][0]+noise_x[2*i];
      double y_gps_noise = gpspoints_[i][1]+noise_y[2*i];
      double z_gps_noise = gpspoints_[i][2]+noise_z[2*i];
      gpspoints_[i] = Eigen::Vector3d(x_gps_noise,y_gps_noise,z_gps_noise);
    }
  }
  else if(noisemodel == "Own")
  {
    /*
     static std::random_device _randomdevice;
     static std::mt19937 _randomGen(_randomdevice());
     for(int i=0;i<points.size();i++)
    {
      std::normal_distribution<double> dist(double(a[i]),3);
      double x = dist(_randomGen);
      double y = dist(_randomGen);
      double z = dist(_randomGen);
      Eigen::Vector3d noise_point = points[i]+Eigen::Vector3d(x,y,z);//
      noise_points.push_back(noise_point);
    }*/
  }
}

//根据每一步迭代的TLS结果中搜索得到最优的那个结果，R_index表示旋转矩阵最优解的序号，t_index表示位置最优解的序号
//R_ls_error_和t_ls_error_表示最小二成的误差
//输出的序号是比最小二乘方法好的序号
vector<int>  Search_best_results(vector<Result> results_,Eigen::Matrix3d R_true_,Eigen::Vector3d t_true_,int& R_index,int& t_index,double R_ls_error_,double t_ls_error_)
{
  vector<int> better_results;
  R_index = 0;
  t_index = 0;
  double minR_error;
  double mint_error;
  double errorR_i;
  double errort_i;
  for(int i=0;i<results_.size();i++)
  {
    if(i==0)
    {
      ICP_Error(minR_error,mint_error,R_true_,t_true_,results_[i].R,results_[i].t);
    }
    else
    {
      ICP_Error(errorR_i,errort_i,R_true_,t_true_,results_[i].R,results_[i].t);
      if(minR_error>errorR_i)
      {
	minR_error = errorR_i;
	R_index = i;
      }
      if(mint_error>errort_i)
      {
	mint_error = errort_i;
	t_index = i;
      }
    }
    //判断是否两个指标都比最小二乘的方法好
    if((R_ls_error_>errorR_i)&&(t_ls_error_>errort_i))
    {
      better_results.push_back(i);
    }
  }
  return better_results;
}

int main(int argc, char **argv)
{
  string path = argv[1];
  vector<double> noiselevel;
  string noisekind;
  int iterations = 4;//总体最小二乘迭代多少次数
  string initial = "ID";//总体最小二乘使用什么初值 LS表示使用最小二乘的初值，ID表示使用单位矩阵
  //readnoise(path,initial,iterations,noiselevel,noisekind);
  vector<Eigen::Vector3d> slampoints,gpspoints;
  vector<Eigen::Vector3d> noise_slam_points,noise_gps_points;
  
   //这两个参数是选择哪个数据集，是否使用仿真数据
  bool simulateddata=true; 
  //数据来源于方兴老师论文中的数据集合
  for(int i=0;i<1;i++)
  {
    slampoints.push_back(Eigen::Vector3d(63,84,21));
    slampoints.push_back(Eigen::Vector3d(210,84,21));
    slampoints.push_back(Eigen::Vector3d(210,273,21));
    slampoints.push_back(Eigen::Vector3d(63,273,21)); 
    gpspoints.push_back(Eigen::Vector3d(290,150,15));
    gpspoints.push_back(Eigen::Vector3d(420,80,2));
    gpspoints.push_back(Eigen::Vector3d(540,200,20));
    gpspoints.push_back(Eigen::Vector3d(390,300,5));
  }
    
  noise_slam_points = slampoints;
  noise_gps_points = gpspoints;
  //我们先使用真实的数据计算
  Eigen::Matrix3d R_ls_realdata=Eigen::Matrix3d::Identity(),R_TLSLie_realdataa=Eigen::Matrix3d::Identity(),R_TLSAngleaxis_realdata=Eigen::Matrix3d::Identity(),R_TLSEuler_realdata = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_ls_realdata=Eigen::Vector3d(0,0,0),t_TLSLie_realddata=Eigen::Vector3d(0,0,0),t_TLSAngleaxis_realddata = Eigen::Vector3d(0,0,0),t_TLSEuler_realdata = Eigen::Vector3d(0,0,0);//最小二乘的结果
  pose_estimation_3d3d_quaeternion( eigenv3d_to_cvp3d(noise_gps_points), eigenv3d_to_cvp3d(noise_slam_points),R_ls_realdata,t_ls_realdata);//计算得到的是slam坐标系的点到gps坐标系下的变化
  Sophus::SE3 se3_ls_real(R_ls_realdata,t_ls_realdata);
  cout<<"ls 的李代数参数 = "<<se3_ls_real.log()<<endl;
  Sophus::SO3 so3_ls_real(R_ls_realdata);
  cout<<"ls 轴角参数 = "<<so3_ls_real.log()<<" 位置 = "<<t_ls_realdata<<endl;
  double residual_ls=Residual_Error(R_ls_realdata,t_ls_realdata,noise_slam_points,noise_gps_points);
  cout<<"LS残差 = "<<residual_ls<<endl;
  vector<double> weightSLAM_realdata,weightGPS_realdata;
  for(int i=0;i<3;i++)
  {
      weightSLAM_realdata.push_back(1.0);
      weightGPS_realdata.push_back(1.0);
  }
  TLS_ICP(R_TLSLie_realdataa,t_TLSLie_realddata,noise_slam_points,noise_gps_points,weightSLAM_realdata,weightGPS_realdata,iterations,"Lie");
  Sophus::SE3 se3_TLSLie_real(R_TLSLie_realdataa,t_TLSLie_realddata);
  cout<<"tls 的李代数参数 = "<<se3_TLSLie_real.log()<<endl;
  cout<<"基于李代数的残差 = "<<Residual_Error(R_TLSLie_realdataa,t_TLSLie_realddata,noise_slam_points,noise_gps_points)<<endl;
  
  TLS_ICP(R_TLSAngleaxis_realdata,t_TLSAngleaxis_realddata,noise_slam_points,noise_gps_points,weightSLAM_realdata,weightGPS_realdata,iterations,"AngleAxis");
  Sophus::SO3 so3_TLSangleaxis_real(R_TLSAngleaxis_realdata);
  cout<<"tls 轴角参数 = "<<so3_TLSangleaxis_real.log()<<" 位置 = "<<t_TLSAngleaxis_realddata<<endl;
  cout<<"基于轴角的残差 = "<<Residual_Error(R_TLSAngleaxis_realdata,t_TLSAngleaxis_realddata,noise_slam_points,noise_gps_points)<<endl;

  
  TLS_ICP(R_TLSEuler_realdata,t_TLSEuler_realdata,noise_slam_points,noise_gps_points,weightSLAM_realdata,weightGPS_realdata,iterations,"Euler");
  Eigen::Vector3d eulerangle = R_TLSEuler_realdata.eulerAngles(2,1,0);
  cout<<"tls 欧拉角参数 = "<<eulerangle<<" 位置 = "<<t_TLSEuler_realdata<<endl;
  cout<<"基于Euler的残差 = "<<Residual_Error(R_TLSEuler_realdata,t_TLSEuler_realdata,noise_slam_points,noise_gps_points)<<endl;
  
  Eigen::Vector3d t_true;
  Eigen::Matrix3d R_true;
  vector<Eigen::Vector3d> gpspoints_true;//根据我们的slam坐标和设定的R和t求得的
  if(simulateddata==true)
  {
      R_true = Eigen::AngleAxisd(PI/4,Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(PI/2,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(PI/3,Eigen::Vector3d::UnitX());
      t_true = Eigen::Vector3d(190,110,-15);
     
      cout<<"R 真值 = "<<endl;
      cout<<R_true<<endl;
      cout<<"t 真值 = "<<endl;
      cout<<t_true<<endl;
      //noise_slam_points= add_Noise(slampoints,slamnoiselevel,noisekind);//Uniform-平均分布 Normal-高斯分布
      for(int i=0;i<slampoints.size();i++)
      {
	gpspoints_true.push_back( R_true*slampoints[i]+t_true );
      }
  }
  
  srand((unsigned)time(NULL));
  for(int k=0;k<8;k++)//针对8种不同的噪声
  {
      noiselevel.clear();
      switch (k)
      {
	case 0:
	   noiselevel.push_back(0.1);noiselevel.push_back(0.1);noiselevel.push_back(0.1);
	   noisekind="Normal";
	   break;
	case 1:
	  noiselevel.push_back(0.1);noiselevel.push_back(0.5);noiselevel.push_back(1);
	  noisekind="Normal";
	  break;
	case 2:
	  noiselevel.push_back(0.1);noiselevel.push_back(0.5);noiselevel.push_back(0.5);
	  noisekind="Normal";
	  break;
	case 3:
	  noiselevel.push_back(0.1);noiselevel.push_back(1);noiselevel.push_back(1);
	  noisekind="Normal";
	  break;
	case 4:
	  noiselevel.push_back(5);noiselevel.push_back(5);noiselevel.push_back(5);
	  noisekind="Uniform";
	  break;
	case 5:
	  noiselevel.push_back(5);noiselevel.push_back(10);noiselevel.push_back(20);
	  noisekind="Uniform";
	  break;
	case 6:
	  noiselevel.push_back(5);noiselevel.push_back(10);noiselevel.push_back(10);
	  noisekind="Uniform";
	  break;
	case 7:
	  noiselevel.push_back(5);noiselevel.push_back(20);noiselevel.push_back(20);
	  noisekind="Uniform";
	  break;
      }
      double avg_ls_t=0,avg_ls_R=0,avg_Lie_t=0,avg_Lie_R=0,avg_Angleaxis_t=0,avg_Angleaxis_R=0;
      int valid_count=0;
      for(int p=0;p<50;p++)
      {
	  noise_gps_points =gpspoints_true;
	  noise_slam_points = slampoints;
          add_Noise(noise_slam_points,noise_gps_points,noiselevel,noisekind);
	      
	      cout<<"不带有噪声的slam点"<<endl;
	      for(int i=0;i<slampoints.size();i++)
	      {
		cout<<slampoints[i][0]<<" "<<slampoints[i][1]<<" "<<slampoints[i][2]<<endl;
	      }
	      cout<<"带有噪声的slam点 = "<<endl;
	      for(int i=0;i<noise_slam_points.size();i++)
	      {
		cout<<noise_slam_points[i][0]<<" "<<noise_slam_points[i][1]<<" "<<noise_slam_points[i][2]<<endl;
	      }
	      cout<<"不带有噪声的GPS点 = "<<endl;
	      for(int i=0;i<gpspoints_true.size();i++)
	      {
		cout<<gpspoints_true[i][0]<<" "<<gpspoints_true[i][1]<<" "<<gpspoints_true[i][2]<<" "<<endl;
	      }
	      cout<<"带有噪声的gps点 = "<<endl;
	      for(int i=0;i<noise_gps_points.size();i++)
	      {
		cout<<noise_gps_points[i][0]<<" "<<noise_gps_points[i][1]<<" "<<noise_gps_points[i][2]<<endl;
	      }
	      
	    
	    
		      //1.计算初值
		      Eigen::Matrix3d R_ini = Eigen::Matrix3d::Identity();
		      Eigen::Vector3d t_ini = Eigen::Vector3d(0,0,0);
		      double error_R_ini,error_t_ini,residual_ini;
		      residual_ini=Residual_Error(R_ini,t_ini,noise_slam_points,noise_gps_points);
		      if(simulateddata)
			ICP_Error(error_R_ini,error_t_ini,R_true,t_true,R_ini,t_ini);
		      
		      //2.最小二乘的结果
		      Eigen::Matrix3d R_ls;
		      Eigen::Vector3d t_ls;//最小二乘的结果
		      pose_estimation_3d3d_quaeternion( eigenv3d_to_cvp3d(noise_gps_points), eigenv3d_to_cvp3d(noise_slam_points),R_ls,t_ls);//计算得到的是slam坐标系的点到gps坐标系下的变化
		      double error_R_ls,error_t_ls,residual_ls;
		      residual_ls=Residual_Error(R_ls,t_ls,noise_slam_points,noise_gps_points);
		      if(simulateddata)
			ICP_Error(error_R_ls,error_t_ls,R_true,t_true,R_ls,t_ls);
		      
		      //3.整体最小二乘
		      vector<double> weightSLAM,weightGPS;
		      for(int i=0;i<slampoints.size();i++)
		      {
			if(noisekind=="Uniform")
			{
			  weightGPS.push_back(1.0);
			  weightSLAM.push_back(1.0);
			}
			else if(noisekind=="Normal")
			{
			  weightGPS.push_back(1/double(noiselevel[i]));
			  weightSLAM.push_back(1.0/double(noiselevel[i]));
			  //weightGPS.push_back(1.0);
			  //weightSLAM.push_back(1.0);
			}
		      }
		      
		      //3.1使用欧拉角参数话的整体最小二乘
		      //cout<<"欧拉角总体最小二乘--------------------------------------------"<<endl;
		      double error_R_Euler,error_t_Euler;
		      vector<Result> Euler_results;
		      Eigen::Vector3d t_Euler;
		      Eigen::Matrix3d R_Euler;
		      if(initial=="ID")
		      {
			t_Euler = t_ini;
			R_Euler = R_ini;
		      }
		      else if(initial == "LS")
		      {
			t_Euler = t_ls;
			R_Euler = R_ls;
		      }
		      Euler_results = TLS_ICP(R_Euler,t_Euler,noise_slam_points,noise_gps_points,weightSLAM,weightGPS,iterations,"Euler");
		      int Euler_R_index,Euler_t_index;
		      vector<int> betterresults_Euler = Search_best_results(Euler_results,R_true,t_true,Euler_R_index,Euler_t_index,error_R_ls,error_t_ls);
		      //residual_Euler=Residual_Error(R_Euler,t_Euler,noise_slam_points,noise_gps_points);
		      double error_R_Euler1,error_t_Euler1,error_R_Euler2,error_t_Euler2;
		      if(simulateddata)
		      {
			  ICP_Error(error_R_Euler1,error_t_Euler1,R_true,t_true,Euler_results[Euler_R_index].R,Euler_results[Euler_R_index].t);
			  ICP_Error(error_R_Euler2,error_t_Euler2,R_true,t_true,Euler_results[Euler_t_index].R,Euler_results[Euler_t_index].t);
		      }
		      
		      //3.2使用轴角参数话的整体最小二乘
		      //cout<<"轴角总体最小二乘--------------------------------------------"<<endl;
		      
		      vector<Result> AngleAxis_results;
		      Eigen::Vector3d t_angleaxis;
		      Eigen::Matrix3d R_angelaxis;
		      if(initial=="ID")
		      {
			t_angleaxis = t_ini;
			R_angelaxis = R_ini;
		      }
		      else if(initial == "LS")
		      {
			t_angleaxis = t_ls;
			R_angelaxis = R_ls;
		      }
		      AngleAxis_results = TLS_ICP(R_angelaxis,t_angleaxis,noise_slam_points,noise_gps_points,weightSLAM,weightGPS,iterations,"AngleAxis");
		      int Angleaxis_R_index,Angleaxis_t_index;
		      vector<int> betterresults_Angleaxis =Search_best_results(AngleAxis_results,R_true,t_true,Angleaxis_R_index,Angleaxis_t_index,error_R_ls,error_t_ls);
		      //residual_angleaxis=Residual_Error(R_angelaxis,t_angleaxis,noise_slam_points,noise_gps_points);
		      double error_R_angleaxis1,error_t_angleaxis1,error_R_angleaxis2,error_t_angleaxis2;
		      if(simulateddata)
		      {
			ICP_Error(error_R_angleaxis1,error_t_angleaxis1,R_true,t_true,AngleAxis_results[Angleaxis_R_index].R,AngleAxis_results[Angleaxis_R_index].t);
			ICP_Error(error_R_angleaxis2,error_t_angleaxis2,R_true,t_true,AngleAxis_results[Angleaxis_t_index].R,AngleAxis_results[Angleaxis_t_index].t);
		      }

		      //3.3使用李代数参数化的整体最小二乘
		      //cout<<"李代数总体最小二乘--------------------------------------------"<<endl;
		      
		      vector<Result> Lie_results;
		      Eigen::Vector3d t_Lie;
		      Eigen::Matrix3d R_Lie; 
		      if(initial=="ID")
		      {
			t_Lie = t_ini;
			R_Lie = R_ini;
		      }
		      else if(initial == "LS")
		      {
			t_Lie = t_ls;
			R_Lie = R_ls;
		      }
		      Lie_results = TLS_ICP(R_Lie,t_Lie,noise_slam_points,noise_gps_points,weightSLAM,weightGPS,iterations,"Lie");
		      //residual_Lie=Residual_Error(R_Lie,t_Lie,noise_slam_points,noise_gps_points);
		      int Lie_R_index,Lie_t_index;
		      vector<int> betterresults_Lie = Search_best_results(Lie_results,R_true,t_true,Lie_R_index,Lie_t_index,error_R_ls,error_t_ls);
		      double error_R_Lie1,error_t_Lie1,error_R_Lie2,error_t_Lie2;
		      if(simulateddata)
		      {
			ICP_Error(error_R_Lie1,error_t_Lie1,R_true,t_true,Lie_results[Lie_R_index].R,Lie_results[Lie_R_index].t);
			ICP_Error(error_R_Lie2,error_t_Lie2,R_true,t_true,Lie_results[Lie_t_index].R,Lie_results[Lie_t_index].t);
		      }
		      
		      
		     //下面是输出
		    
		      cout<<initial<<" 初始值结果 = "<<endl;
		      cout<<"初始姿态误差 = "<<error_R_ini<<" 初始位置误差 = "<<error_t_ini<<" 残差= "<<residual_ini<<endl;
		      cout<<"ls姿态误差 = "<<error_R_ls<<" ls位置误差 = "<<error_t_ls<<" 残差= "<<residual_ls<<endl;
		      cout<<endl;
		       /*cout<<"轴角最优R解序号 = "<<Angleaxis_R_index<<" 轴角最优解t序号 = "<<Angleaxis_t_index<<endl;
		      cout<<"R序号姿态误差 = "<<error_R_angleaxis1<<" R序号位置误差 = "<<error_t_angleaxis1<<endl;
		      cout<<"t序号姿态误差 = "<<error_R_angleaxis2<<" t序号位置误差 = "<<error_t_angleaxis2<<endl;*/
		      double first_betterresult_angleaxiserror_R, first_betterresult_angleaxiserror_t;
		      for(int i=0;i<betterresults_Angleaxis.size();i++)
		      {
			double error_R,error_t;
			ICP_Error(error_R,error_t,R_true,t_true,AngleAxis_results[betterresults_Angleaxis[i]].R,AngleAxis_results[betterresults_Angleaxis[i]].t);
			cout<<"比ls方法好的结果 = "<<betterresults_Angleaxis[i]<<" 姿态误差 = "<<error_R<<" 位置误差 = "<<error_t<<" 残差 = "<<AngleAxis_results[betterresults_Angleaxis[i]].equnorm<<endl;
		        if(i==0)
			{
			  first_betterresult_angleaxiserror_R = error_R;
			  first_betterresult_angleaxiserror_t = error_t;
			  cout<<"轴角比ls方法好的结果 = "<<betterresults_Angleaxis[0]<<" 姿态误差 = "<<error_R<<" 位置误差 = "<<error_t<<" 残差 = "<<AngleAxis_results[betterresults_Angleaxis[0]].equnorm<<endl;
			}			
		      }
		      //cout<<endl;
		      
		      //cout<<"李代数最优R解序号 = "<<Lie_R_index<<" 轴角最优解t序号 = "<<Lie_t_index<<endl;
		      //cout<<"R序号姿态误差 = "<<error_R_Lie1<<" R序号位位置误差 = "<<error_t_Lie1<<endl;
		      //cout<<"t序号姿态误差 = "<<error_R_Lie2<<" t序号位位置误差 = "<<error_t_Lie2<<endl;
		      double first_betterresult_Lieerror_R, first_betterresult_Lieerror_t;
		      for(int i=0;i<betterresults_Lie.size();i++)
		      {
			double error_R,error_t;
			ICP_Error(error_R,error_t,R_true,t_true,Lie_results[betterresults_Lie[i]].R,Lie_results[betterresults_Lie[i]].t);
			cout<<"比ls方法好的结果 = "<<betterresults_Lie[i]<<" 姿态误差 = "<<error_R<<" 位置误差 = "<<error_t<<" 残差 = "<<Lie_results[betterresults_Lie[i]].equnorm<<endl;
			if(i==0)
			{
			  first_betterresult_Lieerror_R = error_R;
			  first_betterresult_Lieerror_t = error_t;
			  cout<<"Lie比ls方法好的结果 = "<<betterresults_Lie[0]<<" 姿态误差 = "<<error_R<<" 位置误差 = "<<error_t<<" 残差 = "<<Lie_results[betterresults_Lie[0]].equnorm<<endl;
			}	
		      }
		      //cout<<endl;
		      
		      
		      if((betterresults_Angleaxis.size()>0)&&(betterresults_Lie.size()>0))
		      {
			valid_count++;
			avg_ls_t = avg_ls_t+error_t_ls;
			avg_ls_R = avg_ls_R+error_R_ls;
			avg_Angleaxis_R = avg_Angleaxis_R + first_betterresult_angleaxiserror_R;
			avg_Angleaxis_t = avg_Angleaxis_t + first_betterresult_angleaxiserror_t;
			avg_Lie_R = avg_Lie_R+first_betterresult_Lieerror_R;
			avg_Lie_t = avg_Lie_t+first_betterresult_Lieerror_t;
		      }
		      
		      //cout<<"欧拉角最优R解序号 = "<<Euler_R_index<<" 轴角最优解t序号 = "<<Euler_t_index<<endl;
		      //cout<<"R序号姿态误差 = "<<error_R_Euler1<<" R序号位置误差 = "<<error_t_Euler1<<endl;
		      //cout<<"t序号姿态误差 = "<<error_R_Euler2<<" t序号位置误差 = "<<error_t_Euler2<<endl;
		      for(int i=0;i<betterresults_Euler.size();i++)
		      {
			double error_R,error_t;
			ICP_Error(error_R,error_t,R_true,t_true,Euler_results[betterresults_Euler[i]].R,Euler_results[betterresults_Euler[i]].t);
			//cout<<"比ls方法好的结果 = "<<betterresults_Euler[i]<<" 姿态误差 = "<<error_R<<" 位置误差 = "<<error_t<<" 残差 = "<<Euler_results[betterresults_Euler[i]].equnorm<<endl;
		      }
      }//p循环结束
      avg_ls_t = avg_ls_t/valid_count;
      avg_ls_R = avg_ls_R/valid_count;
      avg_Angleaxis_R = avg_Angleaxis_R/valid_count;
      avg_Angleaxis_t = avg_Angleaxis_t/valid_count;
      avg_Lie_R = avg_Lie_R/valid_count;
      avg_Lie_t = avg_Lie_t/valid_count;
      cout<<"误差类型 = "<<k<<" 有效数 = "<<valid_count<<endl;
      cout<<"ls姿态平均误差 = "<<avg_ls_R<<" 轴角姿态平均误差 = "<<avg_Angleaxis_R<<" Lie姿态平均误差 = "<<avg_Lie_R<<endl;
      cout<<"ls位置平均误差 = "<<avg_ls_t<<" 轴角位置平均误差 = "<<avg_Angleaxis_t<<" Lie位置平均误差 = "<<avg_Lie_t<<endl;
      cout<<"--------------------------"<<endl;
  }//k循环结束
}
