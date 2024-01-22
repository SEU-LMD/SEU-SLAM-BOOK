#ifndef _FYYCERES_
#define _FYYCERES_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ceres/ceres.h"//ceres的头文件

using namespace std;
namespace CameraCalibration3D
{
  //用于计算单应性优化的参差定义
  //这个函数的作用是已知两点距离，已知一个点的坐标，优化另外一个点
  struct HOMO_COST
  {
      //输入的p是gps坐标，dist是距已经计算得到的slam距离
      HOMO_COST(Eigen::Vector3d p3d,Eigen::Vector2d p2d):_p3d(p3d),_p2d(p2d){}
      
      template<typename T>
      bool operator()(const T* const homomatrix_ptr,T* residual)const
      {
	Eigen::Map<const Eigen::Matrix<T, 3, 4> > homomatrix(homomatrix_ptr);//3指的行数，1指的列数
	//Eigen::Matrix<T,3,1> p3 = _p3d.template cast<T>();
	//Eigen::Matrix<T,2,1> p2 = _p2d.template cast<T>();
	residual[0] = T(_p3d(0))*homomatrix(0,0)+T(_p3d(1))*homomatrix(0,1)+T(_p3d(2))*homomatrix(0,2)+homomatrix(0,3)-T(_p2d(0))*T(_p3d(0))*homomatrix(2,0)-T(_p2d(0))*T(_p3d(1))*homomatrix(2,1)-T(_p2d(0))*T(_p3d(2))*homomatrix(2,2)-T(_p2d(0));
	residual[1] = T(_p3d(0))*homomatrix(1,0)+T(_p3d(1))*homomatrix(1,1)+T(_p3d(2))*homomatrix(1,2)+homomatrix(1,3)-T(_p2d(1))*T(_p3d(0))*homomatrix(2,0)-T(_p2d(1))*T(_p3d(1))*homomatrix(2,1)-T(_p2d(1))*T(_p3d(2))*homomatrix(2,2)-T(_p2d(1));
	return true;
      }
      Eigen::Vector3d _p3d;
      Eigen::Vector2d _p2d;
  };
  //这个函数是用于优化单应性矩阵
  Eigen::Matrix<double,3,4> optimize_homogo(Eigen::Matrix<double,3,4> Tinit,vector<Eigen::Vector3d> points3d,vector<Eigen::Vector2d> points2d,int iterations = 10,string method ="LM")
  {
      ceres::Problem problem;
      for(int i=0;i<points3d.size();i++)
      {
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HOMO_COST,2,11>(new HOMO_COST(points3d[i],points2d[i])),nullptr,Tinit.data());
      }
      ceres::Solver::Options options;
      options.max_num_iterations=iterations; 
      options.linear_solver_type = ceres::DENSE_QR;//DENSE_QR、SPARSE_NORMAL_CHOLESKY、DENSE_NORMAL_CHOLESKY、SPARSE_SCHUR、DENSE_SCHUR
      options.minimizer_progress_to_stdout = false;
      
      //默认使用的是lm优化算法 和 trust region
      if(method=="GN")
      {
	options.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION
	options.line_search_type = ceres::WOLFE;
	options.line_search_direction_type = ceres::STEEPEST_DESCENT;//这里应该是gaussnewton方法
      }
      else if(method=="DL")
      {
	options.minimizer_type = ceres::TRUST_REGION;//ceres::TRUST_REGION
	options.trust_region_strategy_type = ceres::DOGLEG;//LEVENBERG_MARQUARDT
      }
      else if(method=="LM")
      {
	options.minimizer_type = ceres::TRUST_REGION;//ceres::TRUST_REGION
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;//LEVENBERG_MARQUARDT
      }
      
      ceres::Solver::Summary summary;
      //4.开始优化
      ceres::Solve(options,&problem,&summary);
      //cout<<summary.FullReport()<<endl;
      return Tinit;
  }
}
#endif