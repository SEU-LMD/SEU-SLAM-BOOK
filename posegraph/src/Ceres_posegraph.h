#ifndef _CeresPosegraph_
#define _CeresPosegraph_

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include "ceres/ceres.h"//ceres的头文件
#include "ceres/autodiff_cost_function.h"

#include "types.h"

using namespace std;
using namespace ceres;
//本文件主要是实现ceres进行位姿图优化
namespace posegraph
{
  typedef std::map<int,pose> MapOfPoses;
  
  //指定自己的cost function 我们从cerse的例子中复制来的 这里对posegraph优化参数是t和四元数
  class PoseGraph3dErrorTerm 
  {
      public:
      PoseGraph3dErrorTerm(const edge& t_ab_measured, const Eigen::Matrix<double, 6, 6>& sqrt_information): t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}
      
      //这个函数是对残差进行定义的
      template <typename T>
      bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,const T* const p_b_ptr, const T* const q_b_ptr,T* residuals_ptr) const 
      {
	Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);//3指的行数，1指的列数
	Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);

	Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_b(p_b_ptr);
	Eigen::Map<const Eigen::Quaternion<T> > q_b(q_b_ptr);

	// Compute the relative transformation between the two frames.
	Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
	Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

	// Represent the displacement between the two frames in the A frame.
	Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

	// Compute the error between the two orientation estimates.
	Eigen::Quaternion<T> delta_q = t_ab_measured_.delta_q.template cast<T>() * q_ab_estimated.conjugate();

	// Compute the residuals.
	// [ position         ]   [ delta_p          ]
	// [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
	Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
	residuals.template block<3, 1>(0, 0) = p_ab_estimated - t_ab_measured_.delta_t.template cast<T>();
	residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();//vec表示返回四元数的虚部

	// Scale the residuals by the measurement uncertainty.
	    //applyOnTheLeft是Eigen中
	residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

	return true;
      }

      static ceres::CostFunction* Create(const edge& t_ab_measured,const Eigen::Matrix<double, 6, 6>& sqrt_information)
      {
	    //6=残差的维度 输入的参数的维度：3=位置 4=姿态 3=位置 4=姿态
	return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      // The measurement for the position of B relative to A in the A frame.
      const edge t_ab_measured_;
      // The square root of the measurement information matrix.
      const Eigen::Matrix<double, 6, 6> sqrt_information_;
 };
  
  //主要的优化函数
  vector<pose> Ceres_posegraph(vector<pose> poses, vector<edge> edges_vo,string method ="DL",int iterations = 20)
  {
	MapOfPoses mapofposes;
	for(int i=0;i<poses.size();i++)
	{
	  pose posei;
	  posei.id = poses[i].id;
	  posei.t = poses[i].t;
	  posei.q = poses[i].q;
	  mapofposes.insert(pair<int,pose>(poses[i].id,posei));
	}
	
	//1.构造问题
	ceres::Problem problem;
	ceres::LossFunction* loss_function = NULL;//设置loss function
	ceres::LocalParameterization* quaternion_local_parameterization = new EigenQuaternionParameterization;
	const Eigen::Matrix<double,6,6> sqrt_information = Eigen::Matrix<double,6,6>::Identity();
	for(int i=0;i<edges_vo.size();i++)
	{
	    int id1 = edges_vo[i].id1;
	    int id2 = edges_vo[i].id2;
	    MapOfPoses::iterator pose1_iter = mapofposes.find(id1);
	    MapOfPoses::iterator pose2_iter = mapofposes.find(id2);
	    ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(edges_vo[i],sqrt_information);
	    problem.AddResidualBlock(cost_function,loss_function,
				    pose1_iter->second.t.data(),pose1_iter->second.q.coeffs().data(),
				    pose2_iter->second.t.data(),pose2_iter->second.q.coeffs().data()
				    );
	    problem.SetParameterization(pose1_iter->second.q.coeffs().data(),quaternion_local_parameterization);//设置参数化
	    problem.SetParameterization(pose2_iter->second.q.coeffs().data(),quaternion_local_parameterization);
      }
      //2.把第一个位姿设为常数
      MapOfPoses::iterator first_pose_iter = mapofposes.begin();
      problem.SetParameterBlockConstant(first_pose_iter->second.t.data());
      problem.SetParameterBlockConstant(first_pose_iter->second.q.coeffs().data());
      //3.设置优化的参数
      ceres::Solver::Options options;
      options.max_num_iterations=iterations; 
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;//DENSE_QR、SPARSE_NORMAL_CHOLESKY、DENSE_NORMAL_CHOLESKY、SPARSE_SCHUR、DENSE_SCHUR
      options.minimizer_progress_to_stdout = true;
      
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
      cout<<summary.FullReport()<<endl;
      //5.将我们得到的结果转成我们的格式
      vector<pose> optimized_poses;
      for(std::map<int,pose>::const_iterator pose_iter=mapofposes.begin();pose_iter!=mapofposes.end();pose_iter++)
      {
	Eigen::Vector3d ti = pose_iter->second.t;
	Eigen::Quaterniond qi = pose_iter->second.q;
	int id = pose_iter->second.id;
	pose posei;
	posei.id = id;
	posei.t = ti;
	posei.q = qi;
	optimized_poses.push_back(posei);
      }
      return optimized_poses;
  }
}
#endif