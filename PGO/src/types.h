#ifndef _TYPES_
#define _TYPES_
#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

using namespace std;
//这个文件的作用是读取文件到程序中
namespace posegraph
{
  struct edge 
  {
	int id1;
	int id2;
	Eigen::Vector3d delta_t;
	Eigen::Quaterniond delta_q;
        Eigen::Matrix<double,6,6> cov;//输入的协方差矩阵
        double avg_error;
	int indexofloop;
  };
  struct ERRORS
  {
    double i_sum_R;
    double i_sum_t;
    double opt_sum_R;
    double opt_sum_t;
    
    double i_avg_R;
    double i_avg_t;
    double opt_avg_R;
    double opt_avg_t;
  };
  /*
  struct pose {
	int id;
	Eigen::Vector3d t;
	Eigen::Quaterniond q;//存储姿态的四元数
	};*/
  class pose
  {
    public:
	int id;
	Eigen::Vector3d t;
	Eigen::Quaterniond q;//存储姿态的四元数
	
	pose(){}
	pose(Eigen::Vector3d t_,Eigen::Quaterniond q_,int id_=0)
	{
	  t = t_;
	  q = q_;
	  id = id_;
	}
	pose inverse() const
	{
	  pose result;
	  Eigen::Vector3d t1 = this->t;
	  Eigen::Quaterniond q1 = this->q;
	  result.q = q1.toRotationMatrix().transpose();
	  result.t = -q1.toRotationMatrix().transpose()*t1;
	  return result;
	}
	pose operator*(const pose &T2) const
	{
	  pose result;
	  Eigen::Vector3d t1 = this->t;
	  Eigen::Quaterniond q1 = this->q;
	  Eigen::Vector3d t2 = T2.t;
	  Eigen::Quaterniond q2 = T2.q;
	  result.q =q1.toRotationMatrix()*q2.toRotationMatrix();
	  result.t = q1.toRotationMatrix()*t2+t1;
	  return result;
	}
  };
	
  //下面的结构是用
}
#endif