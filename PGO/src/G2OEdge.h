#ifndef _G2oEdge_
#define _G2oEdge_
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>//块求解器
#include <g2o/core/optimization_algorithm_levenberg.h>//非线性求解方法
#include <g2o/core/optimization_algorithm_dogleg.h>//dogleg算法
#include <g2o/core/optimization_algorithm_gauss_newton.h>//高斯牛顿算法
#include <g2o/solvers/eigen/linear_solver_eigen.h>//线性求解器的头文件
#include <g2o/solvers/dense/linear_solver_dense.h>//线性求解器的头文件
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>//线性求解器的头文件
#include <g2o/types/sba/types_six_dof_expmap.h>//定义了我们需要的边
#include <g2o/core/robust_kernel_impl.h>//用于设置鲁邦核函数

#include <sophus/se3.h>
#include <sophus/so3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构
using namespace std;
using namespace Sophus;
//这个文件的作用是定义我们需要的边和顶点
namespace posegraph
{

    //在下面计算边的时候会用到
    Eigen::Matrix3d Hat(Eigen::Vector3d v)
    {
      Eigen::Matrix3d result;
      result<<0,-v(2),v(1),
	      v(2),0,-v(0),
	      -v(1),v(0),0;
      return result;
    }
    //矩阵赋值 输入的是三个3*3矩阵按照顺序合成一个3*9矩阵
    Eigen::Matrix<double,9,3> Assignment(Eigen::Matrix3d A,Eigen::Matrix3d B, Eigen::Matrix3d C)
    {
      Eigen::Matrix<double,9,3> R;
      R(0,0)=A(0,0);R(0,1)=A(0,1);R(0,2)=A(0,2);
      R(1,0)=A(1,0);R(1,1)=A(1,1);R(1,2)=A(1,2);
      R(2,0)=A(2,0);R(2,1)=A(2,1);R(2,2)=A(2,2);
      
      R(3,0)=B(0,0);R(3,1)=B(0,1);R(3,2)=B(0,2);
      R(4,0)=B(1,0);R(4,1)=B(1,1);R(4,2)=B(1,2);
      R(5,0)=B(2,0);R(5,1)=B(2,1);R(5,2)=B(2,2);
      
      R(6,0)=C(0,0);R(6,1)=C(0,1);R(6,2)=C(0,2);
      R(7,0)=C(1,0);R(7,1)=C(1,1);R(7,2)=C(1,2);
      R(8,0)=C(2,0);R(8,1)=C(2,1);R(8,2)=C(2,2);
      return R;
    }
    
    //R矩阵在g2o中的顶点 我们使用的是左乘更新
    class VertexSO3LieAlgebra: public g2o::BaseVertex<3, Sophus::SO3>
    {
      public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	   bool read ( istream& is ){}
	   bool write ( ostream& os ) const {}
	  
	  virtual void setToOriginImpl()
	  {
	      _estimate = Sophus::SO3();
	  }
	  // 左乘更新
	  virtual void oplusImpl ( const double* update )
	  {
	      Sophus::SO3 up =  Sophus::SO3 ( Sophus::SO3::exp(Eigen::Vector3d(update[0], update[1], update[2])) );
	      _estimate = up*_estimate;
	  }
    };
    //误差函数定义:||(R1*△R-R2)*b|| 其中△R是测量值
    class EdgeSO3Base: public g2o::BaseBinaryEdge<9, Sophus::SO3, VertexSO3LieAlgebra, VertexSO3LieAlgebra>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
	
	// 误差的维度是9*1
	virtual void computeError()
	{
	    Sophus::SO3 v1 = (static_cast<VertexSO3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SO3 v2 = (static_cast<VertexSO3LieAlgebra*> (_vertices[1]))->estimate();
	    Eigen::Vector3d errorx(1,0,0);
	    Eigen::Vector3d errory(0,1,0);
	    Eigen::Vector3d errorz(0,0,1);
	    errorx = ( (v1*_measurement).matrix()-v2.matrix() )*errorx;
	    errory = ( (v1*_measurement).matrix()-v2.matrix() )*errory;
	    errorz = ( (v1*_measurement).matrix()-v2.matrix() )*errorz;
	    Eigen::Matrix<double,9,1> totalerror;
	    totalerror<<errorx(0),errorx(1),errorx(2),errory(0),errory(1),errory(2),errorz(0),errorz(1),errorz(2);
	    _error =totalerror;
	}
	
	// 雅可比计算 维度是9*3
	virtual void linearizeOplus()
	{
	    Sophus::SO3 v1 = (static_cast<VertexSO3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SO3 v2 = (static_cast<VertexSO3LieAlgebra*> (_vertices[1]))->estimate();
	    Eigen::Vector3d bx(1,0,0);
	    Eigen::Vector3d by(0,1,0);
	    Eigen::Vector3d bz(0,0,1);
	    Eigen::Matrix3d Jx_i = -Hat(v1* _measurement*bx);
	    Eigen::Matrix3d Jy_i = -Hat(v1* _measurement*by);
	    Eigen::Matrix3d Jz_i = -Hat(v1* _measurement*bz);
	    
	    Eigen::Matrix3d Jx_j = Hat(v2*bx);
	    Eigen::Matrix3d Jy_j = Hat(v2*by);
	    Eigen::Matrix3d Jz_j = Hat(v2*bz);
	    
	    Eigen::Matrix<double,9,3> Jxi,Jxj;
	    Jxi = Assignment(Jx_i,Jy_i,Jz_i);
	    Jxj = Assignment(Jx_j,Jy_j,Jz_j);
	    _jacobianOplusXi = Jxi;
	    _jacobianOplusXj = Jxj;
	}
	 //生成cost function的残差
      static double Produce_Error(vector<pose> poses,vector<edge> edges)
      {
	double sum_error = 0;
	for(int i=0;i<edges.size();i++)
	{
	  int id1 = edges[i].id1;
	  int id2 = edges[i].id2;
	  Eigen::Matrix3d R1 = poses[id1].q.toRotationMatrix();
	  Eigen::Matrix3d R2 = poses[id2].q.toRotationMatrix();
	  Eigen::Matrix3d R_mea = edges[i].delta_q.toRotationMatrix();
	  Eigen::Vector3d errorx(1,0,0);
	  Eigen::Vector3d errory(0,1,0);
	  Eigen::Vector3d errorz(0,0,1);
	  errorx = ( R1*R_mea-R2 )*errorx;
	  errory = ( R1*R_mea-R2 )*errory;
	  errorz = ( R1*R_mea-R2 )*errorz;
	  Eigen::Matrix<double,9,1> totalerror;
	  double error=0;
	  error = pow(errorx(0),2) + pow(errorx(1),2) + pow(errorx(2),2) + pow(errory(0),2) + pow(errory(1),2) + pow(errory(2),2) + pow(errorz(0),2) + pow(errorz(1),2) + pow(errorz(2),2);
	  sum_error =sum_error + error;
	}
	return sum_error;
      }
      static Eigen::Matrix<double,9,9> InformationMatrix()
	{
	  Eigen::Matrix<double,9,9> info;
	  info = Eigen::Matrix<double,9,9>::Identity();
	  return info;
	}
    };
   
    
    //求得Jr.inverse 用于下面的边 这里使用的是gtsam中的logmapderivative代码
    Eigen::Matrix3d JRInv_so3(Eigen::Vector3d err)
    {
      Eigen::Matrix3d result;
      double theta,halftheta;
      theta = err.norm();
      //这里可以使用gtsam代码
      if(theta<0.001)
      {
	result = Eigen::Matrix3d::Identity();
      }
      else
      {
	Eigen::Matrix3d W = Hat(err);
	result = Eigen::Matrix3d::Identity()+0.5*W+(1/(theta*theta)-(1+cos(theta))/(2*theta*sin(theta)))*W*W; 
      } 
      return result;
    }
    //误差函数定义:||log(△R.inverse*Ri.inverse*Rj)|| 使用的左乘更新状态
    class EdgeSO3Lie: public g2o::BaseBinaryEdge<3, Sophus::SO3, VertexSO3LieAlgebra, VertexSO3LieAlgebra>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
	
	// 误差的维度是9*1
	virtual void computeError()
	{
	    Sophus::SO3 v1 = (static_cast<VertexSO3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SO3 v2 = (static_cast<VertexSO3LieAlgebra*> (_vertices[1]))->estimate();
	    Sophus::SO3 error_matrix = _measurement.inverse()*v1.inverse()*v2;
	    _error =error_matrix.log();
	}
	
	// 雅可比计算 维度是9*3
	virtual void linearizeOplus()
	{
	    Sophus::SO3 v1 = (static_cast<VertexSO3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SO3 v2 = (static_cast<VertexSO3LieAlgebra*> (_vertices[1]))->estimate();
	    Eigen::Vector3d error_lie = (_measurement.inverse()*v1.inverse()*v2).log();
	    Eigen::Matrix3d JRINV_error=JRInv_so3(error_lie);
	    _jacobianOplusXi = -JRINV_error*v2.matrix().transpose();
	    _jacobianOplusXj = JRINV_error*v2.matrix().transpose();
	}
	 //根据状态生成cost function的残差
    	 static double Produce_Error(vector<pose> poses,vector<edge> edges)
	{
	  double sum_error=0;
	  for(int i=0;i<edges.size();i++)
	  {
	    int id1 = edges[i].id1;
	    int id2 = edges[i].id2;
	    Eigen::Matrix3d R1 = poses[id1].q.toRotationMatrix();
	    Eigen::Matrix3d R2 = poses[id2].q.toRotationMatrix();
	    Eigen::Matrix3d R_mea = edges[i].delta_q.toRotationMatrix();
	    Sophus::SO3 so3_error(R_mea.transpose()*R1.transpose()*R2);
	    Eigen::Vector3d error_vect = so3_error.log();
	    double error = error_vect.norm();
	    sum_error = sum_error + error*error; 
	  }
	  return sum_error;
	}
	static Eigen::Matrix<double,3,3> InformationMatrix()
	{
	  Eigen::Matrix<double,3,3> info;
	  info = Eigen::Matrix<double,3,3>::Identity();
	  return info;
	}
    };
   
    
    
    //只有位置的顶点 t
    class Vertext: public g2o::BaseVertex<3, Eigen::Vector3d>
    {
       public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	   bool read ( istream& is ){}
	   bool write ( ostream& os ) const {}
	  
	  virtual void setToOriginImpl()
	  {
	      _estimate = Eigen::Vector3d(0,0,0);
	  }
	  // 左乘更新
	  virtual void oplusImpl ( const double* update )
	  {
	      Eigen::Vector3d up =  Eigen::Vector3d( update[0], update[1], update[2] );
	      _estimate = up+_estimate;
	  }
    };
     //只对位置优化
     //误差函数定义:||t1-t2-测量值|| 
    class EdgePositionOnly: public g2o::BaseBinaryEdge<3, Eigen::Vector3d,Vertext, Vertext>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
	
	// 误差的维度是9*1
	virtual void computeError()
	{
	    Eigen::Vector3d v1 = (static_cast<Vertext*> (_vertices[0]))->estimate();
	    Eigen::Vector3d v2 = (static_cast<Vertext*> (_vertices[1]))->estimate();  
	    _error = v2-v1-_measurement;
	}
	
	// 雅可比计算 维度是9*3
	virtual void linearizeOplus()
	{
	    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
	    _jacobianOplusXj =  Eigen::Matrix3d::Identity();
	}
	//计算残差
	static double Produce_Error(vector<pose> poses,vector<edge> edges)
	{
	  double sum_error=0;
	  for(int i=0;i<edges.size();i++)
	  {
	    Eigen::Vector3d t1 = poses[edges[i].id1].t;
	    Eigen::Vector3d t2 = poses[edges[i].id2].t;
	    Eigen::Vector3d t_delta = t2-t1-poses[edges[i].id1].q.toRotationMatrix()*edges[i].delta_t;
	    double error = t_delta.norm();
	    sum_error = sum_error + error*error;
	  }
	  return sum_error;
	}
    };
    
    
    
    //下面是slam14讲中的posegraph优化公式
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    // 给定误差求Jr^{-1}的近似
    Matrix6d JRInv( Sophus::SE3 e )
    {
	Matrix6d J;
	J.block(0,0,3,3) = SO3::hat(e.so3().log());
	J.block(0,3,3,3) = SO3::hat(e.translation());
	J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
	J.block(3,3,3,3) = SO3::hat(e.so3().log());
	J = J*0.5 + Matrix6d::Identity();
	return J;
    }
    // 李代数顶点
    class VertexSE3LieAlgebra: public g2o::BaseVertex<6, Sophus::SE3>
    {
	public:
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	     bool read ( istream& is ){}
	     bool write ( ostream& os) const{}
	    virtual void setToOriginImpl()
	    {
		_estimate = Sophus::SE3();
	    }
	    // 左乘更新  一定要注意这里的左乘更新符合公式推导
	    virtual void oplusImpl ( const double* update )
	    {
		Sophus::SE3 up (
		    Sophus::SO3::exp( Eigen::Vector3d(update[3], update[4], update[5])),//原来的代码写的有些问题 原来的代码中没有exp
		    Eigen::Vector3d ( update[0], update[1], update[2] )//千万要注意其实这里是有近似关系的 我们的update其实是李代数，但是这里为什么位移对应的李代数直接等于t呢是因为当旋转矩阵够小则两者相等
		);
		_estimate = up*_estimate;
	    }
    };

    // 两个李代数节点之边slam14讲中定义的边  Sophus::SE3表示测量值类型呢
    class EdgeSE3LieAlgebra: public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
      
	// 误差计算与书中推导一致
	virtual void computeError()
	{
	    Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
	    _error = (_measurement.inverse()*v1.inverse()*v2).log();
	}
	
	// 雅可比计算
	virtual void linearizeOplus()
	{
	    Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
	    Matrix6d J = JRInv(SE3::exp(_error));
	    // 尝试把J近似为I？
	    _jacobianOplusXi = - J* v2.inverse().Adj();
	    _jacobianOplusXj = J*v2.inverse().Adj();
	}
	//根据状态生成cost function的残差
    	 static double Produce_Error(vector<pose> poses,vector<edge> edges)
	{
	  double sum_error=0;
	  for(int i=0;i<edges.size();i++)
	  {
	    int id1 = edges[i].id1;
	    int id2 = edges[i].id2;
	    Sophus::SE3 T1(poses[id1].q,poses[id1].t);
	    Sophus::SE3 T2(poses[id2].q,poses[id2].t);
	    Sophus::SE3 T_mea(edges[i].delta_q,edges[i].delta_t);
	    double error = ((T_mea.inverse()*T1.inverse()*T2).log()).norm();
	    sum_error = sum_error + error*error; 
	  }
	  return sum_error;
	}
    };
    
    
    
    
    //我们自己定义的顶点 
    //这个顶点的参数顺序是fi 和 t
    class VertexSE3_fiandt: public g2o::BaseVertex<6, Sophus::SE3>
    {
	public:
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	     bool read ( istream& is ){}
	     bool write ( ostream& os) const{}
	    virtual void setToOriginImpl()
	    {
		_estimate = Sophus::SE3();
	    }
	    // 左乘更新  一定要注意这里的左乘更新符合公式推导
	    virtual void oplusImpl ( const double* update )
	    {
	        Sophus::SE3 update_SE3;//表示更新之后的状态
		Sophus::SO3 R_update = Sophus::SO3::exp( Eigen::Vector3d(update[0], update[1], update[2]));
		Eigen::Vector3d t_update(update[3], update[4], update[5]);
		update_SE3 = Sophus::SE3(R_update.matrix()*_estimate.rotation_matrix(),_estimate.translation()+t_update);
		_estimate = update_SE3;
	    }
    };
    
    //我们自己定义的顶点fyy 误差方程=[log(deltaR.inverse*Ri.inverse*Rj) Ri(tj-ti)-delta_t]
    class EdgeSE3_findt: public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3_fiandt, VertexSE3_fiandt>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
      
	// 误差计算与书中推导一致
	virtual void computeError()
	{
	    Sophus::SE3 v1 = (static_cast<VertexSE3_fiandt*> (_vertices[0]))->estimate();
	    Sophus::SE3 v2 = (static_cast<VertexSE3_fiandt*> (_vertices[1]))->estimate();
	    Sophus::SE3 error_sophus = _measurement.inverse()*v1.inverse()*v2;
	    Eigen::Matrix<double,6,1> error;
	    Eigen::Vector3d error_fi = error_sophus.so3().log();
	    Eigen::Vector3d error_t = error_sophus.translation();
	    error<<error_fi,error_t;//需要测试一下
	    _error = error;
	}
	
	// 雅可比计算
	virtual void linearizeOplus()
	{
	    Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	    Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*> (_vertices[1]))->estimate();
	    Eigen::Vector3d tj = v2.translation();
	    Eigen::Vector3d ti = v1.translation();
	    Eigen::Matrix3d Ri = v1.rotation_matrix();
	    Eigen::Matrix3d Rj = v2.rotation_matrix();
	    Eigen::Matrix3d RdeltaT = _measurement.rotation_matrix().transpose();
	    Sophus::SE3 error_sophus = _measurement.inverse()*v1.inverse()*v2;
	    Eigen::Vector3d error_fi = error_sophus.so3().log();
	    Eigen::Matrix<double,6,6> jacobianXi,jacobianXj;
	    Eigen::Matrix3d JRINV_error = JRInv_so3(error_fi);
	    jacobianXi<<-JRINV_error*Rj.transpose(),Eigen::Matrix3d::Zero(),RdeltaT*Ri.transpose()*Hat(tj-ti),-RdeltaT*Ri.transpose();
	    jacobianXj<<JRINV_error*Rj.transpose(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),RdeltaT*Ri.transpose();
	    _jacobianOplusXi = jacobianXi;
	    _jacobianOplusXj = jacobianXj;
	}
	//根据状态生成cost function的残差
    	 static double Produce_Error(vector<pose> poses,vector<edge> edges)
	{
	  double sum_error=0;
	  for(int i=0;i<edges.size();i++)
	  {
	    int id1 = edges[i].id1;
	    int id2 = edges[i].id2;
	    Sophus::SE3 T1(poses[id1].q,poses[id1].t);
	    Sophus::SE3 T2(poses[id2].q,poses[id2].t);
	    Sophus::SE3 T_mea(edges[i].delta_q,edges[i].delta_t);
	    Sophus::SE3 error_sophus = T_mea.inverse()*T1.inverse()*T2;
	    Eigen::Vector3d error_fi = error_sophus.so3().log();
	    Eigen::Vector3d error_t = error_sophus.translation();
	    Eigen::Matrix<double,6,1> error;
	    error<<error_fi,error_t;
	    sum_error = sum_error  + error.norm()*error.norm();
	  }
	  return sum_error;
	}
    };
    
    
    //使用我们自己方法的一元边估计相对位姿变化,这里的误差函数使用的是李代数log(Tmeas.inverse*T) 
    //这里顶点类型我们使用slam14讲中提供的VertexSE3LieAlgebra
    class EdgeUnary_SE3Lie : public g2o::BaseUnaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra>//测量值维度 测量值类型 顶点类型
    {
       public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
	 
	 virtual void computeError()
	 {
	   Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	   _error = (_measurement.inverse()*v1).log();
	 }
	 virtual void linearizeOplus()
	 {
	   Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	   Matrix6d J = JRInv(SE3::exp(_error));
	    _jacobianOplusXi = J* v1.inverse().Adj();
	 }
    };
    
    //这里也是一元边 误差函数使用的是[log(Rmea.inverse*R) tc-t]
    //使用的是我们自己定义的顶点VertexSE3_fiandt
     class EdgeUnary_SE3Fyy : public g2o::BaseUnaryEdge<6, Sophus::SE3, VertexSE3_fiandt>//测量值维度 测量值类型 顶点类型
    {
       public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 bool read ( istream& is ){}
	 bool write ( ostream& os ) const{}
	 
	 virtual void computeError()
	 {
	   Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	   Sophus::SE3 error_sophus = _measurement.inverse()*v1;
	   Eigen::Vector3d delta_fi = error_sophus.so3().log();
	   Eigen::Vector3d delta_t = _measurement.translation()-v1.translation();
	   Eigen::Matrix<double,6,1> error;
	   error<<delta_fi,delta_t;
	   _error = error;
	 }
	 virtual void linearizeOplus()
	 {
	   Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*> (_vertices[0]))->estimate();
	   Sophus::SE3 error_sophus = _measurement.inverse()*v1;
	   Eigen::Vector3d delta_fi = error_sophus.so3().log();
	   Eigen::Matrix3d JRINV_error = JRInv_so3(delta_fi);
	   Eigen::Matrix<double,6,6> jacobianXi;
	   jacobianXi<<JRINV_error* v1.rotation_matrix().transpose(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),-Eigen::Matrix3d::Identity();
	   _jacobianOplusXi = jacobianXi;
	 }
    };

    
}
#endif