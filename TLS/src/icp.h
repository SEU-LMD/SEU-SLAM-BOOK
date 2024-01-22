#ifndef _ICP_
#define _ICP_
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>//这个函数的作用引入罗德里格斯公式

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> //为了求特征值

#include <iostream>

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

#include "convert.h"
using namespace cv;
using namespace std;
namespace TLS
{
    //用于估计两个坐标系的变化
    //使用SVD方法计算得到两相机的相对位姿
    //计算得到的R和t是将pts2坐标系下的坐标变换到pts1坐标系下的位姿变化----------------------------------------------
    //使用的是slam14讲的代码
    //返回的是c,p'=R(p-c)
    Eigen::Vector3d pose_estimation_3d3d (const vector<Point3d>& pts1,const vector<Point3d>& pts2,Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
    {
        cout<<"slam14讲中的SVD方法"<<endl;
	Point3d p1, p2;     // center of mass
	int N = pts1.size();
	for ( int i=0; i<N; i++ )
	{
	    p1 += pts1[i];
	    p2 += pts2[i];
	}
	p1 = Point3d( Vec3d(p1) /  N);
	p2 = Point3d( Vec3d(p2) / N);
	vector<Point3d>     q1 ( N ), q2 ( N ); // remove the center
	for ( int i=0; i<N; i++ )
	{
	    q1[i] = pts1[i] - p1;
	    q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for ( int i=0; i<N; i++ )
	{
	    W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
	}
	double det = W.determinant();
	cout<<"R的行列式大于0，如果R的行列式小于零则W的奇异值有一个需要是0"<<endl;
	cout<<"W行列式："<<det<<endl;
	Eigen::EigenSolver<Eigen::MatrixXd> es1( W );
	cout<<"W的特征值 = "<<endl<<es1.eigenvalues()<<endl;
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::Matrix3d  S = U.inverse() * W * V.transpose().inverse(); 
	cout<<"S="<<endl<<S<<endl;
        cout<<"U="<<endl<<U<<endl;
	cout<<"V="<<endl<<V<<endl;
	cout<<"U的行列式 = "<<U.determinant()<<endl;
	cout<<"V的行列是 = "<<V.determinant()<<endl;
	if (U.determinant() * V.determinant() < 0)
	{
	  R_ = U* ( V.transpose() );
	  cout<<"R处理之前的行列式 = "<<R_.determinant()<<endl;
	  for (int x = 0; x < 3; ++x)
	  {
	      U(x, 2) *= -1;
	  }
	}
	R_ = U* ( V.transpose() );
	cout<<"R的行列式 = "<<R_.determinant()<<endl;
	t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );
	Eigen::Vector3d c = Eigen::Vector3d ( p2.x, p2.y, p2.z )-R_.transpose()*Eigen::Vector3d ( p1.x, p1.y, p1.z );
	return c;
    }
    
    //计算得到的R和t是将pts2坐标系下的坐标变换到pts1坐标系下的位姿变化----------------------------------------------
    //使用的是state robot书中的根据R矩阵求解得到的结果
    Eigen::Vector3d pose_estimation_staterobot_3d3d (const vector<Point3d>& pts1,const vector<Point3d>& pts2,Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
    {
        cout<<"state robot书中的根据R矩阵求解得到的结果"<<endl;
	Point3d p1, p2;     // center of mass
	int N = pts1.size();
	for ( int i=0; i<N; i++ )
	{
	    p1 += pts1[i];
	    p2 += pts2[i];
	}
	p1 = Point3d( Vec3d(p1) /  N);
	p2 = Point3d( Vec3d(p2) / N);
	vector<Point3d>     q1 ( N ), q2 ( N ); // remove the center
	for ( int i=0; i<N; i++ )
	{
	    q1[i] = pts1[i] - p1;
	    q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for ( int i=0; i<N; i++ )
	{
	    W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
	}
	W = W/N;
	double det = W.determinant();
	cout<<"det(W)>0 || det(W)<0且W的最小奇异值只有一个 || rank(W)=2"<<endl;
	cout<<"W行列式："<<det<<endl;
	Eigen::EigenSolver<Eigen::MatrixXd> es1( W );
	cout<<"W的特征值 = "<<endl<<es1.eigenvalues()<<endl;
	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::Matrix3d D = U.transpose()*W*V;
	cout<<"W的奇异值 = "<<D<<endl;
	Eigen::Matrix3d  S; 
	S<<1,0,0,
	   0,1,0,
	   0,0,U.determinant()*V.determinant();
        cout<<"U="<<endl<<U<<endl;
	cout<<"V="<<endl<<V<<endl;
	cout<<"U的行列式 = "<<U.determinant()<<endl;
	cout<<"V的行列是 = "<<V.determinant()<<endl;
	
	R_ = U* S*( V.transpose() );
	t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );
	
	Eigen::Vector3d c = Eigen::Vector3d ( p2.x, p2.y, p2.z )-R_.transpose()*Eigen::Vector3d ( p1.x, p1.y, p1.z );
	return c;
    }
    
    
    
   void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
	cv::reduce(P,C,1,CV_REDUCE_SUM);
	C = C/P.cols;

	for(int i=0; i<P.cols; i++)
	{
	    Pr.col(i)=P.col(i)-C;
	}
    }


    //这部分是orb的代码使用四元数估计ICp----------------------------------------------
    //当N矩阵的最大特征值唯一时，四元数有唯一的解
    //计算得到的是pts2坐标系到pts1坐标系下的点
    //返回的值是c, R(p-c)
    Eigen::Vector3d  pose_estimation_3d3d_quaeternion (
	const vector<Point3d>& pts1,
	const vector<Point3d>& pts2,
	Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
    {
        cout<<"ORB代码中的四元数方法"<<endl;
	cv::Mat mR12i;
	cv::Mat mt12i;
	double ms12i;//这个参数是尺度
	cv::Mat mT12i;
	cv::Mat mT21i;
	cv::Mat P1(3,pts1.size(),CV_64FC1),P2(3,pts1.size(),CV_64FC1);
	for(int i=0;i<pts1.size();i++)
	{
	  double a[3] = {pts1[i].x ,pts1[i].y ,pts1[i].z};
	  double b[3] = {pts2[i].x ,pts2[i].y ,pts2[i].z};
	  cv::Mat col1(3,1,CV_64FC1,a);
	  cv::Mat col2(3,1,CV_64FC1,b);
	  col1.copyTo(P1.col(i));
	  col2.copyTo(P2.col(i));
	}
	


	// Custom implementation of:
	// Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

	// Step 1: Centroid and relative coordinates

	cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
	cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
	cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
	cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

	ComputeCentroid(P1,Pr1,O1);
	ComputeCentroid(P2,Pr2,O2);
	

	// Step 2: Compute M matrix

	cv::Mat M = Pr2*Pr1.t();

	// Step 3: Compute N matrix
	double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

	cv::Mat N(4,4,P1.type());

	N11 = M.at<double>(0,0)+M.at<double>(1,1)+M.at<double>(2,2);
	N12 = M.at<double>(1,2)-M.at<double>(2,1);
	N13 = M.at<double>(2,0)-M.at<double>(0,2);
	N14 = M.at<double>(0,1)-M.at<double>(1,0);
	N22 = M.at<double>(0,0)-M.at<double>(1,1)-M.at<double>(2,2);
	N23 = M.at<double>(0,1)+M.at<double>(1,0);
	N24 = M.at<double>(2,0)+M.at<double>(0,2);
	N33 = -M.at<double>(0,0)+M.at<double>(1,1)-M.at<double>(2,2);
	N34 = M.at<double>(1,2)+M.at<double>(2,1);
	N44 = -M.at<double>(0,0)-M.at<double>(1,1)+M.at<double>(2,2);

	N = (cv::Mat_<double>(4,4) << N11, N12, N13, N14,
				    N12, N22, N23, N24,
				    N13, N23, N33, N34,
				    N14, N24, N34, N44);


	// Step 4: Eigenvector of the highest eigenvalue
	cv::Mat eval, evec;

	cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation
	cout<<"当N矩阵的最大特征值唯一时且N有四个线性无关的特征向量时，四元数有唯一的解"<<endl;
	cout<<"N特征值："<<endl;
	cout<<eval.at<double>(0,0)<<endl;
	cout<<eval.at<double>(0,1)<<endl;
	cout<<eval.at<double>(0,2)<<endl;
	cout<<eval.at<double>(0,3)<<endl;
	cv::Mat vec(1,3,evec.type());
	//cout<<evec<<endl;
	(evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)
	//cout<<vec<<endl;
	// Rotation angle. sin is the norm of the imaginary part, cos is the real part
	//<<norm(vec)<<endl;
	//cout<<evec.at<double>(0,0)<<endl;
	//cout<<norm(vec)/evec.at<double>(0,0)<<endl;
	double ang=atan2(norm(vec),evec.at<double>(0,0));
	//cout<<ang<<endl;
	vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

	mR12i.create(3,3,P1.type());

	cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis
	Eigen::Quaterniond  mQ =  Eigen::Quaterniond(evec.at<double>(0,0),evec.at<double>(0,1),evec.at<double>(0,2),evec.at<double>(0,3));
	Sophus::SO3 SO3_q(mQ);
	Eigen::Vector3d so3 = SO3_q.log();
	cv::Mat mR12;
	cv::Rodrigues(vec,mR12);
	//cout<<mR12<<endl;
	//cout<<mR12i<<endl;
	///cout<<"so3:"<<so3<<endl;
	//cout<<"so3 模："<<so3.norm()<<endl;
	///cout<<"so3 轴："<<so3/so3.norm()<<endl;
	//cout<<"vec:"<<vec<<endl;
	//cout<<"vec 模："<<norm(vec)<<endl;
	///cout<<"vec 轴："<<vec/norm(vec)<<endl;
	Eigen::Matrix<double,3,3,Eigen::ColMajor> R  = mQ.toRotationMatrix();
	// Step 5: Rotate set 2
	cv::Mat P3 = mR12i*Pr2;
      
	// Step 6: Scale
	
	ms12i = 1.0;

	// Step 7: Translation

	mt12i.create(1,3,P1.type());
	mt12i = O1 - ms12i*mR12i*O2;

	// Step 8: Transformation

	// Step 8.1 T12
	mT12i = cv::Mat::eye(4,4,P1.type());

	cv::Mat sR = ms12i*mR12i;

	sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
	mt12i.copyTo(mT12i.rowRange(0,3).col(3));

	// Step 8.2 T21

	mT21i = cv::Mat::eye(4,4,P1.type());

	cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

	sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
	cv::Mat tinv = -sRinv*mt12i;
	tinv.copyTo(mT21i.rowRange(0,3).col(3));
	
	R_ = toMatrix3d(mR12i);
	//cout<<mR12i<<endl;
	//cout<<R<<endl;
	t_ = toVector3d(mt12i);
	cout<<"R的行列式 = "<<R_.determinant()<<endl;
	cv::Mat c;
	c = O2 - mR12i.t()*O1;
	return toVector3d(c);
	//return so3;
   }

   
   //下面是使用非线性优化计算ICp的方法--------------------------------------------------------------------------------------------------------
   // g2o edge
    //这个是作者自己定义的一元边-用于估计R和t
    class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>//这里的3表示误差的维度，Vector3d表示误差的数据类型，VertexSE3Expmap表示待优化点的数据类型
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

	virtual void computeError()
	{
	    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
	    // measurement is p, point is p'
	    _error = _measurement - pose->estimate().map( _point );
	}
	
	virtual void linearizeOplus()
	{
	    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
	    g2o::SE3Quat T(pose->estimate());
	    Eigen::Vector3d xyz_trans = T.map(_point);
	    double x = xyz_trans[0];
	    double y = xyz_trans[1];
	    double z = xyz_trans[2];
	    
	    _jacobianOplusXi(0,0) = 0;
	    _jacobianOplusXi(0,1) = -z;
	    _jacobianOplusXi(0,2) = y;
	    _jacobianOplusXi(0,3) = -1;
	    _jacobianOplusXi(0,4) = 0;
	    _jacobianOplusXi(0,5) = 0;
	    
	    _jacobianOplusXi(1,0) = z;
	    _jacobianOplusXi(1,1) = 0;
	    _jacobianOplusXi(1,2) = -x;
	    _jacobianOplusXi(1,3) = 0;
	    _jacobianOplusXi(1,4) = -1;
	    _jacobianOplusXi(1,5) = 0;
	    
	    _jacobianOplusXi(2,0) = -y;
	    _jacobianOplusXi(2,1) = x;
	    _jacobianOplusXi(2,2) = 0;
	    _jacobianOplusXi(2,3) = 0;
	    _jacobianOplusXi(2,4) = 0;
	    _jacobianOplusXi(2,5) = -1;
	}

	bool read ( istream& in ) {}
	bool write ( ostream& out ) const {}
    protected:
	Eigen::Vector3d _point;
    };
    
    //下面是使用非线性优化计算ICp的方法--------------------------------------------------------------------------------------------------------
    //先定义一个顶点
    class ScaleVertex:public g2o::BaseVertex<1,double>
    {
      public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	virtual void setToOriginImpl()
	{
	  _estimate =1.0;
	}
	
	virtual void oplusImpl(const double* update)
	{
	  _estimate =_estimate+update[0];
	}
	virtual bool read(istream& in){}
	virtual bool write(ostream& out)const {}
    };
   // g2o edge
    //这个是作者自己定义的一元边-用于估计R和t并且还有尺度
    //这里的3表示误差的维度，Vector3d表示误差的数据类型，VertexSE3Expmap表示待优化点的数据类型,double是要估计的尺度
    class EdgeProjectXYZRGBDPoseOnlyWithScale : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap,ScaleVertex>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeProjectXYZRGBDPoseOnlyWithScale( const Eigen::Vector3d& point ) : _point(point) {}

	virtual void computeError()
	{
	    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
	    const ScaleVertex* scale = static_cast<const ScaleVertex*>(_vertices[1]);
	    // measurement is p, point is p'
	    const double s= scale->estimate();
	    //cout<<"尺度="<<s<<endl;
	    _error = _measurement - pose->estimate().map( s*_point );
	}
	
	virtual void linearizeOplus()
	{
	    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
	    ScaleVertex* scale = static_cast<ScaleVertex *>(_vertices[1]);
	    g2o::SE3Quat T(pose->estimate());
	    const double s= scale->estimate();
	    Eigen::Vector3d t;
	    t<<T.toVector()[0],T.toVector()[1],T.toVector()[2];
	    
	    Eigen::Vector3d xyz_trans = T.map(s*_point);
	    double x = xyz_trans[0];
	    double y = xyz_trans[1];
	    double z = xyz_trans[2];
	    Eigen::Vector3d abc_trans = (xyz_trans-t)/s;
	    double a = abc_trans[0];
	    double b = abc_trans[1];
	    double c = abc_trans[2];
	    _jacobianOplusXi(0,0) = 0;
	    _jacobianOplusXi(0,1) = -z;
	    _jacobianOplusXi(0,2) = y;
	    _jacobianOplusXi(0,3) = -1;
	    _jacobianOplusXi(0,4) = 0;
	    _jacobianOplusXi(0,5) = 0;
	    
	    _jacobianOplusXi(1,0) = z;
	    _jacobianOplusXi(1,1) = 0;
	    _jacobianOplusXi(1,2) = -x;
	    _jacobianOplusXi(1,3) = 0;
	    _jacobianOplusXi(1,4) = -1;
	    _jacobianOplusXi(1,5) = 0;
	    
	    _jacobianOplusXi(2,0) = -y;
	    _jacobianOplusXi(2,1) = x;
	    _jacobianOplusXi(2,2) = 0;
	    _jacobianOplusXi(2,3) = 0;
	    _jacobianOplusXi(2,4) = 0;
	    _jacobianOplusXi(2,5) = -1;
	    
	    _jacobianOplusXj(0,0) = -a;
	    _jacobianOplusXj(1,0) = -b;
	    _jacobianOplusXj(2,0) = -c;
	}

	bool read ( istream& in ) {}
	bool write ( ostream& out ) const {}
    protected:
	Eigen::Vector3d _point;
    };
    //这个是后面的lamda约束边也是用于估计R和t的
    class EdgeProjectXYZRGBDPoseOnly_Constrained : public g2o::BaseUnaryEdge<1,  double, g2o::VertexSE3Expmap>
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeProjectXYZRGBDPoseOnly_Constrained( Eigen::Matrix<double,6,1> _Lie_best,double _lamb) : Lie_best(_Lie_best),lamb(_lamb){}

	virtual void computeError()
	{
	    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
	    // measurement is p, point is p'
	    //cout<<pose->estimate().log()<<endl;
	    //cout<<Lie_best<<endl;
	    _error(0,0) = lamb*pose->estimate().log().dot(Lie_best)/(Lie_best.norm()*Lie_best.norm());
	    //cout<<_error(0,0)<<endl;
	}
	
	virtual void linearizeOplus()
	{
	    _jacobianOplusXi(0,0) = lamb*Lie_best(0);
	    _jacobianOplusXi(0,1) = lamb*Lie_best(1);
	    _jacobianOplusXi(0,2) = lamb*Lie_best(2);
	    _jacobianOplusXi(0,3) = lamb*Lie_best(3);
	    _jacobianOplusXi(0,4) = lamb*Lie_best(4);
	    _jacobianOplusXi(0,5) = lamb*Lie_best(5);
	}

	bool read ( istream& in ) {}
	bool write ( ostream& out ) const {}
	Eigen::Matrix<double,6,1> Lie_best;
	double lamb;
    };
    
    
    
    //本优化函数用于优化gps坐标和slam坐标之间的位姿变化
    //lamda表示是否加约束项
    Eigen::Vector3d optimize_pose (const vector< Point3d >& pts1, const vector< Point3d >& pts2,
				  Eigen::Matrix3d& R, Eigen::Vector3d& t ,
				  double lamda=0,bool bRobust = false,int iterations =20 )
    {
        cout<<"非线性优化计算ICP"<<endl;
	cout<<endl<<"before optimization:"<<endl;
	cout<<"T="<<endl;
	cout<<R<<endl<<t<<endl;  
	// 初始化g2o
      
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, 误差的维度为 3
	
	if(lamda!=0)
	{
	  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1,-1> > Block;
	}
	
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
	Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));      // 矩阵块求解器
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );
	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm( solver );

	// vertex
	g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
	pose->setId(0);
    
	pose->setEstimate( g2o::SE3Quat(R,t));
	//pose->setEstimate( g2o::SE3Quat( Eigen::Matrix3d::Identity(),  Eigen::Vector3d( 0,0,0 ) ) );
	optimizer.addVertex( pose );

	// edges
	int index = 1;
	vector<EdgeProjectXYZRGBDPoseOnly*> edges;
	EdgeProjectXYZRGBDPoseOnly_Constrained* edge_lambda;
	for ( size_t i=0; i<pts1.size(); i++ )
	{
	    EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(  Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
	    edge->setId( index );
	    edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	    edge->setMeasurement( Eigen::Vector3d(  pts1[i].x, pts1[i].y, pts1[i].z) );
	    //edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );//高翔的源码设置的是10000,我这里测试过设置为1也能正常优化
	    edge->setInformation( Eigen::Matrix3d::Identity()/2);//设置权重
	    optimizer.addEdge(edge);
	    index++;
	    edges.push_back(edge);
	    if(bRobust)
	    {
		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		edge->setRobustKernel(rk);
		rk->setDelta(sqrt(0.1));//
	    }
	}
	if(lamda!=0)
	{
	  Eigen::Matrix<double,6,1> lie_T = g2o::SE3Quat(R,t).log(); 
	  cout<<"最优解的位姿李代数"<<lie_T<<endl;
	  EdgeProjectXYZRGBDPoseOnly_Constrained* edge  =new EdgeProjectXYZRGBDPoseOnly_Constrained(lie_T,lamda);
	  edge->setId( index );
	  edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	  edge->setInformation( Eigen::Matrix<double,1,1>::Identity() );
	  optimizer.addEdge(edge);
	  edge_lambda = edge;
	}
	optimizer.setVerbose ( true );
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
      
	cout<<"优化之后边的误差大小"<<endl;
	for(int i=0;i<edges.size();i++)
	{
	  cout<<edges[i]->id()<<":"<<sqrt(edges[i]->chi2())<<endl;
	}
	if(lamda!=0)
	{
	  cout<<edge_lambda->id()<<":"<<sqrt(edge_lambda->chi2())<<endl;
	}
	cout<<endl<<"after optimization:"<<endl;
	cout<<"T="<<endl;
	cout<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;  
	Eigen::Matrix<double,4,4> eT = pose->estimate().to_homogeneous_matrix();
	t << eT(0,3),eT(1,3),eT(2,3);
	R << eT(0,0),eT(0,1),eT(0,2),
	    eT(1,0),eT(1,1),eT(1,2),
	    eT(2,0),eT(2,1),eT(2,2);
	Eigen::Quaterniond mQ = pose->estimate().rotation();
	Sophus::SO3 SO3_q(mQ);
	Eigen::Vector3d so3 = SO3_q.log();
	return so3;
    }
    /*
    //这个相比于传统的icp多了一个尺度的求解
    //计算的是pts2坐标系到pts1坐标系的变化
    Eigen::Vector3d optimize_pose_with_scale( const vector< Point3d >& pts1,const vector< Point3d >& pts2,
					      Eigen::Matrix3d& R, Eigen::Vector3d& t ,double& scale,
					      double lamda=0,bool bRobust = false,int iterations =20 )
    {
        cout<<"非线性优化计算带有尺度的ICP"<<endl;
	cout<<endl<<"before optimization:"<<endl;
	cout<<"T="<<endl;
	cout<<R<<endl<<t<<endl;  
	cout<<"尺度 = "<<scale<<endl;
	// 初始化g2o
      
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1,-1> > Block;  // pose维度为 6, 另外一个待估计的尺度维度为 1
	
	if(lamda!=0)
	{
	  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1,-1> > Block;
	}
	
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器-使用这个能正常的求解得到结果但是迭代几步后就停止了
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器-使用这个无法正常求解得到正常的解
	Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));      // 矩阵块求解器
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm( solver );

	// vertex
	g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
	pose->setId(0);
	pose->setEstimate( g2o::SE3Quat(R,t));//pose->setEstimate( g2o::SE3Quat( Eigen::Matrix3d::Identity(),  Eigen::Vector3d( 0,0,0 ) ) );
	optimizer.addVertex( pose );
	
	ScaleVertex* scale_vertex = new ScaleVertex();
	scale_vertex->setId(1);
	scale_vertex->setEstimate(scale);
        optimizer.addVertex( scale_vertex );
	// edges
	int index = 1;
	vector<EdgeProjectXYZRGBDPoseOnlyWithScale*> edges;
	EdgeProjectXYZRGBDPoseOnly_Constrained* edge_lambda;
	for ( size_t i=0; i<pts1.size(); i++ )
	{
	    EdgeProjectXYZRGBDPoseOnlyWithScale* edge = new EdgeProjectXYZRGBDPoseOnlyWithScale(  Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
	    edge->setId( index );
	    edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	    edge->setVertex(1,dynamic_cast<ScaleVertex*>(scale_vertex));//dynamic_cast<ScaleVertex*>(scale_vertex)
	    edge->setMeasurement( Eigen::Vector3d(  pts1[i].x, pts1[i].y, pts1[i].z) );
	    //edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );//高翔的源码设置的是10000,我这里测试过设置为1也能正常优化
	    //edge->setInformation( Eigen::Matrix3d::Identity());//设置权重
	    optimizer.addEdge(edge);
	    index++;
	    edges.push_back(edge);
	    if(bRobust)
	    {
		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		edge->setRobustKernel(rk);
		rk->setDelta(sqrt(0.1));//
	    }
	}
	if(lamda!=0)
	{
	  Eigen::Matrix<double,6,1> lie_T = g2o::SE3Quat(R,t).log();
	  cout<<"最优解的位姿李代数"<<lie_T<<endl;
	  EdgeProjectXYZRGBDPoseOnly_Constrained* edge  = new EdgeProjectXYZRGBDPoseOnly_Constrained(lie_T,lamda);
	  edge->setId( index );
	  edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	  edge->setInformation( Eigen::Matrix<double,1,1>::Identity() );
	  optimizer.addEdge(edge);
	  edge_lambda = edge;
	}
	optimizer.setVerbose ( true );
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
      
	cout<<"优化之后边的误差大小"<<endl;
	for(int i=0;i<edges.size();i++)
	{
	  cout<<edges[i]->id()<<":"<<sqrt(edges[i]->chi2())<<endl;
	}
	if(lamda!=0)
	{
	  cout<<edge_lambda->id()<<":"<<sqrt(edge_lambda->chi2())<<endl;
	}
	cout<<endl<<"after optimization:"<<endl;
	cout<<"T="<<endl;
	cout<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;  
	Eigen::Matrix<double,4,4> eT = pose->estimate().to_homogeneous_matrix();
	t << eT(0,3),eT(1,3),eT(2,3);
	R << eT(0,0),eT(0,1),eT(0,2),
	    eT(1,0),eT(1,1),eT(1,2),
	    eT(2,0),eT(2,1),eT(2,2);
	Eigen::Quaterniond mQ = pose->estimate().rotation();
	scale = scale_vertex->estimate();
	cout<<"尺度 ="<<scale<<endl;
	Sophus::SO3 SO3_q(mQ);
	Eigen::Vector3d so3 = SO3_q.log();
	return so3;
    }
    
      //这个相比于optimize_pose多了一个权重的设置
      Eigen::Vector3d optimize_pose_new (
	  vector<ChessBoard> chess,
	  vector< pair<int,int> > datanum,//第一个元素是哪个标定板,第二个元素是标定板的哪个角点
	  vector<int> weights,//序号是标定板的序号,内容是这个标定版的权重
	  Eigen::Matrix3d& R, Eigen::Vector3d& t ,double lamda=0,bool bRobust = false,int iterations =20 )
      {
	  cout<<endl<<"before optimization:"<<endl;
	  cout<<"T="<<endl;
	  cout<<R<<endl<<t<<endl;  
	  // 初始化g2o
	
	  typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, 误差的维度为 3
	  
	  if(lamda!=0)
	  {
	    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1,-1> > Block;
	  }
	  
	  Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
	  Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));      // 矩阵块求解器
	  //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	  //g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );
	  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	  g2o::SparseOptimizer optimizer;
	  optimizer.setAlgorithm( solver );

	  // vertex
	  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
	  pose->setId(0);
      
	  pose->setEstimate( g2o::SE3Quat(R,t));
	  //pose->setEstimate( g2o::SE3Quat( Eigen::Matrix3d::Identity(),  Eigen::Vector3d( 0,0,0 ) ) );
	  optimizer.addVertex( pose );

	  // edges
	  int index = 1;
	  vector<EdgeProjectXYZRGBDPoseOnly*> edges;
	  EdgeProjectXYZRGBDPoseOnly_Constrained* edge_lambda;
	  for ( size_t i=0; i<datanum.size(); i++ )
	  {
	      EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly( chess[datanum[i].first].slam[datanum[i].second] );//这个是slam坐标
	      edge->setId( index );
	      edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	      edge->setMeasurement( chess[datanum[i].first].gps[datanum[i].second]);//这个是gps坐标
	      //edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );//高翔的源码设置的是10000,我这里测试过设置为1也能正常优化
	      edge->setInformation( weights[datanum[i].first]*Eigen::Matrix3d::Identity());//设置权重
	      optimizer.addEdge(edge);
	      index++;
	      edges.push_back(edge);
	      if(bRobust)
	      {
		  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		  edge->setRobustKernel(rk);
		  rk->setDelta(sqrt(0.1));//
	      }
	  }
	  if(lamda!=0)
	  {
	    Eigen::Matrix<double,6,1> lie_T = g2o::SE3Quat(R,t).log(); 
	    cout<<"最优解的位姿李代数"<<lie_T<<endl;
	    EdgeProjectXYZRGBDPoseOnly_Constrained* edge  =new EdgeProjectXYZRGBDPoseOnly_Constrained(lie_T,lamda);
	    edge->setId( index );
	    edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
	    edge->setInformation( Eigen::Matrix<double,1,1>::Identity() );
	    optimizer.addEdge(edge);
	    edge_lambda = edge;
	  }
	  optimizer.setVerbose ( true );
	  optimizer.initializeOptimization();
	  optimizer.optimize(iterations);
	
	  cout<<"优化之后边的误差大小"<<endl;
	  for(int i=0;i<edges.size();i++)
	  {
	    cout<<edges[i]->id()<<":"<<sqrt(edges[i]->chi2())<<endl;
	  }
	  if(lamda!=0)
	  {
	    cout<<edge_lambda->id()<<":"<<sqrt(edge_lambda->chi2())<<endl;
	  }
	  cout<<endl<<"after optimization:"<<endl;
	  cout<<"T="<<endl;
	  cout<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;  
	  Eigen::Matrix<double,4,4> eT = pose->estimate().to_homogeneous_matrix();
	  t << eT(0,3),eT(1,3),eT(2,3);
	  R << eT(0,0),eT(0,1),eT(0,2),
	      eT(1,0),eT(1,1),eT(1,2),
	      eT(2,0),eT(2,1),eT(2,2);
	  Eigen::Quaterniond mQ = pose->estimate().rotation();
	  Sophus::SO3 SO3_q(mQ);
	  Eigen::Vector3d so3 = SO3_q.log();
	  return so3;
      }*/
}
#endif