#ifndef _ICP_
#define _ICP_


#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>//这个函数的作用引入罗德里格斯公式

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> //为了求特征值

using namespace std;
using namespace cv;
namespace ICP
{
   
    //用于估计两个坐标系的变化
    //使用SVD方法计算得到两相机的相对位姿
    //计算得到的R和t是将pts2坐标系下的坐标变换到pts1坐标系下的位姿变化----------------------------------------------
    //使用的是slam14讲的代码
    //返回的是c,p'=R(p-c)
    void pose_estimation_3d3d (const vector<Point3d>& pts1,const vector<Point3d>& pts2,Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
    {
        //cout<<"slam14讲中的SVD方法"<<endl;
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
		//cout<<"R的行列式大于0，如果R的行列式小于零则W的奇异值有一个需要是0"<<endl;
		//cout<<"W行列式："<<det<<endl;
		Eigen::EigenSolver<Eigen::MatrixXd> es1( W );
		cout<<"ICP SVD 特征值 = "<<endl<<es1.eigenvalues()<<endl;
		// SVD on W
		Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Matrix3d  S = U.inverse() * W * V.transpose().inverse(); 
		//cout<<"S="<<endl<<S<<endl;
        //cout<<"U="<<endl<<U<<endl;
		//cout<<"V="<<endl<<V<<endl;
		//cout<<"U的行列式 = "<<U.determinant()<<endl;
		//cout<<"V的行列是 = "<<V.determinant()<<endl;
		if (U.determinant() * V.determinant() < 0)
		{
		R_ = U* ( V.transpose() );
		//cout<<"R处理之前的行列式 = "<<R_.determinant()<<endl;
		for (int x = 0; x < 3; ++x)
		{
			U(x, 2) *= -1;
		}
		}
		R_ = U* ( V.transpose() );
		//cout<<"R的行列式 = "<<R_.determinant()<<endl;
		t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );
	}
	
	/*
	struct ICPCeres
	{
	ICPCeres ( Point3f uvw,Point3f xyz ) : _uvw(uvw),_xyz(xyz) {}
	// 残差的计算
	template <typename T>
	bool operator() (
		const T* _q,    // 模型参数
		const T* _t,
		T* residual ) const     // 残差
	{
	
		Eigen::Quaternion<T> q_r{_q[3],_q[0],_q[1],_q[2]};
		Eigen::Matrix<T,3,1> point{T(_xyz.x),T(_xyz.y),T(_xyz.z)};
		Eigen::Matrix<T,3,1> tt{_t[0],_t[1],_t[2]};
			
		Eigen::Matrix<T,3,1> sta=q_r*point+tt;	
		residual[0] = T(_uvw.x)-(sta(0));
		residual[1] = T(_uvw.y)-(sta(1));
		residual[2] = T(_uvw.z)-(sta(2));
		

		return true;
	}
	static ceres::CostFunction* Create(const Point3f uvw,const Point3f xyz) {
		return (new ceres::AutoDiffCostFunction<ICPCeres, 3, 4, 3>(new ICPCeres(uvw,xyz)));
	}
	const Point3f _uvw;
	const Point3f _xyz;
	};*/
	
	/*
	void pose_estimation_ICP_ceres(const vector<Point3f> &pts1,const vector<Point3f> &pts2,Eigen::Matrix3d &R, Eigen::Vector3d &t)
	{

		// compute q1*q2^T
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for ( int i=0; i<N; i++ )
		{
			W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
		}
		double det = W.determinant();
		//cout<<"R的行列式大于0，如果R的行列式小于零则W的奇异值有一个需要是0"<<endl;
		//cout<<"W行列式："<<det<<endl;
		Eigen::EigenSolver<Eigen::MatrixXd> es1( W );
		cout<<"ICP SVD 特征值 = "<<endl<<es1.eigenvalues()<<endl;
		// SVD on W
		Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Matrix3d  S = U.inverse() * W * V.transpose().inverse(); 
		//cout<<"S="<<endl<<S<<endl;
			//cout<<"U="<<endl<<U<<endl;
		//cout<<"V="<<endl<<V<<endl;
		//cout<<"U的行列式 = "<<U.determinant()<<endl;
		//cout<<"V的行列是 = "<<V.determinant()<<endl;
		if (U.determinant() * V.determinant() < 0)
		{
		R_ = U* ( V.transpose() );
		//cout<<"R处理之前的行列式 = "<<R_.determinant()<<endl;
		for (int x = 0; x < 3; ++x)
		{
			U(x, 2) *= -1;
		}
		}
		R_ = U* ( V.transpose() );
		//cout<<"R的行列式 = "<<R_.determinant()<<endl;
		t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );
		}
		ceres::Problem problem;
		Eigen::Matrix<double, 6, 1> tf_calculated_parameter=initialvalue;
		
		Eigen::Matrix4d result=Sophus::SE3d::exp(tf_calculated_parameter).matrix();
		//Eigen::Matrix3d R;
		R<<result(0,0),result(0,1),result(0,2),
		result(1,0),result(1,1),result(1,2),
		result(2,0),result(2,1),result(2,2);
		
		Eigen::Quaterniond q(R);
		double q1[4];
		q1[0]=q.x();
		q1[1]=q.y();
		q1[2]=q.z();
		q1[3]=q.w();
		
		double t1[3];
		t1[0]=result(0,3),t1[1]=result(1,3),t1[2]=result(2,3);
		
		for (int i = 0; i < pts2.size(); ++i) {
		
		
		ceres::CostFunction *cost_function = ICPCeres::Create(pts2[i], pts1[i]);
		problem.AddResidualBlock(cost_function, NULL, q1,t1);
		}
		ceres::Solver::Options options;
		options.linear_solver_type=ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout=true;
		ceres::Solver::Summary summary;
		ceres::Solve(options,&problem,&summary);
		
		std::cout << summary.BriefReport() << std::endl;
		
		
		Eigen::Quaterniond qr;
		qr.normalize();
		qr.x()=q1[0];
		qr.y()=q1[1];
		qr.z()=q1[2];
		qr.w()=q1[3];
		
		t<<t1[0],t1[1],t1[2];
		Sophus::SE3d SE3_qt(qr,t);
		
		Eigen::Matrix4d result1=SE3_qt.matrix();
		
		R<<result1(0,0),result1(0,1),result1(0,2),
		result1(1,0),result1(1,1),result1(1,2),
		result1(2,0),result1(2,1),result1(2,2);

		std::cout << "R:\n"
			<< R<<std::endl;
		std::cout << "t:\n"
			<< t<<std::endl;

	}*/

}
#endif

