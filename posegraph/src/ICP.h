#ifndef _ICP_
#define _ICP_
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>//这个函数的作用引入罗德里格斯公式

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> //为了求特征值

#include <iostream>
#include <boost/concept_check.hpp>

#include "types.h"
using namespace cv;
using namespace std;
namespace posegraph
{
  
    //将eigen的vector3d数据结构转换成opencv的point3d数据结构----------------------------------------------
    vector<Point3d> eigenv3d_to_cvp3d(vector<Eigen::Vector3d> a) 
    {
      vector<Point3d> b;
      for(int k=0;k<a.size();k++)
      {
	b.push_back(cv::Point3d(a[k](0),a[k](1),a[k](2)));
      }
      return b;
    }
    //将opencv的3*3矩阵转换成eigen的3*3矩阵----------------------------------------------
   Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
   {
      Eigen::Matrix<double,3,3> M;

      M << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
	  cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
	  cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

      return M;
   }
    //将opencv的3维向量转换成eigen的三维向量----------------------------------------------
    Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector)
    {
      Eigen::Matrix<double,3,1> v;
      v << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);

      return v;
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
    void  pose_estimation_3d3d_quaeternion (const vector<Point3d>& pts1,const vector<Point3d>& pts2,Eigen::Matrix3d& R_, Eigen::Vector3d& t_)
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
	//return so3;
   }
   
   //将vos的位置变换到gts中去,这个函数主要用于freiburgh数据集
   //返回的是变换到vo坐标系下的真值坐标
   vector<pose> AlignPose(vector<pose> vos,vector<pose> gts)
   {
     vector<Eigen::Vector3d> vos_t,gts_t;
     for(int i=0;i<vos.size()/5;i++)//使用百分之20的位置进行algin操作
     {
       Eigen::Vector3d t_gt = gts[i].t;
       Eigen::Vector3d t_vo = vos[i].t;
       vos_t.push_back(t_vo);
       gts_t.push_back(t_gt);
    }
    Eigen::Vector3d tc;
    Eigen::Matrix3d Rc;
    pose_estimation_3d3d_quaeternion(eigenv3d_to_cvp3d(vos_t),eigenv3d_to_cvp3d(gts_t),Rc,tc);
    vector<pose> results;
    for(int i=0;i<vos.size();i++)
    {
      pose posei = gts[i];
      posei.t = Rc*posei.t+tc;
      results.push_back(posei);
    }
    return results;
  }
}
#endif
