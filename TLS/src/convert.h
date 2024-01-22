#ifndef _CONVERT_
#define _CONVERT_
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace cv;
namespace TLS
{
 //将旋转向量转换成旋转矩阵----------------------------------------------
  Eigen::Matrix3d AngleAxiTOMatrix(Eigen::Vector3d a)
  {
    Eigen::Matrix3d R;
    Eigen::AngleAxisd rotation_vector(a.norm(),a/a.norm());
    R = rotation_vector.toRotationMatrix();
    return R;
  }
  //旋转矩阵转旋转向量
  Eigen::Vector3d MatrixTOAngleAxis(Eigen::Matrix3d R)
  {
    Eigen::Vector3d r;
    Eigen::AngleAxisd rotation_vector(R);
    r = rotation_vector.angle()*rotation_vector.axis();
    return r;
  }
  //将eigen数据类型的R和t转换成opencv格式的4*4矩阵----------------------------------------------
  cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Vector3d &t)
  {
    cv::Mat cvMat = cv::Mat_<double>(3,4);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<double>(i,j)=R(i,j);
	    //cout<<
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<double>(i,3)=t(i);
    }
      
    return cvMat.clone();
  }
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
  //将opencv数据的三位向量转成Eigen的三维向量----------------------------------------------
  Eigen::Vector3d cvp3d_to_eigenv3d(Point3d a)
  {
    Eigen::Vector3d b(a.x,a.y,a.z);
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
}
#endif