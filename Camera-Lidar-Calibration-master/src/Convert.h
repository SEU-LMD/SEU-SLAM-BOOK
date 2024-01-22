#ifndef _CONVERT_
#define _CONVERT_
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
namespace Convert
{
  CvMat Convert_to_CvMat(Eigen::MatrixXd T)
    {
       cv::Mat T_cvmat;
       T_cvmat =  cv::Mat( T.rows(), T.cols(), CV_64F);
       for(int i=0;i<T.rows();i++)
       {
	 for(int j=0;j<T.cols();j++)
	 {
	    T_cvmat.at<double>(i,j) = T(i,j);
	    //cout<<T(i,j)<<" ";
	 }
	 //cout<<endl;
       }
       CvMat result = T_cvmat;
       //show_cvMat_instance(result,1);
       return result;
    }
    
    //将opencv 3*3矩阵转换成eigen的3*3矩阵
    Eigen::Matrix3d Convert_to_EigenMatrix(Mat H_cv)
    {
      Eigen::Matrix3d H_eigen;
      for(int i=0;i<3;i++)
      {
	for(int j=0;j<3;j++)
	{
	  H_eigen(i,j) = H_cv.at<float>(i,j);
	}
      }
      return H_eigen;
    }
    Eigen::Matrix<double,3,4> Convert_to_EigenMatrix(CvMat T)
    {
      Eigen::Matrix<double,3,4> T_eigen;
      Mat Ttemp = cvarrToMat(&T);
      for(int i=0;i<3;i++)
      {
	for(int j=0;j<4;j++)
	{
	  T_eigen(i,j) = Ttemp.at<double>(i,j);
	  //cout<<T_eigen(i,j)<<" ";
	}
	//cout<<endl;
      }
      //show_cvMat_instance(T,1);
      return T_eigen;
    }
}
#endif