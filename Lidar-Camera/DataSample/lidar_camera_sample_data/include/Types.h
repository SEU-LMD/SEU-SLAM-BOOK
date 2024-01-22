#ifndef _TYPES_
#define _TYPES_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;
//这个文件的作用是读取文件到程序中
namespace sample_data
{       
	template <typename T>
	cv::Mat toCvMat(const Eigen::Matrix<T,3,1> m,string type="float")
	{	
	    if(type=="float")
	    {
		cv::Mat cvMat(3,1,CV_32FC1);
		for(int i=0;i<3;i++)
			cvMat.at<float>(i)=float(m(i));

		return cvMat.clone();
	    }
	    else
	    {
		cv::Mat cvMat(3,1,CV_64FC1);
		for(int i=0;i<3;i++)
			cvMat.at<double>(i)=double(m(i));

		return cvMat.clone();
	    }
	}
	
	template <typename T>
	cv::Mat toCvMat(const Eigen::Matrix<T,3,3> m,string type= "float")
	{
	  if(type=="float")
	  {
	    cv::Mat cvMat(3,3,CV_32FC1);
	    for(int i=0;i<3;i++)
		for(int j=0; j<3; j++)
		    cvMat.at<float>(i,j)=float(m(i,j));

	    return cvMat.clone();
	  }
	  else
	  {
	    cv::Mat cvMat(3,3,CV_64FC1);
	    for(int i=0;i<3;i++)
		for(int j=0; j<3; j++)
		    cvMat.at<double>(i,j)=double(m(i,j));

	    return cvMat.clone();
	  }
	}
	
	template <typename T>
	cv::Mat toCvMat(const Eigen::Matrix<T,1,5> m,string type="float")
	{
	  if(type=="float")
	  {
	    cv::Mat cvMat(1,5,CV_32FC1);
	    for(int i=0;i<5;i++)
		  cvMat.at<float>(i)=float(m(i));
	    return cvMat.clone();
	  }
	  else
	  {
	    cv::Mat cvMat(1,5,CV_64FC1);
	    for(int i=0;i<5;i++)
		  cvMat.at<double>(i)=double(m(i));
	    return cvMat.clone();
	  }
	}
	
	template <typename T>
	cv::Mat toCvMat(const Eigen::Matrix<T,3,4> m,string type = "float")
	{
	  if(type=="float")
	  {
	    cv::Mat cvMat(3,4,CV_32FC1);
	    for(int i=0;i<3;i++)
		for(int j=0; j<4; j++)
		    cvMat.at<float>(i,j)=float(m(i,j));
	    return cvMat.clone();
	  }
	  else
	  {
	    cv::Mat cvMat(3,4,CV_32FC1);
	    for(int i=0;i<3;i++)
		for(int j=0; j<4; j++)
		    cvMat.at<double>(i,j)=(double)m(i,j);
	    return cvMat.clone();
	  }
	}
	
	struct CAMERAINFO
	{
	  cv::Size Imgsize;
	  Eigen::Matrix<float,3,4> P_l,P_r;//3*4矩阵
	  Eigen::Matrix<float,3,3> K_l,K_r;
	  Eigen::Matrix<float,1,5> Dis_l,Dis_r;
	  Eigen::Matrix3f Rcam_l,Rcam_r;
	  Eigen::Vector3f text;
	};
	
	Eigen::Matrix3f Vector_to_Matrix3f(vector<float> data)
	{
	  Eigen::Matrix3f res;
	  res(0,0) = (data[0]);res(0,1) = (data[1]);res(0,2) = (data[2]);
	  res(1,0) = (data[3]);res(1,1) = (data[4]);res(1,2) = (data[5]);
	  res(2,0) = (data[6]);res(2,1) = (data[7]);res(2,2) = (data[8]);
	  return res;
	}
	Eigen::Matrix<float,3,4> Vector_to_Matrix3_4f(vector<float> data)
	{
	  Eigen::Matrix<float,3,4> res;
	  res(0,0) = (data[0]);res(0,1) = (data[1]);res(0,2) = (data[2]);res(0,3) = (data[3]);
	  res(1,0) = (data[4]);res(1,1) = (data[5]);res(1,2) = (data[6]);res(1,3) = (data[7]);
	  res(2,0) = (data[8]);res(2,1) = (data[9]);res(2,2) = (data[10]);res(2,3) = (data[11]);
	  return res;
	}
	
	Eigen::Matrix<float,1,5> Vector_to_Matrix1_5f(vector<float> data)
	{
	  Eigen::Matrix<float,1,5> res;
	  res(0,0) = (data[0]);
	  res(0,1) = (data[1]);
	  res(0,2) = (data[2]);
	  res(0,3) = (data[3]);
	  res(0,4) = (data[4]);
	  return res;
	}
	
	Eigen::Matrix<double,4,4> To_Matrix44(Eigen::Matrix3d data)
	{
	  Eigen::Matrix<double,4,4> res;
	  res(0,0) = data(0,0);res(0,1) = data(0,1);res(0,2) = data(0,2);res(0,3) = 0;
	  res(1,0) = data(1,0);res(1,1) = data(1,1);res(1,2) = data(1,2);res(1,3) = 0;
	  res(2,0) = data(2,0);res(2,1) = data(2,1);res(2,2) = data(2,2);res(2,3) = 0;
	  res(3,0) = 0;	       res(3,1) = 0;	    res(3,2) = 0;        res(3,3) = 1;
	  return res;
	}
	
	Eigen::Matrix<double,3,4> To_Matrix34(Eigen::Matrix3d data)
	{
	  Eigen::Matrix<double,3,4> res;
	  res(0,0) = data(0,0);res(0,1) = data(0,1);res(0,2) = data(0,2);res(0,3) = 0;
	  res(1,0) = data(1,0);res(1,1) = data(1,1);res(1,2) = data(1,2);res(1,3) = 0;
	  res(2,0) = data(2,0);res(2,1) = data(2,1);res(2,2) = data(2,2);res(2,3) = 0;
	  return res;
	}
	
	string getImgType(int imgTypeInt)
	{
	    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)
	
	    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
				    CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
				    CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
				    CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
				    CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
				    CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
				    CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};
	
	    string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
				    "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
				    "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
				    "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
				    "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
				    "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
				    "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};
	
	    for(int i=0; i<numImgTypes; i++)
	    {
		if(imgTypeInt == enum_ints[i]) return enum_strings[i];
	    }
	    return "unknown image type";
	}
	 template <typename T>
       vector<T> &operator +(vector<T>  &s3,vector<T>  &s0)
       {
	 s3.insert(s3.end(),s0.begin(),s0.end());
	 return s3;	   
       }
	
	
}
#endif
