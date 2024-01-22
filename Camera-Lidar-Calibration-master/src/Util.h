#ifndef _UTIL__
#define _UTIL__

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <algorithm>
//用于删除目录
#include <dirent.h>  
//用于新建目录
#include <sys/stat.h>
#include <boost/concept_check.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include <opencv/cv.h>    
#include <opencv/highgui.h>    

#include "DealString.h"

using namespace std;
//using namespace cv;
namespace Util
{

  //对输入的数据进行直方图进行分析
  vector<pair<float,int>> Analyze_Data(vector<float> data,int num,bool mess = false)
  {
    vector<pair<float,int>> res;
    sort(data.begin(),data.end());//默认升序
    float min_value = data[0];
    float max_value = data[data.size()-1];
    float delta = (max_value-min_value)/num;
    float th = min_value+delta;
    int count = 0;
    for(int i=0;i<data.size();i++)
    {
      if(data[i]>th)
      {
	res.push_back(pair<float,int>(th,count));
	if(mess)
	  cout<<th<<" "<<count<<endl;
	count = 0;
	th = th+delta;
      }
      else
	count++;
    }
    return res;   
  }


  
  //从输入的范围内输出一个随机整数
  int produce_random(int min,int max)
  {
    int d = max - min + 1;
    return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
  }
  //随机生成num个范围在min到max的随机整数
  set<int> produce_randoms(int min,int max,int num)
  {
    set<int> result;
    for(int i=0;i<num;i++)
    {
      if(i==0)
      {
	result.insert(produce_random(min,max));
      }
      else
      {
	bool right_randomnum = false;
	while(!right_randomnum)
	{
	  int a = produce_random(min,max);
	  set<int>::iterator iter = result.find(a);
	  if(iter==result.end())
	  {
	    right_randomnum = true;
	    result.insert(a);
	  }
	}
      }
    }
    return result;
  }
  //此函数的作用是将in中的数据和target中的数据按照距离进行一一匹配，如果target中一个元素对应in中两个元素则返回的值是空的
  //返回的数据内容是in的内容，序号是target的序号
  vector<Point2f> match_closest(vector<Point2f> target,vector<Point2f> in)
  {
    vector<Point2f> result;
    set<int> match_point;
    for(int i=0;i<target.size();i++)
    {
      float minidistance=-1;
      float distance=0;
      int id_min=0;
      Point2f bestpoint={0,0};
      for(int p=0;p<in.size();p++)
      {
	distance =  (target[i].x-in[p].x)*(target[i].x-in[p].x)+(target[i].y-in[p].y)*(target[i].y-in[p].y);
	if((distance < minidistance)||(minidistance==-1))
	{
	  minidistance = distance;
	  bestpoint = in[p];
	  id_min = p;
	}
      }
      //set<int>::iterator iter;
      if(match_point.find(id_min)==match_point.end())
      {
	  result.push_back(bestpoint);
	  match_point.insert(id_min);
      }
      else
      {
	result.clear();
	break;
      }
    }
    return result;
  }
  
  //将两个图像合成为一个图像
  void imageJoinHorizon(cv::Mat &src1, cv::Mat &src2,cv::Mat &img_merge)  
  {  
    
      Size size(src1.cols + src2.cols, src1.rows);  
    
      cv::Mat outImg_left, outImg_right;  
    
      //img_merge.create(size, CV_MAKETYPE(src1.depth(), 1));  
      img_merge.create(size, src1.type());  
      img_merge = Scalar::all(0);  
      outImg_left = img_merge(Rect(0, 0, src1.cols, src1.rows));  
      outImg_right = img_merge(Rect(src1.cols, 0, src2.cols, src2.rows));  
      src1.copyTo(outImg_left);  
      src2.copyTo(outImg_right);  
  }  
  void imageJoinVertical(cv::Mat &src1, cv::Mat &src2,cv::Mat &img_merge)
  {
    Size size(src1.cols, src1.rows+src2.rows);  
    cv::Mat outImg_up, outImg_low;  
    img_merge.create(size, src1.type());  
    img_merge = Scalar::all(0);  
    
    outImg_up = img_merge(Rect(0, 0, src1.cols, src1.rows));  
    outImg_low = img_merge(Rect(0, src1.rows, src2.cols, src2.rows));  
    src1.copyTo(outImg_up);  
    src2.copyTo(outImg_low);  
  }
  
    vector<Eigen::Vector3f> PCA(vector<Eigen::Vector3f> orignal,Eigen::Matrix3f& R,Eigen::Vector3f& t)//得到的R和t是将输入的坐标变换到新的坐标系中的变换
    {
	Eigen::MatrixXf X(3,orignal.size());
	Eigen::Vector3f average;
	Eigen::Vector3f sum(0,0,0);
	for(int i=0;i<orignal.size();i++)
	{
	  sum = sum + orignal[i];
	}
	average = sum/float(orignal.size());
	for(int i=0;i<orignal.size();i++)
	{
	  Eigen::Vector3f a = orignal[i]-average;
	  X(0,i) = a[0];
	  X(1,i) = a[1];
	  X(2,i) = a[2];
	}
	
	Eigen::MatrixXf XTX = X*X.transpose();
	Eigen::EigenSolver<Eigen::MatrixXf> es1( XTX );
	Eigen::MatrixXcf evecs = es1.eigenvectors();
	Eigen::MatrixXcf evals = es1.eigenvalues();
	//cout<<"特征值 = "<<endl;
	//cout<<evals<<endl;
	//cout<<"特征向量 = "<<endl;
	//cout<<evecs<<endl;
	
	for(int i=0;i<3;i++)//使用特征向量更新旋转矩阵R
	{
	  R(0,i) = evecs.real()(0,i);
	  R(1,i) = evecs.real()(1,i);
	  R(2,i) = evecs.real()(2,i);
	}
	if(R.determinant()<0)
	{
	  R = R*-1.0;
	}
	//cout<<"R = "<<endl;
	//cout<<"R 行列式 = "<<R.determinant()<<endl;
	//cout<<R<<endl;
	//R = Eigen::Matrix3d::Identity();
	t = -R*average;
	vector<Eigen::Vector3f> output;
	for(int i=0;i<orignal.size();i++)
	{
	  Eigen::Vector3f coor_new = R*(orignal[i]-average);
	  output.push_back(coor_new);
	}
	return output;
    }
    
    //
    void helper(vector<int> n,int k,int level, vector<int>& out,vector<vector<int>>& res)
    {
	  if(out.size()==k)
	  {
		  res.push_back(out);
		  return;
	  }
	  for(int i=level;i<n.size();i++)
	  {
		  out.push_back(n[i]);
		  helper(n,k,i+1,out,res);
		  out.pop_back();//删除最后一个元素
	  }
    }
    vector<vector<int>> combine(vector<int> n,int k)
    {
	    vector<vector<int>> res;
	    vector<int> out;
	    helper(n,k,0,out,res);
	    return res;
    }
    
  
  class FindCorners
  {
    public:
      float min_x,min_y,max_x,max_y;
      cv::Mat img;
      cv::Size cornersize;
      std::vector<cv::Point2f> corners;
      FindCorners(cv::Mat img_,cv::Size cornersize_,float min_x_,float min_y_,float max_x_,float max_y_)
      {
	img = img_.clone();
	cornersize = cornersize_;
	min_x = min_x_;
	min_y = min_y_;
	max_x = max_x_;
	max_y = max_y_;
      }
      void Recurse_FindCorners(float p=0.01)
      {
	  min_x = min_x-img.cols*p;
	  if(min_x<0)
	    min_x = 0;
	  min_y = min_y-img.rows*p;
	  if(min_y<0)
	    min_y = 0;
	  max_x = max_x + img.cols*p;
	  if(max_x>img.cols-1)
	    max_x = img.cols-1;
	  max_y = max_y + img.rows*p;
	  if(max_y>img.rows-1)
	    max_y = img.rows-1;
	  
	  //3.然后提取特征点
	  cv::Rect r(min_x, min_y, max_x-min_x+1,max_y-min_y+1);//左上角坐标和宽高
	  cv::Mat imgtmp = img(r).clone();
	  
	  if(cv::findChessboardCorners(imgtmp,cornersize,corners))
	  {
	    cv::Mat image_gray;
	    //将彩色图像转换成灰色图像
	    cv::cvtColor(imgtmp,image_gray,CV_RGB2GRAY);
	    //找到亚像素级别的角点位置
	    cv::find4QuadCornerSubpix(image_gray,corners,Size(5,5));
	    //在图像中绘制出角点
	    for(int j=0;j<corners.size();j++)
	    {
	      corners[j].x = corners[j].x + min_x;
	      corners[j].y = corners[j].y + min_y;
	    }
	  }
	  else
	  {
	    if(p+0.005>1)
	      p = 1;
	    else
	      p = p+0.005;
	    Recurse_FindCorners(p);
	  }
      }
  };//类结束
  
  
}

#endif