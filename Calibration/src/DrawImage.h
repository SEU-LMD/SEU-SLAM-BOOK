#ifndef _DRAWIMG_
#define _DRAWIMG_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

#include <opencv2/opencv.hpp>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include "Eigen/Dense"


using namespace std;
//这个文件的作用是读取文件到程序中
namespace DrawImage
{  
   //opencv颜色设置
   //https://www.cnblogs.com/long5683/p/9757135.html
   vector<cv::Scalar> colors = {cv::Scalar(255,255,255), //cv::Scalar(255,255,255)白色
				cv::Scalar(28,255,255),//cv::Scalar(28,255,255) 黄色
				cv::Scalar(87,207,227),//cv::Scalar(87,207,227)香蕉黄
				cv::Scalar(0,0,255),//cv::Scalar(0,0,255) 红色
				cv::Scalar(80,127,255),//cv::Scalar(80,127,255)珊瑚红色
				cv::Scalar(0,255,0),//cv::Scalar(0,255,0) 绿色
				cv::Scalar(34,139,34),//cv::Scalar(34,139,34)深绿色
				cv::Scalar(255,0,0),//cv::Scalar(255,0,0) 蓝色
				cv::Scalar(84,46,84),//cv::Scalar(84,46,84)深蓝
				cv::Scalar(0,0,0)//cv::Scalar(0,0,0)黑色
				};
    
    //我们将输入的距离归一化到[min,max]
    vector<int> normalizeIntensity(multimap<float,Eigen::Vector2f> data, float min, float max)
    {
        vector<int> colors;
        float min_found = 10e5;
        float max_found = -10e5;
	multimap<float,Eigen::Vector2f>::iterator iter;
        for (iter = data.begin();iter!=data.end();iter++)
        {
            max_found = MAX(max_found, iter->first);
            min_found = MIN(min_found, iter->first);
        }

        for (iter = data.begin();iter!=data.end();iter++)
        {
            int color = round((iter->first - min_found) / (max_found - min_found) * (max - min) + min);
            colors.push_back(color);
        }
        return colors;
    }
    
    
    
    //img = 输入的灰色图像
    cv::Mat DrawPointCloud(pcl::PointCloud<pcl::PointXYZ>& clouds,cv::Mat img, Eigen::Matrix3f R,Eigen::Vector3f t,Eigen::Matrix3f K,pcl::PointCloud<pcl::PointXYZRGB>& colorpoints)
    {
        colorpoints.clear();
        //将灰度图转换成彩色图方便显示
        cv::Mat color_img;
        cv::cvtColor(img, color_img, CV_GRAY2BGR);//将输入的图像变成彩色图像
        //对输入的点云进行处理
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_withoutNAN(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(clouds, *cloud_withoutNAN, indices);
	
        //将点云变换到相机坐标系下
        /*vector<Eigen::Vector3f> pts_3d;
        for(int i=0;i<cloud_withoutNAN->size();i++)
        {
            pcl::PointXYZ point_3d = cloud_withoutNAN->points[i];
            pts_3d.push_back(Eigen::Vector3f(point_3d.x, point_3d.y, point_3d.z));
        }*/
        //使用方法 https://blog.csdn.net/zhanghaizhe/article/details/77528297
        //键值对，可以保存重复的value数值
        std::multimap<float,Eigen::Vector2f> drawpoints;
	int count_inrange = 0 ;
	for(int i=0;i<cloud_withoutNAN->points.size();i++)
	{
	  Eigen::Vector3f pts_3d(cloud_withoutNAN->points[i].x,cloud_withoutNAN->points[i].y,cloud_withoutNAN->points[i].z);
	  if(pts_3d(0)>=0.1)//只投影距离0.1米远的三维点云
	  {
	    float range = pts_3d(0)*pts_3d(0)+pts_3d(1)*pts_3d(1)+pts_3d(2)*pts_3d(2);
	    Eigen::Vector3f temp_point = R*pts_3d+t;
	    float u,v;

	    Eigen::Vector3f p_cam =K*temp_point;
	    u = p_cam(0)/p_cam(2);
	    v= p_cam(1)/p_cam(2);
	    
	    
	    if (u > 0 && u < img.cols && v > 0 && v < img.rows)
	    {
	      pcl::PointXYZRGB rgb_point;
	      rgb_point.x = pts_3d(0);
	      rgb_point.y = pts_3d(1);
	      rgb_point.z = pts_3d(2);
	      uchar b = color_img.at<cv::Vec3b>(v,u)[0];
	      uchar g = color_img.at<cv::Vec3b>(v,u)[1];
	      uchar r = color_img.at<cv::Vec3b>(v,u)[2];

	      uint32_t rgb = (static_cast<uint32_t>(r)<<16 | static_cast<uint32_t>(g)<<8 | static_cast<uint32_t>(b));
	      rgb_point.rgb = *reinterpret_cast<float*>(&rgb);
	      colorpoints.push_back(rgb_point);
	      drawpoints.insert(pair<float,Eigen::Vector2f>(range,Eigen::Vector2f(u,v)));
	      count_inrange++;
	  }
	}
      }
      
      //drawpoints属于的数据结构multimap，自动进行从小到大进行排序的
      std::multimap<float,Eigen::Vector2f>::iterator iter;
      int index = 0;
      for(iter=drawpoints.begin();iter!=drawpoints.end();iter++)
      {
	if(index>drawpoints.size()*0.9)
	{
	  multimap<float, Eigen::Vector2f>::iterator temp = iter ;
	  temp--;
	  drawpoints.erase(iter);
	  iter = temp;
	}
	index++;
      }    
      //cout<<"投影到图像上的激光点数量 = "<<drawpoints.size()<<endl;
    
    //然后对range进行归一化
    //序号对应drawpoints中的点，value=点对应的颜色
    vector<int> normalize_range = normalizeIntensity(drawpoints,0,colors.size()-1);
    //最后根据range的结果对颜色进行绘制
    index = 0;
    for(iter=drawpoints.begin();iter!=drawpoints.end();iter++)
    {
    	cv::Scalar col =  colors[normalize_range[index]];
	cv::circle(color_img,cv::Point(iter->second(0),iter->second(1)),1,col,-1);
        index++;
    }    
    
    //在图像左上角上绘制距离的图例
    for(int i=0;i<colors.size();i++)
    {
      cv::circle(color_img,cv::Point(10+i*15,10),6,colors[i],-1,8);
    }
    
    return color_img.clone();
  }
}
#endif

