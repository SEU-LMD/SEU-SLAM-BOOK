#include <stdio.h>    
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointField.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "Segment.h"

#include <opencv/cv.h>    
#include <opencv2/imgproc.hpp>
#include <opencv/highgui.h>    
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<opencv2/flann/miniflann.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> //为了求特征值

#include "DealString.h"
#include "FileIO.h"
#include "Util.h"
#include "CameraCalibrationNew.h"
#include "Segment.h"
#include "ICP.h"
#include "CornerDetAC.h"
#include "ChessboradStruct.h"
#include "DrawImage.h"
#include "PCLlib.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#define NB 3
#define corner_height 3
#define corner_width 5
using namespace std;

int chcount = 0;//一张图片被选择的次数
cv::Mat img,pre_img,org_img;

cv::Mat board_img[3];
cv::Size cornersize(corner_height,corner_width);//宽,高
int numofcorner = 0;//一个标定版上的角点个数
cv::Rect rect(0, 0, 0, 0);//前两个坐标是坐上角的原点，后两个坐标是col 和row
int min_x_th =0;
int min_y_th =0;
int max_x_th =0;
int max_y_th =0;

vector<cv::Point2f> pre_first_edge_points;//first edge_points
vector<vector<cv::Point2f>> current_corners,pre_corners;
vector<vector<int>> current_edge_indx;
vector<Point2f> current_board_centroid;//每个标定版的中心点

struct BOARD
{
    vector<cv::Point2f> corners;
    vector<cv::Point2f> orderd_corners;
    vector<int> edge_indx;//4个元素
    vector<int> ordered_edge_indx;//4个元素
    Point2f centroid;
    int first_edge_indx;//决定了ordered_edge_indx的顺序
    
    //深拷贝
    BOARD operator=(BOARD& BOARDTmp)
    {

        corners = BOARDTmp.corners;
        orderd_corners = BOARDTmp.orderd_corners;
	edge_indx = BOARDTmp.edge_indx;
	ordered_edge_indx = BOARDTmp.ordered_edge_indx;
	centroid = BOARDTmp.centroid;
	first_edge_indx = BOARDTmp.first_edge_indx;
        return *this;
    };
};
struct BOARD cur_boards[3],pre_boards[3];
vector<int> reorder_info={0,0,0};
int click_count =-1;
map<int,vector<cv::Point2f>> all_corners;//所有能提取到有效内角点的序号
vector<int> camera_cal_frame;//用于标定相机的frame的序号
map<int,vector<Eigen::Vector4f>> camera_planes;//提取得到的相机平面参数坐标,顺序是右、左、下

vector<int> lidar_valid_frame;//能够有效提取出激光三个平面的frame序号
int useCamera,useLidar,UseExtraData;
int waittime;
bool isMouseCallback = true;
bool detectfirstframe = false;

void Update_Rect_Th(cv::Rect r)
{
  min_x_th = r.x;
  min_y_th = r.y;
  max_x_th = r.width+r.x;
  max_y_th = r.height+r.y;
}
bool Update_Rect(int x_max,int x_min,int y_max,int y_min,cv::Size img_size)
{
    bool res = false;
    if((rect.x==0)&&(rect.y==0)&&(rect.width==0)&&(rect.width==0))
    {
		rect.x = x_min;
		rect.y = y_min;
		rect.width = x_max - x_min;
		rect.height = y_max - y_min;
		res = true;
    }
    else
    {
		int max_x_org = rect.x+rect.width;
		int max_y_org = rect.y+rect.height;
		if(x_min<rect.x)
		{
			rect.x = x_min;
			if((min_x_th-rect.x)>0.01*img_size.width)
			{
				res = true;
			}
		}
		if(y_min<rect.y)
		{
			rect.y = y_min;
			if((min_y_th-rect.y)>0.01*img_size.height)
				res = true;
		}
		
		if(max_x_org<x_max)
		{
			rect.width = x_max-rect.x;
			if((x_max-max_x_th)>0.01*img_size.width)
				res = true;
		}
		else
		rect.width = max_x_org-rect.x;

		
		if(max_y_org<y_max)
		{
			rect.height = y_max-rect.y;
			if((y_max-max_y_th)>0.01*img_size.height)
				res  = true;
		}
		else
			rect.height = max_y_org-rect.y;
    }
    
    if(res)
      Update_Rect_Th(rect);
    return res;
}

double R_error()
{
	
}


vector<cv::Point3d> ConvertToDouble(vector<cv::Point3f> data)
{
  vector<cv::Point3d> res;
  for(int i=0;i<data.size();i++)
  {
    cv::Point3d temp;
    temp.x = data[i].x;
    temp.y  = data[i].y;
    temp.z  = data[i].z;
    res.push_back(temp);
  }
  return res;
}

vector<cv::Point2d> ConvertToDouble(vector<cv::Point2f> data)
{
  vector<cv::Point2d> res;
  for(int i=0;i<data.size();i++)
  {
    cv::Point2d temp;
    temp.x = data[i].x;
    temp.y  = data[i].y;
    res.push_back(temp);
  }
  return res;
}



Eigen::Vector4f Calculate_Planar_Model(vector<Point3f>& data,float th = 0.02)
{
    pcl::PointCloud<pcl::PointXYZI> cloudin;
    for(int i=0;i<data.size();i++)
    {
      pcl::PointXYZI temp;
      temp.x = data[i].x;
      temp.y = data[i].y;
      temp.z = data[i].z;
      cloudin.points.push_back(temp);
    }
    
    
    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloudin.makeShared()));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
    ransac.setDistanceThreshold (th);
    ransac.computeModel();
    ransac.getInliers(inliers);
    Eigen::VectorXf planar_coefficients;
    ransac.getModelCoefficients(planar_coefficients);
    
    
    Eigen::Vector4f res(planar_coefficients(0),planar_coefficients(1),planar_coefficients(2),planar_coefficients(3));
    return res;
}


//返回的序号是pre，内容是curr中的序号
vector<int> findClosest(vector<Eigen::Vector2f> curr,vector<Eigen::Vector2f> pre)
{
  vector<int> res;
  for(int i=0;i<pre.size();i++)
  {
    float mindist = 100000;
    int minindx = 0;
    for(int j=0;j<curr.size();j++)
    {
      float dist = (pre[i]-curr[j]).norm();
      if(mindist>dist)
      {
	mindist = dist;
	minindx = j;
      }
    }
    res.push_back(minindx);
  }
  return res;
}


//降序排列
struct CmpByValue {  
  bool operator()(const pair<int,float>& lhs, const pair<int,float>& rhs) {  
    return lhs.second > rhs.second;  
  }  
};
//升序
struct CmpByValueAscend {  
  bool operator()(const pair<int,float>& lhs, const pair<int,float>& rhs) {  
    return lhs.second < rhs.second;  
  }  
};

float euclideanDist(cv::Point2f& a, cv::Point2f& b)
{
    cv::Point2f diff = a - b;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}



vector<int> SearchClosest(vector<cv::Point2f>& source_points,vector<cv::Point2f>& target_points)
{
    //生成Mat对象
    cv::Mat source = cv::Mat(source_points).reshape(1);
    source.convertTo(source,CV_32F);

    cv::flann::KDTreeIndexParams indexParams(2); 
    cv::flann::Index kdtree(source, indexParams); //kd树索引建立完毕

    int quervNum = 1;
    vector<float> vecQuery(2);
    vector<int> vecIndex(quervNum),source_index;
    vector<float> vecDist(quervNum);
    cv::flann::SearchParams params(32);

    //计算最邻点
    for(int i=0;i<target_points.size();i++)
    {
        vecQuery.clear();
        vecQuery = { target_points[i].x, target_points[i].y};
        kdtree.knnSearch(vecQuery, vecIndex, vecDist, quervNum, params);
        source_index.push_back(vecIndex[0]);  
    }
    
    //查询是否有重复索引值
    vector<int> temp_index(source_index);
    sort(temp_index.begin(),temp_index.end());
    for(int i=1;i<temp_index.size();i++)
    {
	//cout<<temp_index[i]<<endl;
        if(temp_index[i-1]==temp_index[i])
            source_index.clear();
    }
        
    return source_index;
}

float DistToLine(cv::Point2f p,cv::Point2f a,cv::Point2f b)
{
  float A,B,C;
  A = a.y-b.y;
  B = b.x-a.x;
  C = a.x*b.y-a.y*b.x;
  float dist = abs(A*p.x+B*p.y+C)/sqrt(A*A+B*B);
  return dist;
}

bool Update_Ordered_Info(int indx_bd)
{
	//更新标定版上的ordered_edge_indx
	cur_boards[indx_bd].ordered_edge_indx.clear();
	vector<pair<int,float>> dist_map;
	Point2f firstpoint = cur_boards[indx_bd].corners[cur_boards[indx_bd].first_edge_indx];//取出选中的点
	for(int i=0;i<4;i++)
	{
		Point2f tmppoint = cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[i]];
		float dist = euclideanDist(firstpoint,tmppoint);
		dist_map.push_back(pair<int,float>(cur_boards[indx_bd].edge_indx[i],dist));
	}
	sort(dist_map.begin(), dist_map.end(), CmpByValue()); 
	cur_boards[indx_bd].ordered_edge_indx.push_back(cur_boards[indx_bd].first_edge_indx);
	cur_boards[indx_bd].ordered_edge_indx.push_back(dist_map[2].first);
	cur_boards[indx_bd].ordered_edge_indx.push_back(dist_map[1].first);
	cur_boards[indx_bd].ordered_edge_indx.push_back(dist_map[0].first);//最远的距离
	
	//然后更新orderd_corners
	//根据最外侧的四个角点生成其他的24个角点
	Point2f A = cur_boards[indx_bd].corners[cur_boards[indx_bd].ordered_edge_indx[0]];
	Point2f B = cur_boards[indx_bd].corners[cur_boards[indx_bd].ordered_edge_indx[1]];
	Point2f C = cur_boards[indx_bd].corners[cur_boards[indx_bd].ordered_edge_indx[2]];
	Point2f D = cur_boards[indx_bd].corners[cur_boards[indx_bd].ordered_edge_indx[3]];
	Point2f vector1 = B-A;
	Point2f vector2 = D-C;
	if((vector2.x*vector1.x+vector2.y*vector1.y)<0)
	{
		Point2f tmp = C;
		C = D;
		D = tmp;
		int indx_tmp = cur_boards[indx_bd].ordered_edge_indx[3];
		cur_boards[indx_bd].ordered_edge_indx[3] = cur_boards[indx_bd].ordered_edge_indx[2];
		cur_boards[indx_bd].ordered_edge_indx[2] = indx_tmp;
	}
	//第一个点是白色，依次是红绿蓝
	cv::circle( img, A, 3,cv::Scalar(255,255,255) ,CV_FILLED, CV_AA, 0 );//红色
	cv::circle( img, B, 3,cv::Scalar(0,0,255) ,CV_FILLED, CV_AA, 0 );//红色
	cv::circle( img, C, 3,cv::Scalar(0,255,0) ,CV_FILLED, CV_AA, 0 );//绿色
	cv::circle( img, D, 3,cv::Scalar(255,0,0) ,CV_FILLED, CV_AA, 0 );//蓝色
	
	cur_boards[indx_bd].orderd_corners.clear();
	
	vector<pair<int,float>> start_dist_order,end_dist_order;
	for(int j=0;j<cur_boards[indx_bd].corners.size();j++)
	{
		float dist= DistToLine(cur_boards[indx_bd].corners[j],A,C);
		start_dist_order.push_back(pair<int,float>(j,dist));
		dist = DistToLine(cur_boards[indx_bd].corners[j],B,D);
		end_dist_order.push_back(pair<int,float>(j,dist));
	}
	sort(start_dist_order.begin(),start_dist_order.end(),CmpByValueAscend());//升序
	sort(end_dist_order.begin(),end_dist_order.end(),CmpByValueAscend());//升序
	
	vector<pair<int,float>> start_cd,end_cd;
	for(int i=0;i<cornersize.height;i++)
	{
		float dis = euclideanDist(A,cur_boards[indx_bd].corners[start_dist_order[i].first]);
		start_cd.push_back(pair<int,float>(start_dist_order[i].first,dis));
		dis = euclideanDist(B,cur_boards[indx_bd].corners[end_dist_order[i].first]);
		end_cd.push_back(pair<int,float>(end_dist_order[i].first,dis));
	}
	sort(start_cd.begin(),start_cd.end(),CmpByValueAscend());//升序
	sort(end_cd.begin(),end_cd.end(),CmpByValueAscend());//升序
	
	vector<Point2f> search_points;
	for(int row=0;row<cornersize.height;row++)
	{
		Point2f start = cur_boards[indx_bd].corners[start_cd[row].first];
		Point2f end = cur_boards[indx_bd].corners[end_cd[row].first];
		for(int col=0;col<cornersize.width;col++)
		{
			Point2f delta = (end-start)/(float)(cornersize.width-1);
			search_points.push_back(delta*col+start);
		}
	}
      //然后设置最近邻搜索
      vector<int> res = SearchClosest(cur_boards[indx_bd].corners,search_points);
      if(res.size()!=0)
      {
			cur_boards[indx_bd].orderd_corners.clear();
			for(int i=0;i<res.size();i++)
			{
			Point2f tmp = cur_boards[indx_bd].corners[res[i]];
			cur_boards[indx_bd].orderd_corners.push_back(tmp);
			}
      }
      else
      {
	        return false;
      }
      
      return true;
}

//根据重心更新当前标定版的顺序 然后再确定当前frame标定版的first_edge_indx序号
void TrackCentroid()
{
  /*
  //首先确定标定板的序号，下面的代码是使用track方式追踪中心点
  vector<Eigen::Vector2f> source,target;
  for(int i=0;i<3;i++)
  {
    target.push_back(Eigen::Vector2f(pre_boards[i].centroid.x,pre_boards[i].centroid.y));
    source.push_back(Eigen::Vector2f(cur_boards[i].centroid.x,cur_boards[i].centroid.y));
  }
  reorder_info.clear();
  reorder_info = findClosest(source,target);
  //cout<<"reorder_info = "<<endl;
  //cout<<reorder_info[0]<<endl;
  //cout<<reorder_info[1]<<endl;
  //cout<<reorder_info[2]<<endl;
  struct BOARD board0 = cur_boards[reorder_info[0]];
  struct BOARD board1 = cur_boards[reorder_info[1]];
  struct BOARD board2 = cur_boards[reorder_info[2]];
  cur_boards[0] = board0;
  cur_boards[1] = board1;
  cur_boards[2] = board2;
  */
  reorder_info.clear();
  int indx3 = -1;
  float max_dist = -1;
  for(int i=0;i<3;i++)
  {
    if(cur_boards[i].centroid.y>max_dist)
    {
      max_dist = cur_boards[i].centroid.y;
      indx3 = i;
    }
  }
  reorder_info[2] = indx3;
  
  vector<int> rest;
  for(int i=0;i<3;i++)
  {
    if(i!=indx3)
    {
      rest.push_back(i);
    }
  }
  if(cur_boards[rest[0]].centroid.x>cur_boards[rest[1]].centroid.x)
  {
    reorder_info[0] = rest[0];
    reorder_info[1] = rest[1];
  }
  else
  {
    reorder_info[0] = rest[1];
    reorder_info[1] = rest[0];
  }
  struct BOARD board0 = cur_boards[reorder_info[0]];
  struct BOARD board1 = cur_boards[reorder_info[1]];
  struct BOARD board2 = cur_boards[reorder_info[2]];
  cur_boards[0] = board0;
  cur_boards[1] = board1;
  cur_boards[2] = board2;  
  
  
  vector<Point2f> curr_first_edge_points;
  vector<unsigned char> status;
  vector<float> error;
  cv::calcOpticalFlowPyrLK(pre_img,img,pre_first_edge_points,curr_first_edge_points,status,error,cv::Size(21,21),5);
  
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    //if(status[indx_bd]==1)
    //  cout<<" good"<<endl;
    //else
    // cout<<" bad"<<endl;
    cv::circle( img, curr_first_edge_points[indx_bd], 6,cv::Scalar(28,255,255));
    float min_dist = 10000;
    int min_indx = -1;
    vector<pair<int,float>> map_dist;
    for(int i=0;i<4;i++)
    {
      float dist = euclideanDist(curr_first_edge_points[indx_bd],cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[i]]);
      map_dist.push_back(pair<int,float>(cur_boards[indx_bd].edge_indx[i],dist));
    }
    sort(map_dist.begin(),map_dist.end(),CmpByValue());//降序
    cur_boards[indx_bd].first_edge_indx = map_dist[3].first;
  }
  
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    pre_first_edge_points[indx_bd] = cur_boards[indx_bd].corners[cur_boards[indx_bd].first_edge_indx];
  }
  /*
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    vector<pair<int,float>> map_y;
    for(int i=0;i<4;i++)
    {
      Point2f point = cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[i]];
      map_y.push_back(pair<int,float>(cur_boards[indx_bd].edge_indx[i],point.y));
    }
    sort(map_y.begin(),map_y.end(),CmpByValue());//降序
    int cd[2]={map_y[2].first,map_y[3].first};
    int min_indx = -1;
    if(cur_boards[indx_bd].corners[cd[0]].x<cur_boards[indx_bd].corners[cd[1]].x)
      min_indx = cd[0];
    else
      min_indx = cd[1];
    cur_boards[indx_bd].first_edge_indx = min_indx;
  }*/
  /*
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    vector<Eigen::Vector3f> curr_edge_points,pre_edge_points;
    for(int i=0;i<cur_boards[indx_bd].corners.size();i++)
    {
      Point2f cur_point = cur_boards[indx_bd].corners[i];
      curr_edge_points.push_back(Eigen::Vector3f(cur_point.x,cur_point.y,0));
      Point2f pre_point = pre_boards[indx_bd].corners[i];
      pre_edge_points.push_back(Eigen::Vector3f(pre_point.x,pre_point.y,0));
    }
    Eigen::Matrix3f R_pca_cur,R_pca_pre;
    Eigen::Vector3f t_pca_cur,t_pca_pre;
    vector<Eigen::Vector3f> cur_pca_points= Util::PCA(curr_edge_points,R_pca_cur,t_pca_cur);
    vector<Eigen::Vector3f> pre_pca_points= Util::PCA(pre_edge_points,R_pca_pre,t_pca_pre);
    
    Point2f pre_first_edge_pca_point;
    pre_first_edge_pca_point.x = pre_pca_points[pre_boards[indx_bd].first_edge_indx][0];
    pre_first_edge_pca_point.y = pre_pca_points[pre_boards[indx_bd].first_edge_indx][1];
    //float norm_pre = sqrt(pre_first_edge_pca_point.x*pre_first_edge_pca_point.x+pre_first_edge_pca_point.y*pre_first_edge_pca_point.y);
    float min_dist =10000;
    int min_indx = -1;
    for(int j=0;j<4;j++)
    {
      //然后计算向量的角度
      Point2f cur_edge_pca_point;
      cur_edge_pca_point.x = cur_pca_points[cur_boards[indx_bd].edge_indx[j]][0];
      cur_edge_pca_point.y = cur_pca_points[cur_boards[indx_bd].edge_indx[j]][1];
      //float norm_cur = sqrt(cur_edge_pca_point.x*cur_edge_pca_point.x+cur_edge_pca_point.y*cur_edge_pca_point.y);
      float dist = euclideanDist(pre_first_edge_pca_point,cur_edge_pca_point);
      //float angle = 180.0*acos((pre_first_edge_pca_point.x*cur_edge_pca_point.x+pre_first_edge_pca_point.y*cur_edge_pca_point.y)/(norm_pre*norm_cur))/PI;
      if(min_dist>dist)
      {
	min_dist = dist;
	min_indx = cur_boards[indx_bd].edge_indx[j];
      }
    }
    cur_boards[indx_bd].first_edge_indx = min_indx;
    
    //在当前frame上绘制小坐标系的结果
    Eigen::Vector3f xaxis(30,0,0);
    Eigen::Vector3f yaxis(0,30,0);
    Eigen::Vector3f origin(0,0,0);
    Eigen::Vector3f xaxis_orig = R_pca_cur.transpose()*(xaxis-t_pca_cur);
    Eigen::Vector3f yaxis_orig = R_pca_cur.transpose()*(yaxis-t_pca_cur);
    Eigen::Vector3f orig = R_pca_cur.transpose()*(origin-t_pca_cur);
    cv::arrowedLine(img,Point2f(orig[0],orig[1]),Point2f(xaxis_orig[0],xaxis_orig[1]),cv::Scalar(0,0,255),2);
    cv::arrowedLine(img,Point2f(orig[0],orig[1]),Point2f(yaxis_orig[0],yaxis_orig[1]),cv::Scalar(0,255,0),2);
  }
  */
  //目前存在风险，如果距离较远如何处理
  //确定当前frame标定版的first_edge_indx序号
  /*
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    Point2f pre_first_edge_point = pre_boards[indx_bd].corners[pre_boards[indx_bd].first_edge_indx];
    float min_dist = 10000;
    int min_indx = -1;
    for(int j=0;j<4;j++)
    {
      float dist = euclideanDist(pre_first_edge_point,cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[j]]);
      if(min_dist>dist)
      {
	min_dist = dist;
	min_indx = cur_boards[indx_bd].edge_indx[j];
      }
    }
    cur_boards[indx_bd].first_edge_indx = min_indx;
  }*/
  //第二种的first_edge_indx 检测方式
  /*
  for(int indx_bd=0;indx_bd<3;indx_bd++)
  {
    Point2f pre_centroid = pre_boards[indx_bd].centroid;
    Point2f pre_first_edge_point = pre_boards[indx_bd].corners[pre_boards[indx_bd].first_edge_indx];
    Point2f pre_vector = pre_first_edge_point- pre_centroid;
    float norm_pre = sqrt(pre_vector.x*pre_vector.x+pre_vector.y*pre_vector.y);
    Point2f cur_centroid = cur_boards[indx_bd].centroid;
    float min_angle =10000;
    int min_indx = -1;
    for(int j=0;j<4;j++)
    {
      //然后计算向量的角度
      Point2f cur_edge_point = cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[j]];
      Point2f cur_vector = cur_edge_point - cur_centroid;
      float norm_cur = sqrt(cur_vector.x*cur_vector.x+cur_vector.y*cur_vector.y);
      float angle = 180.0*acos((cur_vector.x*pre_vector.x+cur_vector.y*pre_vector.y)/(norm_pre*norm_cur))/PI;
      if(min_angle>angle)
      {
	min_angle = angle;
	min_indx = cur_boards[indx_bd].edge_indx[j];
      }
    }
    cur_boards[indx_bd].first_edge_indx = min_indx;
  }*/
  
}

void on_mouse( int event, int x, int y, int flags, void* ustc)    
{
	if(isMouseCallback)
	{
      if(event == CV_EVENT_LBUTTONDOWN)//左键按下
      {
	click_count++;
	if(click_count==3)
	{
	  click_count = 0;
	  img = org_img.clone();
	  for (int indx_bd = 0; indx_bd < 3; indx_bd++)
	  {
		//更新标定板中的corner centroid 和 edge_indx
		for (int i = 0; i < cur_boards[indx_bd].corners.size(); i++)
		{
			if(indx_bd==0)
			  cv::circle( img, cur_boards[indx_bd].corners[i], 1,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			if(indx_bd==1)
			  cv::circle( img, cur_boards[indx_bd].corners[i], 1,Scalar(0, 255, 0) ,CV_FILLED, CV_AA, 0 );
			if(indx_bd==2)
			  cv::circle( img, cur_boards[indx_bd].corners[i], 1,Scalar(0, 0, 255) ,CV_FILLED, CV_AA, 0 );
		}  
		for(int i=0;i<4;i++)
		{
		    Point2f tmp = cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[i]];
		    if(indx_bd==0)
		      cv::circle( img, tmp, 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );//蓝色
		    if(indx_bd==1)
		      cv::circle( img, tmp, 3,Scalar(0, 255, 0) ,CV_FILLED, CV_AA, 0 );//绿色
		    if(indx_bd==2)
		      cv::circle( img, tmp, 3,Scalar(0, 0, 255) ,CV_FILLED, CV_AA, 0 );//红色
		}
	  }
	}
	
	//更新标定板中的first_edge_indx和标定板的交换信息reorder_info
	cv::Point2f clickpoint = cv::Point2f((float)x,(float)y); 
	float min_dist = 100000;
	int min_board_indx = -1;
	int min_corner_indx = -1;
	for(int indx_bd=0;indx_bd<3;indx_bd++)
	{
	    for(int i=0;i<4;i++)
	    {
			int edge_indx = cur_boards[indx_bd].edge_indx[i];
			float dist = euclideanDist(cur_boards[indx_bd].corners[edge_indx],clickpoint);
			if(min_dist>dist)
			{
				min_board_indx = indx_bd;
				min_corner_indx = edge_indx;
				min_dist = dist;
			}
	    }
	}//最近的点检测完毕
	cur_boards[min_board_indx].first_edge_indx = min_corner_indx;
	reorder_info[click_count] = min_board_indx;
	cout<<"min_board_indx = "<<min_board_indx<<" min_corner_indx = "<<min_corner_indx<<endl;
	cv::circle( img, cur_boards[min_board_indx].corners[min_corner_indx], 3,Scalar(255, 255, 255) ,CV_FILLED, CV_AA, 0 );
	//更新ordered_edge_indx和ordered_corner
	bool valid = Update_Ordered_Info(min_board_indx);//根据点的顺序重新对board进行排序
	if(valid)
	{
	    for(int i=0;i<cur_boards[min_board_indx].orderd_corners.size();i++)
	    {
	      Point2f tmp = cur_boards[min_board_indx].orderd_corners[i];
	      if((i==0)||(i==corner_height-1)||(i==corner_height*corner_width-corner_height)||(i==corner_height*corner_width-1))
		cv::putText(img,std::to_string(i+click_count*numofcorner),tmp,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(80,127,255),1,1);
	    }
	    cout<<"Update_Ordered_Info Success"<<endl;
	}
	else
	{
	  cout<<"Update_Ordered_Info Fails"<<endl;
	}
	
	cv::imshow ( "Image", img);
      }
  }
}

Eigen::Vector4f Calculate_Planar_Model(pcl::PointCloud<pcl::PointXYZI> cloudin,float th = 0.02)
{
    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloudin.makeShared()));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
    ransac.setDistanceThreshold (th);//与平面距离小于th的点做为局内点考虑
    ransac.computeModel();
    ransac.getInliers(inliers);//存储估计得到点局内点
    Eigen::VectorXf planar_coefficients;
    ransac.getModelCoefficients(planar_coefficients);//获取平面参数
    //cout<<"平面模型= "<<planar_coefficients[0]<<" "<<planar_coefficients[1]<<" "<<planar_coefficients[2]<<" "<<planar_coefficients[3]<<endl;
    //for(int i=0;i<inliers.size();i++)//得到平面ransac估计之后的内点数据
    //{
      //cout<<inliers[i]<<" "<<cloudin.points.size()<<endl;
    //  outcloud.push_back(cloudin.points[inliers[i]]);
    //}
    Eigen::Vector4f res(planar_coefficients(0),planar_coefficients(1),planar_coefficients(2),planar_coefficients(3));
    return res;
}

struct CloudInfo
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  Eigen::Vector3f center;
  Eigen::Vector4f plane_param;
  set<int> vertical_indx;//占了多少线的激光
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration");
	ros::NodeHandle n;
	string path_root;
	string lidar_config_path;
	string path_Ext;
	n.getParam("/calibration/root", path_root);//根目录
	n.getParam("/calibration/path_Extrinsic", path_Ext);//外参保存目录
	n.getParam("/calibration/lidar_config_path", lidar_config_path);
	n.getParam("/calibration/Camera", useCamera);//是否标定相机
	n.getParam("/calibration/Lidar", useLidar);//
	n.getParam("/calibration/wait", waittime);
	n.getParam("/calibration/UseExtraData", UseExtraData);//是否使用测评数据
	string path_coor = path_root+"/3D.txt";
	string path_coor_planar = path_root+"/3D_planar.txt";
	string path_image = path_root+"/names.txt";
	string path_cal_res = path_root+"/LCamera.txt";

	
	vector<string> imgnames = FileIO::ReadTxt2String(path_image,false);
	//下面开始进行激光三维激光点云的提取
	map<int,vector<Eigen::Vector4f>> lidar_planes;
	map<int,vector<Eigen::Vector4f>> cam_planes;//根据world plane和相机的位姿矩阵得到相机坐标系下的平面坐标，顺序是右左下
	

	cv::Mat distParameter;//摄像机的畸变系数
	cv::Mat intrincMatrix;
	Eigen::Matrix3d Rcl_;
	Eigen::Vector3d Tcl_;
	Mat mapx,mapy;
	if(useCamera==1)
	{
		vector<vector<float>> coor = FileIO::ReadTxt2Float(path_coor,false);
		vector<vector<float>> coor_planar = FileIO::ReadTxt2Float(path_coor_planar,false);
		cvNamedWindow("Image");
		cvSetMouseCallback( "Image", on_mouse, NULL );//设置当前窗口鼠标的回调函数on_mouse!!!!!!!!!!!
		numofcorner = cornersize.height*cornersize.width;//一块标定版上的角点个数
		Size image_size;//图像大小
		for(int img_indx=0;img_indx<imgnames.size();img_indx++)//imgnames.size()
		{
			vector<string> name = read_format(imgnames[img_indx]," ");
			org_img=cv::imread(path_root+"/leftImg/left_"+name[1]+".png",1);
			img = org_img.clone();
			//在图像上方标注第几张图像 n/all
			cv::putText(img,std::to_string(img_indx+1)+"/"+std::to_string(imgnames.size()),cv::Point2f(10,30),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(155,155,155),3);
			//图像大小初始化
			if(img_indx==0)
			{
				image_size.width = img.cols;
				image_size.height = img.rows;
			}

			vector<Point> corners_p;//存储找到的角点
			std::vector<cv::Mat> chessboards;
			CornerDetAC corner_detector(img);
			ChessboradStruct chessboardstruct;
			Corners corners_s;
			corner_detector.detectCorners(img, corners_p, corners_s, 0.05);//检测棋盘格角点
			ImageChessesStruct ics;
			chessboardstruct.chessboardsFromCorners(corners_s, chessboards, 0.6);
			//一共检测到了三个标定版
			if((chessboards.size()==3)&&(chessboards[0].cols*chessboards[0].rows==numofcorner)&&(chessboards[1].cols*chessboards[1].rows==numofcorner)&&(chessboards[2].cols*chessboards[2].rows==numofcorner))//表示有效frame
			{
				//遍历某个标定版
				int max_x = -100000;
				int min_x = 100000;
				int max_y = -100000;
				int min_y = 100000;
				//遍历当前帧，检测棋盘格并标注
				for (int indx_bd = 0; indx_bd < 3; indx_bd++)
				{
				    cur_boards[indx_bd].corners.clear();//角点清零
					cur_boards[indx_bd].orderd_corners.clear();
					cur_boards[indx_bd].edge_indx.clear();
					cur_boards[indx_bd].ordered_edge_indx.clear();
					//更新标定板中的corner centroid、 edge_indx和centroid
					Point2f acc(0,0);
					//遍历棋盘格每个角点，对不同标定板的角点用不同颜色标注，并记录顶点序号
					for (int i = 0; i < chessboards[indx_bd].rows; i++)
					{
						for (int j = 0; j < chessboards[indx_bd].cols; j++)
						{
							//获取角点序号、、、、
							int d = chessboards[indx_bd].at<int>(i, j);
							//更新标定板范围
							cv::Point2f point(corners_s.p[d].x, corners_s.p[d].y);
							if(max_x<(int)point.x)
							    max_x = point.x;
							if(min_x>(int)point.x)
							    min_x = point.x;
							if(max_y<(int)point.y)
							    max_y = point.y;
							if(min_y>(int)point.y)
							    min_y = point.y;
							cur_boards[indx_bd].corners.push_back(point);
							//计算所有角点坐标和
							acc  = acc + point;
							//不同标定板角点标注
							if(indx_bd==0)
							    cv::circle( img, point, 1,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
							if(indx_bd==1)
							    cv::circle( img, point, 1,Scalar(0, 255, 0) ,CV_FILLED, CV_AA, 0 );
							if(indx_bd==2)
							    cv::circle( img, point, 1,Scalar(0, 0, 255) ,CV_FILLED, CV_AA, 0 );
							
							//cv::putText(img,std::to_string(j+i*chessboards[indx_bd].cols),point,cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(112,112,112),1,1);
							//记录顶点序号
							if((i==0)&&(j==0))
							    cur_boards[indx_bd].edge_indx.push_back(j+i*chessboards[indx_bd].cols);
							if((i==0)&&(j==chessboards[indx_bd].cols-1))
							    cur_boards[indx_bd].edge_indx.push_back(j+i*chessboards[indx_bd].cols);
							if((i==chessboards[indx_bd].rows-1)&&(j==0))
							    cur_boards[indx_bd].edge_indx.push_back(j+i*chessboards[indx_bd].cols);
							if((i==chessboards[indx_bd].rows-1)&&(j==chessboards[indx_bd].cols-1))
							    cur_boards[indx_bd].edge_indx.push_back(j+i*chessboards[indx_bd].cols);
						}
					}  

					//标注标定板顶点
					for(int i=0;i<4;i++)
					{
						Point2f tmp = cur_boards[indx_bd].corners[cur_boards[indx_bd].edge_indx[i]];
						//cout<<"  Edge Point = "<<tmp.x<<" "<<tmp.y<<endl;
						if(indx_bd==0)
							cv::circle( img, tmp, 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );//蓝色
						if(indx_bd==1)
							cv::circle( img, tmp, 3,Scalar(0, 255, 0) ,CV_FILLED, CV_AA, 0 );//绿色
						if(indx_bd==2)
							cv::circle( img, tmp, 3,Scalar(0, 0, 255) ,CV_FILLED, CV_AA, 0 );//红色
					}
					//形心坐标
					Point2f centroid = acc/(float)numofcorner;
					cur_boards[indx_bd].centroid = centroid;
				}
				//判断当前帧是否用于内参标定
				bool ischoose = Update_Rect(max_x,min_x,max_y,min_y,image_size);
				//ischoose = true;
				//选中绿框，没选中红框
				if(ischoose)
					cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 2);
				else
					cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 2);
		
                //如果是第一帧，调整标定板顺序
				if(detectfirstframe==false)
				{
					cv::imshow ( "Image", img);
					//等待手动标注标定板顺序
					cv::waitKey(0);
					//初始化
					isMouseCallback = false;
					detectfirstframe = true;
					//最后交换顺序
					struct BOARD board0 = cur_boards[reorder_info[0]];
					struct BOARD board1 = cur_boards[reorder_info[1]];
					struct BOARD board2 = cur_boards[reorder_info[2]];
					cur_boards[0] = board0;
					cur_boards[1] = board1;
					cur_boards[2] = board2;
					
					pre_boards[0] = cur_boards[0];
					pre_boards[1] = cur_boards[1];
					pre_boards[2] = cur_boards[2];
					pre_first_edge_points.clear();
					for(int i=0;i<3;i++)
						pre_first_edge_points.push_back(cur_boards[i].corners[cur_boards[i].first_edge_indx]);
				}
				else
				{
					//根据上一个frame的重心来确定每个board中的first_edge_indx的序号
					TrackCentroid();
					Update_Ordered_Info(0);//根据点的顺序重新对board进行排序
					Update_Ordered_Info(1);//根据点的顺序重新对board进行排序
					Update_Ordered_Info(2);//根据点的顺序重新对board进行排序
					for(int indx_bd=0;indx_bd<3;indx_bd++)
					{
						for(int i=0;i<cur_boards[indx_bd].orderd_corners.size();i++)
						{
							Point2f tmp = cur_boards[indx_bd].orderd_corners[i];
							if((i==0)||(i==corner_height-1)||(i==corner_height*corner_width-corner_height)||(i==corner_height*corner_width-1))
								cv::putText(img,std::to_string(i+indx_bd*numofcorner),tmp,cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(80,127,255),1,1);
						}
					}
					cv::imshow ( "Image", img);
					cv::waitKey(waittime);
					//最后交换顺序
					struct BOARD board0 = cur_boards[0];
					struct BOARD board1 = cur_boards[1];
					struct BOARD board2 = cur_boards[2];
					cur_boards[0] = board0;
					cur_boards[1] = board1;
					cur_boards[2] = board2;
					
					pre_boards[0] = cur_boards[0];
					pre_boards[1] = cur_boards[1];
					pre_boards[2] = cur_boards[2];
				}
				//存放当前帧所有角点
				vector<cv::Point2f> three_bd_corners;
				for(int indx_bd = 0;indx_bd<3;indx_bd++)
				{
					for(int i=0;i<pre_boards[indx_bd].orderd_corners.size();i++)
						three_bd_corners.push_back(pre_boards[indx_bd].orderd_corners[i]);
				}
				
				if(ischoose)
				{
					all_corners.insert(pair<int,vector<Point2f>>(img_indx,three_bd_corners));
					camera_cal_frame.push_back(img_indx);
				}
					

			}
			else
			{
				//如果检测到当前帧的角点数量错误
				if(detectfirstframe)
				{
					vector<Point2f> curr_first_edge_points;
					vector<unsigned char> status;
					vector<float> error;
					cv::calcOpticalFlowPyrLK(pre_img,img,pre_first_edge_points,curr_first_edge_points,status,error,cv::Size(21,21),5);
					pre_first_edge_points = curr_first_edge_points;
					for(int i=0;i<3;i++)
					{
						cv::circle( img, pre_first_edge_points[i], 6,cv::Scalar(28,255,255));
					}
					cv::rectangle(img, rect, cv::Scalar(255, 255, 255), 2);
					cv::imshow ( "Image", img);
					cv::waitKey(waittime);
				}
			}
			if(img_indx==cal_num-1)
			{
				if(!detectfirstframe)
				{
					cout<<"no chessboard can be detected!"<<endl;
					abort();
				}
			}
	        pre_img = img.clone();
        }//for循环全部结束

      	if(camera_cal_frame.size()<2)
		{
			cout<<"camera_cal_frame:"<<camera_cal_frame.size()<<endl;
			cout<<"no enough images can be used to calibration!"<<endl;
			abort();
		}
  
		vector<vector<Point2f>> v2d,v2dp,valid2d,valid2d_planar;
		vector<vector<Point3f>> v3d,v3dp,valid3d,valid3d_planar;

        cout<<"number of camera_cal_frame:"<<camera_cal_frame.size()<<endl;
		for(int p=0;p<camera_cal_frame.size();p++)
		{
			vector<Point2f> temp,temp2;
			vector<Point3f> temp3d,temp3d2;
			//将标定特征点存入valid2d
			for(int i=0;i<all_corners[camera_cal_frame[p]].size();i++)
			{
				temp.push_back(all_corners[camera_cal_frame[p]][i]);
			}
			valid2d.push_back(temp);
			temp.clear();
			
			
			for(int j=0;j<3;j++)
			{
				for(int i=0;i<all_corners[camera_cal_frame[p]].size()/3;i++)
				{
					temp.push_back(all_corners[camera_cal_frame[p]][j*corner_height*corner_width+i]);
					if(temp.empty())
						cout<<"vector is empty!"<<endl;
				}
				valid2d_planar.push_back(temp);
				temp.clear();
			}
			
			for(int i=0;i<coor.size();i++)
			{
				temp3d.push_back(Point3f(coor[i][0],coor[i][1],coor[i][2]));
			}
			valid3d.push_back(temp3d);
			
			temp3d.clear();
			for(int i=0;i<coor_planar.size()/3;i++)
			{
				temp3d.push_back(Point3f(coor_planar[i][0],coor_planar[i][1],coor_planar[i][2]));
			}
			for(int i=0;i<3;i++)
			{
				valid3d_planar.push_back(temp3d);
			}    


            //test
			// for(int i=0;i<3;i++)
			// {
            //     temp2.push_back(all_corners[camera_cal_frame[p]][i*numofcorner]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][i*numofcorner+corner_height-1]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][(i+1)*numofcorner-corner_height]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][(i+1)*numofcorner-1]);

            //     temp3d2.push_back(Point3f(coor[i*numofcorner][0],coor[i*numofcorner][1],coor[i*numofcorner][2]));
			// 	temp3d2.push_back(Point3f(coor[i*numofcorner+corner_height-1][0],coor[i*numofcorner+corner_height-1][1],coor[i*numofcorner+corner_height-1][2]));
			// 	temp3d2.push_back(Point3f(coor[(i+1)*numofcorner-corner_height][0],coor[(i+1)*numofcorner-corner_height][1],coor[(i+1)*numofcorner-corner_height][2]));
			// 	temp3d2.push_back(Point3f(coor[(i+1)*numofcorner-1][0],coor[(i+1)*numofcorner-1][1],coor[(i+1)*numofcorner-1][2]));
			// }
            // v3d.push_back(temp3d2);
			// v2d.push_back(temp2);
			// temp2.clear();
			// temp3d2.clear();
			// for(int i=0;i<3;i++)
			// {
            //     temp2.push_back(all_corners[camera_cal_frame[p]][i*numofcorner]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][i*numofcorner+corner_height-1]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][(i+1)*numofcorner-corner_height]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][(i+1)*numofcorner-1]);
			// 	temp2.push_back(all_corners[camera_cal_frame[p]][i*numofcorner+7]);

            //     temp3d2.push_back(Point3f(coor_planar[i*numofcorner][0],coor_planar[i*numofcorner][1],coor_planar[i*numofcorner][2]));
			// 	temp3d2.push_back(Point3f(coor_planar[i*numofcorner+corner_height-1][0],coor_planar[i*numofcorner+corner_height-1][1],coor_planar[i*numofcorner+corner_height-1][2]));
			// 	temp3d2.push_back(Point3f(coor_planar[(i+1)*numofcorner-corner_height][0],coor_planar[(i+1)*numofcorner-corner_height][1],coor_planar[(i+1)*numofcorner-corner_height][2]));
			// 	temp3d2.push_back(Point3f(coor_planar[(i+1)*numofcorner-1][0],coor_planar[(i+1)*numofcorner-1][1],coor_planar[(i+1)*numofcorner-1][2]));
			// 	temp3d2.push_back(Point3f(coor_planar[i*numofcorner+7][0],coor_planar[i*numofcorner+7][1],coor_planar[i*numofcorner+7][2]));
			// 	v2dp.push_back(temp2);
			// 	v3dp.push_back(temp3d2);
			// 	temp2.clear();
			//     temp3d2.clear();
			// }

        }


		//ConvertToDouble(valid3d[0]);
		cout<<"Set 3D and 2D Coordinates is Finished"<<endl;
		cout<<"valid frame size = "<<valid3d.size()<<endl;

      

		cv::Mat distParameter_planar = Mat(1,5,CV_64FC1,Scalar::all(0));//摄像机的畸变系数
		cv::Mat intrincMatrix_planar;//内参矩阵,矩阵大小是3×3,所有初始值是0，每个元素是1维的，数据类型是32位float;
		vector<cv::Mat> rotateMat_planar;//旋转向量，后面会将其转换为矩阵 无论distparmater数据类型如何变换，输出的是double类型
		vector<cv::Mat> translateMat_planar;//平移向量，后面会将其转换为矩阵
        //获取内参初始值
		//cv::calibrateCamera(v3dp,v2dp,image_size,intrincMatrix_planar,distParameter_planar,rotateMat_planar,translateMat_planar);//CALIB_FIX_K3
		//intrincMatrix_planar = cv::initCameraMatrix2D(valid3d_planar,valid2d_planar,image_size);
		intrincMatrix_planar = CameraCalibration3D::initCameraMatrix2DNew(valid3d,valid2d,image_size,true);
		cout<<"Initial intrincMatrix = "<<endl;
		cout<<intrincMatrix_planar<<endl;
		cout<<"Initial distParameter = "<<endl;
		cout<<distParameter_planar<<endl;
		//cout<<"rotateMat:"<<rotateMat_planar[0]<<endl;
		//cout<<"translateMat:"<<translateMat_planar[0]<<endl;
		//3.得到相机的标定参数的初值
		//intrincMatrix = CameraCalibration3D::initCameraMatrix2DNew(valid3d,valid2d,image_size,true);
		//cout<<"Initial intrincMatrix = "<<endl;
		//cout<<intrincMatrix<<endl;
		distParameter = distParameter_planar.clone();//摄像机的畸变系数
		intrincMatrix = intrincMatrix_planar.clone();//内参矩阵,矩阵大小是3×3,所有初始值是0，每个元素是1维的，数据类型是32位float;
		vector<cv::Mat> rotateMat;//旋转向量，后面会将其转换为矩阵 无论distparmater数据类型如何变换，输出的是double类型
		vector<cv::Mat> translateMat;//平移向量，后面会将其转换为矩阵
		cv::calibrateCamera(valid3d,valid2d,image_size,intrincMatrix,distParameter,rotateMat,translateMat,CALIB_USE_INTRINSIC_GUESS);//cv::CALIB_USE_INTRINSIC_GUESS
		distParameter_planar=distParameter.clone();
		//4.最后保存标定结果
		cout<<"Final intrincMatrix = "<<endl;
		cout<<intrincMatrix<<endl;
		cout<<"Final distParameter = "<<endl;
		cout<<distParameter<<endl;
		

		cv::Mat distParameter_zero = Mat(1,5,CV_64FC1,Scalar::all(0));//摄像机的畸变系数
		Mat mapx_initial;
		Mat mapy_initial;
		cv::initUndistortRectifyMap(intrincMatrix,distParameter,Mat::eye(3,3,CV_32FC1),intrincMatrix,image_size,CV_32FC1,mapx,mapy);
		cv::initUndistortRectifyMap(intrincMatrix_planar,distParameter_planar,Mat::eye(3,3,CV_32FC1),intrincMatrix_planar,image_size,CV_32FC1,mapx_initial,mapy_initial);

		//绘制三位点投影到图像上的像素坐标
		for(int p=0;p<camera_cal_frame.size();p++)
		{
			int id_img = camera_cal_frame[p];
			vector<string> name;
			name = read_format(imgnames[id_img]," ");
				
			Mat img_org =cv::imread(path_root+"/leftImg/left_"+name[1]+".png",1);
			
			Mat img_unditort;// = img_res.clone();
			Mat img_unditort_initial;// = img_res.clone();
			cv::remap(img_org,img_unditort,mapx,mapy,INTER_LINEAR);//去畸变
			cv::remap(img_org,img_unditort_initial,mapx_initial,mapy_initial,INTER_LINEAR);
			vector<Point2d> img_points_distort,img_points_distort_initial;
			vector<Point2d> img_points_undistort,img_points_undistort_initial;
			
			
			cv::projectPoints(ConvertToDouble(valid3d[0]), rotateMat[p], translateMat[p], intrincMatrix, distParameter, img_points_distort);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d[0]), rotateMat[p], translateMat[p], intrincMatrix, distParameter_zero, img_points_undistort);//使用的是opencv的函数
			
			
			vector<Point2d> img_points_1,img_points_2,img_points_3;
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p], translateMat_planar[3*p], intrincMatrix_planar, distParameter_planar, img_points_1);//使用的是opencv的函数
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p+1], translateMat_planar[3*p+1], intrincMatrix_planar, distParameter_planar, img_points_2);//使用的是opencv的函数
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p+2], translateMat_planar[3*p+2], intrincMatrix_planar, distParameter_planar, img_points_3);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_planar, img_points_1);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_planar, img_points_2);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_planar, img_points_3);//使用的是opencv的函数
			img_points_distort_initial.insert(img_points_distort_initial.end(),img_points_1.begin(),img_points_1.end());
			img_points_distort_initial.insert(img_points_distort_initial.end(),img_points_2.begin(),img_points_2.end());
			img_points_distort_initial.insert(img_points_distort_initial.end(),img_points_3.begin(),img_points_3.end());
			
			vector<Point2d> img_points_rectifed_1,img_points_rectifed_2,img_points_rectifed_3;
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p], translateMat_planar[3*p], intrincMatrix_planar, distParameter_zero, img_points_rectifed_1);//使用的是opencv的函数
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p+1], translateMat_planar[3*p+1], intrincMatrix_planar, distParameter_zero, img_points_rectifed_2);//使用的是opencv的函数
			// cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat_planar[3*p+2], translateMat_planar[3*p+2], intrincMatrix_planar, distParameter_zero, img_points_rectifed_3);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_zero, img_points_rectifed_1);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_zero, img_points_rectifed_2);//使用的是opencv的函数
			cv::projectPoints(ConvertToDouble(valid3d_planar[0]), rotateMat[p], translateMat[p], intrincMatrix_planar, distParameter_zero, img_points_rectifed_3);//使用的是opencv的函数
			img_points_undistort_initial.insert(img_points_undistort_initial.end(),img_points_rectifed_1.begin(),img_points_rectifed_1.end());
			img_points_undistort_initial.insert(img_points_undistort_initial.end(),img_points_rectifed_2.begin(),img_points_rectifed_2.end());
			img_points_undistort_initial.insert(img_points_undistort_initial.end(),img_points_rectifed_3.begin(),img_points_rectifed_3.end());
			
			Mat img_final = img_org.clone();
			for(int j=0;j<img_points_distort.size();j++)
			{
				cv::circle( img_final, img_points_distort[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
				cv::circle( img_unditort, img_points_undistort[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			}
			cv::rectangle(img_unditort, rect, cv::Scalar(0, 255, 0), 2);
			cv::Mat merge_img_final;
			Util::imageJoinHorizon(img_final,img_unditort,merge_img_final);
		
			Mat img_initial = img_org.clone();
			for(int j=0;j<img_points_distort_initial.size();j++)
			{
				cv::circle( img_initial, img_points_distort_initial[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
				cv::circle( img_unditort_initial, img_points_undistort_initial[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			}
			cv::rectangle(img_unditort_initial, rect, cv::Scalar(0, 255, 0), 2);
			cv::Mat merge_img_initial;
			Util::imageJoinHorizon(img_initial,img_unditort_initial,merge_img_initial);
			
			cv::Mat merge_img;
			Util::imageJoinVertical(merge_img_initial,merge_img_final,merge_img);
			
			cv::putText(merge_img,std::to_string(p+1)+"/"+std::to_string(camera_cal_frame.size()),cv::Point2f(10,30),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(155,155,155),3);
			cv::imshow ( "Image", merge_img);
			cv::waitKey(0);

		}
		cout<<translateMat[0].type()<<endl;
		cout<<translateMat[0].size()<<endl;
      
		/*
		vector<string> content;
		content.push_back( std::to_string(intrincMatrix.at<float>(0,0)));
		content.push_back( std::to_string(intrincMatrix.at<float>(1,1)));
		content.push_back( std::to_string(intrincMatrix.at<float>(0,2)));
		content.push_back( std::to_string(intrincMatrix.at<float>(1,2)));
		content.push_back( std::to_string(distParameter.at<float>(0,0)));
		content.push_back( std::to_string(distParameter.at<float>(0,1)));
		content.push_back( std::to_string(distParameter.at<float>(0,2)));
		content.push_back( std::to_string(distParameter.at<float>(0,3)));
		//content.push_back( std::to_string(distParameter.at<float>(0,4)));
		FileIO::WriteSting2Txt(path_cal_res,content);
		*/
		
		
		
		vector<Eigen::Vector4f> world_plane;//世界坐标系下的三个平面参数，顺序是右 左 下
		vector<Eigen::Vector3f> world_centroid;
		for(int indx_bd = 0;indx_bd<3;indx_bd++)
		{
			float acc_x = 0;
			float acc_y = 0;
			float acc_z = 0;
			vector<Point3f> one_board;
			for(int i=0;i<numofcorner;i++)
			{
				one_board.push_back(valid3d[0][i+indx_bd*numofcorner]);
				acc_x = acc_x +valid3d[0][i+indx_bd*numofcorner].x;
				acc_y = acc_y +valid3d[0][i+indx_bd*numofcorner].y;
				acc_z = acc_z +valid3d[0][i+indx_bd*numofcorner].z;
			}
			acc_x = acc_x/numofcorner;
			acc_y = acc_y/numofcorner;
			acc_z = acc_z/numofcorner;
			world_centroid.push_back(Eigen::Vector3f(acc_x,acc_y,acc_z));
			Eigen::Vector4f plane_model = Calculate_Planar_Model(one_board);
			world_plane.push_back(plane_model);
			cout<<"world plane = "<<plane_model.transpose()<<endl;
		}
		//比较重要！！！！！！！！！！！！！！！！！！！！！！！1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//intrincMatrix = intrincMatrix_planar.clone();
		//distParameter = distParameter_planar.clone();
		//mapx = mapx_initial.clone();
		//mapy = mapy_initial.clone();
		for(map<int,vector<Point2f>>::iterator iter = all_corners.begin();iter!=all_corners.end();iter++)
		{
			vector<string> name;
			name = read_format(imgnames[iter->first]," ");
			Mat img_org =cv::imread(path_root+"/leftImg/left_"+name[1]+".png",1);
			//for test
			/*
			vector<Point2d> pnp_projected_points;
			cv::projectPoints(ConvertToDouble(valid3d[0]), angleaxis, cvt, intrincMatrix, distParameter, pnp_projected_points);
			vector<string> name;
			name = read_format(imgnames[iter->first]," ");
			Mat img_org =cv::imread(path_root+"/leftImg/left_"+name[1]+".png",1);
			Mat img_rectified;
			cv::remap(img_org,img_rectified,mapx,mapy,INTER_LINEAR);
			for(int j=0;j<pnp_projected_points.size();j++)
			{
			cv::circle( img_org, pnp_projected_points[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			}*/
			Mat img_unditort;
			cv::remap(img_org,img_unditort,mapx,mapy,INTER_LINEAR);
			vector<cv::Point2f> undistorpoints;
			cv::undistortPoints(iter->second, undistorpoints, intrincMatrix, distParameter,cv::noArray(),intrincMatrix);
			//for(int j=0;j<undistorpoints.size();j++)
			//{
			//  cv::circle( img_unditort, undistorpoints[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			//}
			//cv::imshow ( "Image", img_unditort);
			//cv::waitKey(0);
			
			cv::Mat angleaxis;
			cv::Mat cvt;
			vector<Point3f> first_board_3d(valid3d[0].begin(),valid3d[0].end());
			vector<Point2f> first_board_2d(undistorpoints.begin(),undistorpoints.end());
			cv::solvePnP ( ConvertToDouble(valid3d[0]), ConvertToDouble(undistorpoints), intrincMatrix, Mat(1,5,CV_64FC1,Scalar::all(0)), angleaxis, cvt, false ,cv::SOLVEPNP_EPNP); // SOLVEPNP_ITERATIVE,SOLVEPNP_UPNP,SOLVEPNP_DLS,SOLVEPNP_EPNP
			cv::solvePnP ( ConvertToDouble(valid3d[0]), ConvertToDouble(undistorpoints), intrincMatrix, Mat(1,5,CV_64FC1,Scalar::all(0)), angleaxis, cvt, true ,cv::SOLVEPNP_ITERATIVE);
			cv::Mat cvR;
			cv::Rodrigues ( angleaxis, cvR );
			cout<<"img_index:"<<iter->first<<endl;
			cout<<"cvR:"<<cvR<<endl;
			cout<<"cvt:"<<cvt<<endl;
			vector<Point2d> pnp_projected_points;
			cv::projectPoints(ConvertToDouble(first_board_3d), angleaxis, cvt, intrincMatrix, Mat(1,5,CV_64FC1,Scalar::all(0)), pnp_projected_points);
			for(int j=0;j<pnp_projected_points.size();j++)
			{
				cv::circle( img_unditort, pnp_projected_points[j], 3,Scalar(255, 0, 0) ,CV_FILLED, CV_AA, 0 );
			}
			cv::imshow ( "Image", img_unditort);
			cv::waitKey(0);
			
			Eigen::Matrix3f R_eigen;//Rcw
			Eigen::Vector3f t_eigen;
			R_eigen<<(float)cvR.at<double>(0,0),(float)cvR.at<double>(0,1),(float)cvR.at<double>(0,2),
				(float)cvR.at<double>(1,0),(float)cvR.at<double>(1,1),(float)cvR.at<double>(1,2),
				(float)cvR.at<double>(2,0),(float)cvR.at<double>(2,1),(float)cvR.at<double>(2,2);
			t_eigen<<(float)cvt.at<double>(0),(float)cvt.at<double>(1),(float)cvt.at<double>(2);
			cout<<t_eigen.transpose()<<endl;
			vector<Eigen::Vector4f> tmp_Plane;
			vector<Eigen::Vector4f> tmp_Plane2;
			cout<<endl;
			for(int i=0;i<3;i++)
			{
				Eigen::Vector3f normal(world_plane[i][0],world_plane[i][1],world_plane[i][2]);
				Eigen::Vector3f cam_normal = R_eigen*normal;
				if((R_eigen*world_centroid[i]+t_eigen).adjoint()*cam_normal>0)
				    cam_normal = -cam_normal;
				
				float cam_D = world_plane[i][3]-(cam_normal[0]*t_eigen[0]+cam_normal[1]*t_eigen[1]+cam_normal[2]*t_eigen[2]);
				
				cout<<"index:"<<iter->first<<endl;
				cout<<cam_normal[0]<<" "<<cam_normal[1]<<" "<<cam_normal[2]<<" "<<cam_D<<endl;
				vector<Point3f> oneboad;
				Eigen::Vector3f avg(0,0,0);
				for(int j=0;j<numofcorner;j++)
				{
					Point3f point = valid3d[0][i*numofcorner+j];
					Eigen::Vector3f point_eigen(point.x,point.y,point.z);
					point_eigen = R_eigen*point_eigen+t_eigen;
					avg = point_eigen + avg;
					point.x = point_eigen[0];
					point.y = point_eigen[1];
					point.z = point_eigen[2];
					oneboad.push_back(point);
				}
				avg = avg/numofcorner;
				Eigen::Vector4f p_cam = Calculate_Planar_Model(oneboad);
				if(avg.adjoint()*p_cam.head<3>()>0)
				    p_cam = -1.0* p_cam;
				tmp_Plane2.push_back(p_cam);
				if(cam_normal[0]*avg[0]+cam_normal[1]*avg[1]+cam_normal[2]*avg[2]>0)
				    tmp_Plane.push_back(Eigen::Vector4f(-cam_normal[0],-cam_normal[1],-cam_normal[2],-cam_D));
				else
				    tmp_Plane.push_back(Eigen::Vector4f(cam_normal[0],cam_normal[1],cam_normal[2],cam_D));
			}
			cout<<"method 1 plane param = "<<tmp_Plane[0].transpose()<<" "<<tmp_Plane[1].transpose()<<" "<<tmp_Plane[2].transpose()<<endl;
			cout<<"method 2 plane param = "<<tmp_Plane2[0].transpose()<<" "<<tmp_Plane2[1].transpose()<<" "<<tmp_Plane2[2].transpose()<<endl<<endl;
			cam_planes.insert(pair<int,vector<Eigen::Vector4f>>(iter->first,tmp_Plane2));
			cout<<"cam plane indx = "<<iter->first<<" "<<tmp_Plane[0].transpose()<<" "<<tmp_Plane[1].transpose()<<" "<<tmp_Plane[2].transpose()<<endl;
		}
        cv::destroyAllWindows();
  
    }
  
	if(useLidar==1)
	{
		//这个是我们之前使用的方法
		//Segment::Segment seg;
		//seg.initialize(n,lidar_config_path);
		ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/MapCloud",1);//聚类的点云
		nav_msgs::Path path_msg;
		ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/Path", 1);//轨迹
		ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/Pose", 1);//ndt输出得到的pose
		
		pcl::PointCloud<pcl::PointXYZI> global_map;
		vector<Eigen::Matrix4f> lidar_poses;
		PCLlib::NDT ndt(lidar_config_path);//ndt初始化构造函数
		for(int i=0;i<camera_cal_frame.size();i++)//imgnames.size()
		{
			cout<<camera_cal_frame[i]<<"/"<<imgnames.size()<<endl;//i
			vector<string> name = read_format(imgnames[camera_cal_frame[i]]," ");//读取点云文件//i
			vector<vector<float>> cloud_points = FileIO::ReadTxt2Float(path_root+"/lidar/"+name[0]+".txt");
			pcl::PointCloud<pcl::PointXYZI> cloud;
			for(int j=0;j<cloud_points.size();j++)
			{
				float x = cloud_points[j][0]; 
				float y = cloud_points[j][1];
				float z = cloud_points[j][2];
				//float range = sqrt(temp.x * temp.x + temp.y * temp.y + temp.z * temp.z);
				//if(range > 1)
				{
					float horizonAngle = -atan2(y, x)*180/PI+180;//输出范围0-360度
					if((horizonAngle>=90)&&(horizonAngle<=270))
					{
					    pcl::PointXYZI temp;
					    temp.x = x;
					    temp.y = y;
					    temp.z = z;
						temp.intensity = camera_cal_frame[i];//i
						cloud.push_back(temp);
					}
				}
			}
		
			Eigen::Matrix4f Twl;//ndt计算得到的位姿
			ndt.AddCurrentCloud(cloud,Twl);//ndt定位算法
			lidar_poses.push_back(Twl);
			
			
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());//当前帧变换到世界地图中的坐标
			pcl::transformPointCloud(cloud, *transformed_scan_ptr, Twl);//将当前帧的点云scan_ptr变化到世界地图下得到transformed_scan_ptr
			global_map += *transformed_scan_ptr;//将当前帧点云融合到世界坐标当中
			cout<<"cloud size = "<<cloud.points.size()<<" transformed cloud size = "<<transformed_scan_ptr->points.size()<<" global map size = "<<global_map.points.size()<<endl;
			//6.将一些数据pub出去给rviz
			ros::Time ros_timestamp = ros::Time::now();
			if(map_pub.getNumSubscribers()>0)//全局地图
			{
				sensor_msgs::PointCloud2 map_msg;
				pcl::toROSMsg(global_map,map_msg);
				map_msg.header.frame_id = "/velodyne";
				map_msg.header.stamp = ros::Time::now();
				map_pub.publish(map_msg);
			}
			//发布位姿信息
			geometry_msgs::PoseStamped ndtpose_msg;
			ndtpose_msg.header.frame_id = "/velodyne";
			ndtpose_msg.header.stamp = ros::Time::now();
			ndtpose_msg.pose.position.x = Twl(0,3);
			ndtpose_msg.pose.position.y = Twl(1,3);
			ndtpose_msg.pose.position.z = Twl(2,3);
			Eigen::Matrix3f R_ndt = Twl.block<3,3>(0,0);
			Eigen::Quaternionf q(R_ndt);
			ndtpose_msg.pose.orientation.x = q.x();
			ndtpose_msg.pose.orientation.y = q.y();
			ndtpose_msg.pose.orientation.z = q.z();
			ndtpose_msg.pose.orientation.w = q.w();
			
			path_msg.poses.push_back(ndtpose_msg); 
			if(pose_pub.getNumSubscribers()>0)//ndt实时的位姿
			{
				pose_pub.publish(ndtpose_msg);
			}
			if(path_pub.getNumSubscribers()>0)//ndt的轨迹
			{
				path_msg.header.frame_id = "/velodyne";
				path_msg.header.stamp = ros::Time::now();
				path_msg.poses.push_back(ndtpose_msg);
				path_pub.publish(path_msg);
			}
		
		}
		
		/*保存全局地图和每帧的激光位姿态
		vector<string> save_global_map;
		for(int i=0;i<global_map.points.size();i++)
		{
		string tmp = "";
		tmp = to_string(global_map.points[i].x)+" "+to_string(global_map.points[i].y)+" "+to_string(global_map.points[i].z)+" "+to_string(global_map.points[i].intensity);
		save_global_map.push_back(tmp);
		}
		FileIO::WriteSting2Txt("/home/fyy/catkin_ws/src/calibration/map.txt",save_global_map);
		
		vector<string> save_lidar_pose;
		for(int i=0;i<lidar_poses.size();i++)
		{
		Eigen::Vector3f t = lidar_poses[i].block<3,1>(0,3);
		Eigen::Quaternionf q;
		Eigen::Matrix3f R = lidar_poses[i].block<3,3>(0,0);
		q = R;
		string tmp = to_string(t[0])+" ";
		tmp = tmp + to_string(t[1])+" ";
		tmp = tmp + to_string(t[2])+" ";
		tmp = tmp + to_string(q.w())+" "; 
		tmp = tmp + to_string(q.x())+" "; 
		tmp = tmp + to_string(q.y())+" "; 
		tmp = tmp + to_string(q.z()); 
		save_lidar_pose.push_back(tmp);
		}
		FileIO::WriteSting2Txt("/home/fyy/catkin_ws/src/calibration/ndt_pose.txt",save_lidar_pose);
		*/
		
		ros::Publisher segmentcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/SegementCloud",1);//聚类的点云
		ros::Publisher framesegmentcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/FrameSegementCloud",1);
		ros::Publisher threeplane_pub = n.advertise<sensor_msgs::PointCloud2>("/ThreePlane",1);//聚类的点云
		
		//读取已经生成好的地图
		/*
		vector<vector<float>> cloud_points = FileIO::ReadTxt2Float("/home/fyy/catkin_ws/src/calibration/map.txt");
		pcl::PointCloud<pcl::PointXYZI> global_map;
		for(int j=0;j<cloud_points.size();j++)
		{
		float x = cloud_points[j][0]; 
		float y = cloud_points[j][1];
		float z = cloud_points[j][2];
		int frame_indx =  cloud_points[j][3];

		pcl::PointXYZI temp;
		temp.x = x; 
		temp.y = y;
		temp.z = z;
		temp.intensity = frame_indx;
		global_map.points.push_back(temp);
		}*/
		vector<string> global_points;
		for(int i=0;i<global_map.size();i++)
		{
			global_points.push_back(to_string(global_map.points[i].x)+" "+to_string(global_map.points[i].y)+" "+to_string(global_map.points[i].z)+" "+to_string(global_map.points[i].intensity));
		}
		FileIO::WriteSting2Txt("/home/xuan/catkin_ws/src/calibration/global_points.txt",global_points);

		cout<<"Finish Reading"<<endl;
		PCLlib::RegionGrow regiongrow(lidar_config_path);
		TicToc tic;
		pcl::PointCloud<pcl::PointXYZI> segment_cloud = regiongrow.SegmentCloud(global_map);//intensity = 0,表示没有被分类的点
		cout<<"Segment Cost Time(ms) = "<<tic.toc()<<endl;

		pcl::PointCloud<pcl::PointXYZI> local_cloud;
		vector<string> Segment_cloud;
		for(int i=0;i<segment_cloud.size();i++)
		{
			Segment_cloud.push_back(to_string(segment_cloud[i].x)+" "+to_string(segment_cloud[i].y)+" "+to_string(segment_cloud[i].z)+" "+to_string(segment_cloud[i].intensity));
		}
        FileIO::WriteSting2Txt("/home/xuan/catkin_ws/src/calibration/segment_cloud.txt",Segment_cloud);
		for(int i=0;i<camera_cal_frame.size();i++)//
		{
			vector<string> name = read_format(imgnames[camera_cal_frame[i]]," ");//i
			vector<vector<float>> cloud_points_tmp = FileIO::ReadTxt2Float(path_root+"/lidar/"+name[0]+".txt");
			for(int j=0;j<cloud_points_tmp.size();j++)
			{
				float x = cloud_points_tmp[j][0]; 
				float y = cloud_points_tmp[j][1];
				float z = cloud_points_tmp[j][2];
				float horizonAngle = -atan2(y, x)*180/PI+180;//输出范围0-360度
				if((horizonAngle>=70)&&(horizonAngle<=290))
				{
					pcl::PointXYZI tmp;
					tmp.x = x;
					tmp.y = y;
					tmp.z = z;
					tmp.intensity = camera_cal_frame[i];//i
					local_cloud.points.push_back(tmp);
				}
			}
		}

		int max_frame_indx = camera_cal_frame.back()+1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
		pcl::PointCloud<pcl::PointXYZI> eachframe_segmentcloud[max_frame_indx];
		pcl::PointCloud<pcl::PointXYZI> three_segmentcloud[max_frame_indx];
		for(int i=0;i<segment_cloud.points.size();i++)
		{
			int frame_indx = local_cloud.points[i].intensity;//第几帧
			int seg_indx = segment_cloud.points[i].intensity;//平面序号
			if(seg_indx!=0)
			{
				local_cloud.points[i].intensity = seg_indx;
				eachframe_segmentcloud[frame_indx].points.push_back(local_cloud.points[i]);
			}
		}
		struct Vote
		{
			int ind[3];
			int count;
			Vote(int ind_[3],int count_)
			{
				for(int i=0;i<3;i++)
				{
					ind[i]=ind_[i];
				}
				count=count_;
			}
		};
		// struct Planers
		// {
		// 	vector<int> index;
		// 	vector<Eigen::Vector4f> planr;
		// }
		vector<Vote> vote;
		map<int,map<int,CloudInfo>> local_segclouds;
		set<int> cal_index;
		//对每帧的点云数据根据分类的label结果进行分类
		for(int i=0;i<camera_cal_frame.size();i++)
		{
			float min_pitch = 10000;
			float max_pitch = -10000;
			map<int,CloudInfo> local_segcloud;
			set<int> indx_planes;//保存这一帧平面的所有分类结果
			//cout<<"cal_frame:"<<camera_cal_frame[i]<<endl;
			//将点云按照平面分类序号存入
			for(int j=0;j<eachframe_segmentcloud[camera_cal_frame[i]].points.size();j++)//i
			{
				int seg_indx = eachframe_segmentcloud[camera_cal_frame[i]].points[j].intensity;
				//如果没找到元素返回end（），找到了返回索引值
				if(local_segcloud.find(seg_indx)==local_segcloud.end())
				{
					indx_planes.insert(seg_indx);//插入索引			
					CloudInfo cloudinfo;
					cloudinfo.cloud.points.push_back(eachframe_segmentcloud[camera_cal_frame[i]].points[j]);//i
					local_segcloud.insert(pair<int,CloudInfo>(seg_indx,cloudinfo));
				}
				else
				{
					local_segcloud[seg_indx].cloud.points.push_back(eachframe_segmentcloud[camera_cal_frame[i]].points[j]);//i
				}
					//float x = eachframe_segmentcloud[i].points[j].x;
					//float y = eachframe_segmentcloud[i].points[j].y;
					//float z = eachframe_segmentcloud[i].points[j].z;
					//float verticalAngle = atan2(z, sqrt(x * x + y * y))*180/PI;
					//if(verticalAngle<min_pitch)
					//  min_pitch = verticalAngle;
					// if(verticalAngle>max_pitch)
					// max_pitch = verticalAngle;
			}

				//float unit_pitch = (max_pitch-min_pitch)/(16-1);
			//计算平面中心和平面参数
			for(map<int,CloudInfo>::iterator iter = local_segcloud.begin();iter!=local_segcloud.end();iter++)
			{
				//float avg_x = 0;
				//float avg_y = 0;
				//float avg_z = 0;
				//for(int j=0;j<iter->second.cloud.points.size();j++)
				//{
				//  float x = iter->second.cloud.points[j].x;
				//  avg_x = avg_x +x;
				//  float y = iter->second.cloud.points[j].y;
				//  avg_y = avg_y +y;
				//  float z = iter->second.cloud.points[j].z;
				//  avg_z = avg_z +z;
					//float verticalAngle = atan2(z, sqrt(x * x + y * y))*180/PI;
					//int pitchInd = round((verticalAngle-min_pitch)/unit_pitch);
					//if(iter->second.vertical_indx.find(pitchInd)==iter->second.vertical_indx.end())
					//  iter->second.vertical_indx.insert(pitchInd);
				//}
				//int num = iter->second.cloud.points.size();
				Eigen::Vector4f centroid;
				//计算质心 ->first代表下标   ->second代表值 centroid（x0，y0,z0,1）
				pcl::compute3DCentroid(iter->second.cloud, centroid);
				iter->second.center = Eigen::Vector3f(centroid[0],centroid[1],centroid[2]);
				//判断当前类别点云数量是否能拟合平面，不能则跳过
				if(iter->second.cloud.size()<5)
				{
					int temp=iter->first;
					iter++;
					indx_planes.erase(temp);
					local_segcloud.erase(temp);
					iter--;
					continue;
				}
				//获得平面参数
				Eigen::Vector4f plane_param = Calculate_Planar_Model(iter->second.cloud);
				Eigen::Vector3f normal(plane_param[0],plane_param[1],plane_param[2]);//平面法线
				if(normal.adjoint()*iter->second.center>0)
				{
					plane_param = plane_param*-1.0;
				}
				iter->second.plane_param = plane_param;
				
			}
            
			/*
			vector<int> deleted_indx;
			for(map<int,CloudInfo>::iterator iter = local_segcloud.begin();iter!=local_segcloud.end();iter++)
			{
			if(iter->second.vertical_indx.size()<2)
				deleted_indx.push_back(iter->first);
			}
			cout<<"Plane number before filter = "<<local_segcloud.size()<<endl;
			cout<<"Deleted Plane num = "<<deleted_indx.size()<<endl;
			for(int j=0;j<deleted_indx.size();j++)
			{
			local_segcloud.erase(deleted_indx[j]);
			indx_planes.erase(deleted_indx[j]);
			}*/


			//找到所有三个平面的组合
			vector<int> valid_indx_planes;
			valid_indx_planes.assign(indx_planes.begin(),indx_planes.end());
			//cout<<"nums of planes:"<<valid_indx_planes.size()<<endl;
			//排列组合，排列出所有可能点组合。或许可以优化？
			vector<vector<int>> combinations = Util::combine(valid_indx_planes,3);
			//cout<<"combinations:"<<combinations.size()<<endl;
			vector<vector<int>> valid_combinations;
			for(int j=0;j<combinations.size();j++)
			{
				int a = combinations[j][0];
				int b = combinations[j][1];
				int c = combinations[j][2];
				Eigen::Vector3f a_center = local_segcloud[a].center;
				Eigen::Vector3f b_center = local_segcloud[b].center;
				Eigen::Vector3f c_center = local_segcloud[c].center;
				if( ((a_center-b_center).norm()<1.0)&&((a_center-c_center).norm()<1.0)&&((b_center-c_center).norm()<1.0) )//中心距离小于1m
				{
					Eigen::Vector3f a_normal = local_segcloud[a].plane_param.block<3,1>(0,0);
					Eigen::Vector3f b_normal = local_segcloud[b].plane_param.block<3,1>(0,0);
					Eigen::Vector3f c_normal = local_segcloud[c].plane_param.block<3,1>(0,0);
					if((a_normal.adjoint()*b_normal<0.15)&&(a_normal.adjoint()*c_normal<0.15)&&(b_normal.adjoint()*c_normal<0.15))//三个法线尽量垂直
					{
						int ground = a;
						int left = b;
						int right = c;
						float min_z = a_center[2];
						if(min_z>b_center[2]) 
						{
							ground = combinations[j][1];
							left = combinations[j][0];
							right = combinations[j][2];
							min_z = b_center[2];
						}
						if(min_z>c_center[2])
						{
							ground = combinations[j][2];
							left = combinations[j][0];
							right = combinations[j][1];
						}
						if(local_segcloud[left].center[1]<local_segcloud[right].center[1])
						{
							int tmp = left;
							left = right;
							right = tmp;
						}
						if( ((local_segcloud[right].center-local_segcloud[left].center).adjoint()*local_segcloud[left].plane_param.block<3,1>(0,0)>0)&&
						((local_segcloud[left].center-local_segcloud[right].center).adjoint()*local_segcloud[right].plane_param.block<3,1>(0,0)>0) )
						{
							vector<int> temp_valid;
							temp_valid.push_back(left);
							temp_valid.push_back(right);
							temp_valid.push_back(ground);
							valid_combinations.push_back(temp_valid);
							cout<<i<<": "<<left<<" "<<right<<" "<<ground<<endl;
						}
						else
							continue;
					}
					else
						continue;
				}
				else
					continue;
			}
            
			//找到了空间中有三个相互垂直的平面
			if(valid_combinations.size()>=1)
			{
				cal_index.insert(camera_cal_frame[i]);
				local_segclouds.insert(pair<int,map<int,CloudInfo>>(camera_cal_frame[i],local_segcloud));
				int min_idnx = 0;
				float min_dist = 1000000;
				for(int k=0;k<valid_combinations.size();k++)
				{
					int a = valid_combinations[k][0];
					int b = valid_combinations[k][1];
					int c = valid_combinations[k][2];
					//两两平面之间的距离和
					float dist = (local_segcloud[a].center-local_segcloud[b].center).norm()+
						(local_segcloud[a].center-local_segcloud[c].center).norm()+
						(local_segcloud[b].center-local_segcloud[c].center).norm();
					if(dist<min_dist)
					{
						min_dist = dist;
						min_idnx = k;
					}
				}
				int left = valid_combinations[min_idnx][0];
				int right = valid_combinations[min_idnx][1];
				int ground = valid_combinations[min_idnx][2];
							
				cout<<i<<" chosen = "<<left<<" "<<right<<" "<<ground<<endl;
				int chose[]={left,right,ground};
				if(vote.size()==0)
				{
					Vote temp(chose,1);
					vote.push_back(temp);
				}
				else
				{
                    for(int k=0;k<vote.size();k++)
					{
						if(vote[k].ind[0]==chose[0]&&vote[k].ind[1]==chose[1]&&vote[k].ind[2]==chose[2])
						{
							vote[k].count=vote[k].count+1;
							break;
						}
						if(k==vote.size()-1)
						{
							Vote temp(chose,1);
					        vote.push_back(temp);
						}
					}
				}
			}
			}//遍历每一帧结束

			int index;
			for(int k=0;k<vote.size();k++)
			{
				int temp;
				cout<<"chosen:"<<vote[k].ind[0]<<" "<<vote[k].ind[1]<<" "<<vote[k].ind[2]<<endl;
				cout<<"counts:"<<vote[k].count<<endl;
				if(k==0)
				{
					index=k;
					temp=vote[k].count;
					continue;
				}
				if(temp<vote[k].count)
				{
					index=k;
					temp=vote[k].count;
				}
			}
			cout<<"final chosen:"<<vote[index].ind[0]<<" "<<vote[index].ind[1]<<" "<<vote[index].ind[2]<<endl;
            cout<<"cal_index.size():"<<cal_index.size()<<endl;

			vector<Eigen::Vector4f> tmp_plane_params;
			int left,right,ground;
			left=vote[index].ind[0];
			right=vote[index].ind[1];
			ground=vote[index].ind[2];
			vector<string> plane_points;
			for(set<int>::iterator iter=cal_index.begin();iter!=cal_index.end();iter++)
			{
				tmp_plane_params.clear();
				map<int,CloudInfo> local_segcloud=local_segclouds[*iter];
				if(local_segcloud.find(left)!=local_segcloud.end()||local_segcloud.find(right)!=local_segcloud.end()||local_segcloud.find(ground)!=local_segcloud.end())
				{
					tmp_plane_params.push_back(local_segcloud[right].plane_param);//NOTICE Sequenc!!!!!!!!!!!!!
					tmp_plane_params.push_back(local_segcloud[left].plane_param);
					tmp_plane_params.push_back(local_segcloud[ground].plane_param);
					lidar_planes.insert(pair<int,vector<Eigen::Vector4f>>(*iter,tmp_plane_params));
				}
				for(int k=0;k<eachframe_segmentcloud[*iter].points.size();k++)//i
				{   //i
					if( (eachframe_segmentcloud[*iter].points[k].intensity==left)||
					(eachframe_segmentcloud[*iter].points[k].intensity==right) ||
					(eachframe_segmentcloud[*iter].points[k].intensity==ground) )
					{
						three_segmentcloud[*iter].points.push_back(eachframe_segmentcloud[*iter].points[k]);//i
						plane_points.push_back(to_string(eachframe_segmentcloud[*iter].points[k].x)+" "+to_string(eachframe_segmentcloud[*iter].points[k].y)+" "+to_string(eachframe_segmentcloud[*iter].points[k].z)+" "+to_string(eachframe_segmentcloud[*iter].points[k].intensity)+" "+to_string(*iter));
					}
				}
				
				
				
			}
			FileIO::WriteSting2Txt("/home/xuan/catkin_ws/src/calibration/plane_points.txt",plane_points);
            cout<<"lidar_planes.size():"<<lidar_planes.size()<<endl;
		
		
		//发布信息
		for(int i=0;i<camera_cal_frame.size();i++)
		{
			sleep(1.5);//等待1.5秒时间
			if(framesegmentcloud_pub.getNumSubscribers()>0)
			{
				sensor_msgs::PointCloud2 segementcloud_msg;
				pcl::toROSMsg(eachframe_segmentcloud[camera_cal_frame[i]],segementcloud_msg);
				segementcloud_msg.header.frame_id = "/velodyne";
				segementcloud_msg.header.stamp = ros::Time::now();
				framesegmentcloud_pub.publish(segementcloud_msg);
			}
			if(map_pub.getNumSubscribers()>0)//全局地图
			{
				sensor_msgs::PointCloud2 map_msg;
				pcl::toROSMsg(global_map,map_msg);
				map_msg.header.frame_id = "/velodyne";
				map_msg.header.stamp = ros::Time::now();
				map_pub.publish(map_msg);
			}
			if(segmentcloud_pub.getNumSubscribers()>0)//全局地图
			{
				sensor_msgs::PointCloud2 segementcloud_msg;
				pcl::toROSMsg(segment_cloud,segementcloud_msg);
				segementcloud_msg.header.frame_id = "/velodyne";
				segementcloud_msg.header.stamp = ros::Time::now();
				segmentcloud_pub.publish(segementcloud_msg);
			}
			if(threeplane_pub.getNumSubscribers()>0)//全局地图
			{
				if(three_segmentcloud[i].points.size()!=0)
				{
					sensor_msgs::PointCloud2 threeplanecloud_msg;
					pcl::toROSMsg(three_segmentcloud[i],threeplanecloud_msg);
					threeplanecloud_msg.header.frame_id = "/velodyne";
					threeplanecloud_msg.header.stamp = ros::Time::now();
					threeplane_pub.publish(threeplanecloud_msg);
				}
			}
		//if(i==max_frame_indx-1)
		//i = 0;
		}
	}//if use lidar is set to false
	
  
	if((useLidar==1)&&(useCamera==1))
	{
		cout<<"0.Start Extrinsic Optimization!!!!!!!!!!!"<<endl;
		//最后开始计算外参
		vector<vector<Eigen::Vector4f>> lidarPlanes;
		vector<vector<Eigen::Vector4f>> camPlanes;
		for(map<int,vector<Eigen::Vector4f>>::iterator iter=lidar_planes.begin();iter!=lidar_planes.end();iter++)
		{
			int i=iter->first;
			//这一frame同时检测到了相机和激光的平面，才会加入到优化中
			//if((cam_planes.find(i)!=cam_planes.end())&&(lidar_planes.find(i)!=lidar_planes.end()))
			{
				lidarPlanes.push_back(lidar_planes[i]);
				camPlanes.push_back(cam_planes[i]);
				 cout<<"The frame indx to join extrinsic optimization = "<<i<<endl;
				// //cout<<" lidar plane norm= "<<lidar_planes[i][0].head<3>().norm()<<" "<<lidar_planes[i][1].head<3>().norm()<<endl;
				// //cout<<" cam plane norm = "<<cam_planes[i][0].head<3>().norm()<<" "<<cam_planes[i][1].head<3>().norm()<<endl;
				 cout<<" lidar plane = "<<lidar_planes[i][0].transpose()<<" "<<lidar_planes[i][1].transpose()<<" "<<lidar_planes[i][2].transpose()<<endl;
				 cout<<" cam plane  = "<<cam_planes[i][0].transpose()<<" "<<cam_planes[i][1].transpose()<<" "<<cam_planes[i][2].transpose()<<endl;
			}
		}
		cout<<"Num of frames to optimize extrisinc = "<<lidarPlanes.size()<<endl;
		//为了计算得到tc,L和Rc,L
		//先拟合得到Rc,l 然后再线性计算得到tc,l
		if(lidarPlanes.size()!=0)
		{
			cout<<"1.Start Rotatoin Matrix Optimization!!!!!"<<endl;
			Eigen::Matrix3d Rcl;
			Eigen::Vector3d tcl;
			vector<Point3d> lidar_normals;
			vector<Point3d> cam_normals;
			//然后计算tcl，使用eigen求解线性方程组
			Eigen::MatrixXd A;
			Eigen::MatrixXd b;
			A = Eigen::MatrixXd::Zero(3*lidarPlanes.size(),3);
			b = Eigen::MatrixXd::Zero(3*lidarPlanes.size(),1);
			for(int i=0;i<lidarPlanes.size();i++)
			{
				for(int j=0;j<3;j++)
				{
					//if(j==0)
					//{
					lidar_normals.push_back(cv::Point3d((double)lidarPlanes[i][j](0),(double)lidarPlanes[i][j](1),(double)lidarPlanes[i][j](2)));
					cam_normals.push_back(cv::Point3d((double)camPlanes[i][j](0),(double)camPlanes[i][j](1),(double)camPlanes[i][j](2)));
					
					A(3*i+j,0) = (double)camPlanes[i][j](0);
					A(3*i+j,1) = (double)camPlanes[i][j](1);
					A(3*i+j,2) = (double)camPlanes[i][j](2);
					b(3*i+j,0) = (double)lidarPlanes[i][j](3)-(double)camPlanes[i][j](3);
					
					//A(i,0) = (double)camPlanes[i][j](0);
					//A(i,1) = (double)camPlanes[i][j](1);
					//A(i,2) = (double)camPlanes[i][j](2);
					//b(i,0) = (double)lidarPlanes[i][j](3)-(double)camPlanes[i][j](3);
					//}
				}
			}
			ICP::pose_estimation_3d3d (cam_normals,lidar_normals,Rcl,tcl);
			cout<<"ICP tcl(should be equal to zero) = "<<tcl.transpose()<<endl;
			
			cout<<"2.Start translation vector Optimization!!!!!"<<endl;
			Eigen::EigenSolver<Eigen::MatrixXd> es( A.transpose()*A );
			 cout<<"A = "<<endl<<A<<endl;
			 cout<<"ATA = "<<endl<<A.transpose()*A<<endl;
			 cout<<"b = "<<endl<<b<<endl;

			Eigen::MatrixXd tcl_xd = A.colPivHouseholderQr().solve(b);
			
			cout<<"求解tcl 矩阵的特征值 = "<<endl<<es.eigenvalues()<<endl;
			tcl(0) = tcl_xd(0);
			tcl(1) = tcl_xd(1);
			tcl(2) = tcl_xd(2);
			
			cout<<"Final Tcl = "<<endl<<Rcl<<endl;
			cout<<tcl.transpose()<<endl;
			//
			string data_path;
		    n.getParam("/calibration/path_Extrinsic", data_path);//根目录
			Rcl_=Rcl;
			Tcl_=tcl;
			vector<string> calibraion_res;
			calibraion_res.push_back( to_string(Rcl(0,0))+" "+to_string(Rcl(0,1))+" "+to_string(Rcl(0,2)) );
			calibraion_res.push_back( to_string(Rcl(1,0))+" "+to_string(Rcl(1,1))+" "+to_string(Rcl(1,2)) );
			calibraion_res.push_back( to_string(Rcl(2,0))+" "+to_string(Rcl(2,1))+" "+to_string(Rcl(2,2)) );
			calibraion_res.push_back( to_string(tcl[0])+" "+to_string(tcl[1])+" "+to_string(tcl[2]) );
			calibraion_res.push_back( to_string(intrincMatrix.at<double>(0,0))+" "+to_string(intrincMatrix.at<double>(0,1))+" "+to_string(intrincMatrix.at<double>(0,2)) );
			calibraion_res.push_back( to_string(intrincMatrix.at<double>(1,0))+" "+to_string(intrincMatrix.at<double>(1,1))+" "+to_string(intrincMatrix.at<double>(1,2)) );
			calibraion_res.push_back( to_string(intrincMatrix.at<double>(2,0))+" "+to_string(intrincMatrix.at<double>(2,1))+" "+to_string(intrincMatrix.at<double>(2,2)) );
			calibraion_res.push_back(to_string(distParameter.at<double>(0,0))+" "+
						to_string(distParameter.at<double>(0,1))+" "+
						to_string(distParameter.at<double>(0,2))+" "+
						to_string(distParameter.at<double>(0,3))+" "+
						to_string(distParameter.at<double>(0,4)));
			FileIO::WriteSting2Txt(data_path,calibraion_res);
			
			vector<string> all_plane;
			for(int i=0;i<imgnames.size();i++)
		    {
				if((cam_planes.find(i)!=cam_planes.end())&&(lidar_planes.find(i)!=lidar_planes.end()))
				{
					all_plane.push_back(to_string(lidar_planes[i][0](0))+" "+to_string(lidar_planes[i][0](1))+" "+to_string(lidar_planes[i][0](2))+" "+to_string(lidar_planes[i][0](3))+
					" "+to_string(cam_planes[i][0](0))+" "+to_string(cam_planes[i][0](1))+" "+to_string(cam_planes[i][0](2))+" "+to_string(cam_planes[i][0](3)));
					all_plane.push_back(to_string(lidar_planes[i][1](0))+" "+to_string(lidar_planes[i][1](1))+" "+to_string(lidar_planes[i][1](2))+" "+to_string(lidar_planes[i][1](3))+
					" "+to_string(cam_planes[i][1](0))+" "+to_string(cam_planes[i][1](1))+" "+to_string(cam_planes[i][1](2))+" "+to_string(cam_planes[i][1](3)));
					all_plane.push_back(to_string(lidar_planes[i][2](0))+" "+to_string(lidar_planes[i][2](1))+" "+to_string(lidar_planes[i][2](2))+" "+to_string(lidar_planes[i][2](3))+
					" "+to_string(cam_planes[i][2](0))+" "+to_string(cam_planes[i][2](1))+" "+to_string(cam_planes[i][2](2))+" "+to_string(cam_planes[i][2](3)));
				}
		    }
            FileIO::WriteSting2Txt("/home/xuan/catkin_ws/src/calibration/Planes.txt",all_plane);

			//求解完外参之后，将3D点投影到图像上进行比较
			cout<<"3.Start 3D Points Projection!!!!!"<<endl;
			cvNamedWindow("Image");
			Eigen::Matrix3f Rcl_float;
			Eigen::Vector3f tcl_float;
			Eigen::Matrix3f K;
			for(int i=0;i<3;i++)
			{
				tcl_float[i] = (float)tcl[i];
				for(int j=0;j<3;j++)
				{
					Rcl_float(i,j) = (float)Rcl(i,j);
					K(i,j) = (float)intrincMatrix.at<double>(i,j);
				}
			}
			cout<<"K = "<<K<<endl;
			for(int i=0;i<imgnames.size();i++)
			{
				vector<string> name = read_format(imgnames[i]," ");
				img=cv::imread(path_root+"/leftImg/left_"+name[1]+".png",cv::IMREAD_UNCHANGED);
				cv::Mat rectify_img;
				cv::remap(img,rectify_img,mapx,mapy,INTER_LINEAR);
					
				vector<vector<float>> cloud_points = FileIO::ReadTxt2Float(path_root+"/lidar/"+name[0]+".txt");
				pcl::PointCloud<pcl::PointXYZ> framecloud;
				for(int j=0;j<cloud_points.size();j++)
				{
					pcl::PointXYZ temp;
					temp.x = cloud_points[j][0]; 
					temp.y = cloud_points[j][1];
					temp.z = cloud_points[j][2];
					framecloud.push_back(temp);
				}
				
				//然后调用绘制函数进行绘图
				pcl::PointCloud<pcl::PointXYZRGB> colPoints;//发布和图像融合后的彩色点
				cv::Mat dra_img = DrawImage::DrawPointCloud(framecloud,rectify_img,Rcl_float,tcl_float,K,colPoints);
				cv::imshow("Image",dra_img);
				cv::waitKey(waittime);
			}
			cv::destroyAllWindows();
		}//能够找到配对的激光和视觉平面
	}//外参标定结束
	else
		cout<<"Can not find matched lidar plane and camera plane"<<endl;
		
   
    if(UseExtraData)
    {
		//选择另外的数据集进行测试
		//1.首先读取lidar_to_camera.txt结果
		string data_path;
		n.getParam("/calibration/ExtraData_path", data_path);//根目录
		vector<vector<float>> content = FileIO::ReadTxt2Float(path_Ext);//path_Ext
		Eigen::Matrix3f Rcl_float;
		Eigen::Vector3f tcl_float;
		Eigen::Matrix3f K;
		
        cv::Mat K_cv = Mat(3,3,CV_64FC1,Scalar::all(0));//摄像机的畸变系数
        cv::Mat distParameter_res = Mat(1,5,CV_64FC1,Scalar::all(0));//摄像机的畸变系数

			Rcl_float(0,0) = content[0][0];Rcl_float(0,1) = content[0][1];Rcl_float(0,2) = content[0][2];
			Rcl_float(1,0) = content[1][0];Rcl_float(1,1) = content[1][1];Rcl_float(1,2) = content[1][2];
			Rcl_float(2,0) = content[2][0];Rcl_float(2,1) = content[2][1];Rcl_float(2,2) = content[2][2];
			tcl_float[0] = content[3][0];
			tcl_float[1] = content[3][1];
			tcl_float[2] = content[3][2];

			K(0,0) =  content[4][0];K(0,1) =  content[4][1];K(0,2) =  content[4][2];
			K(1,0) =  content[5][0];K(1,1) =  content[5][1];K(1,2) =  content[5][2];
			K(2,0) =  content[6][0];K(2,1) =  content[6][1];K(2,2) =  content[6][2];
			
			K_cv.at<double>(0,0) = K(0,0);K_cv.at<double>(0,1) = K(0,1);K_cv.at<double>(0,2) = K(0,2);
			K_cv.at<double>(1,0) = K(1,0);K_cv.at<double>(1,1) = K(1,1);K_cv.at<double>(1,2) = K(1,2);
			K_cv.at<double>(2,0) = K(2,0);K_cv.at<double>(2,1) = K(2,1);K_cv.at<double>(2,2) = K(2,2);

			distParameter_res.at<double>(0,0) = content[7][0];
			distParameter_res.at<double>(0,1) = content[7][1];
			distParameter_res.at<double>(0,2) = content[7][2];
			distParameter_res.at<double>(0,3) = content[7][3];
			distParameter_res.at<double>(0,4) = content[7][4];
		

		cout<<"Rcl = "<<endl<<Rcl_float<<endl;
		cout<<"tcl = "<<endl<<tcl_float.transpose()<<endl;
		cout<<"K = "<<endl<<intrincMatrix<<endl;
		cout<<"dist parameter = "<<endl<<distParameter_res<<endl;
		Mat mapx,mapy;
		Size image_size;//图像大小
		vector<string> data_imgnames = FileIO::ReadTxt2String(data_path+"names.txt",false);
		cvNamedWindow("Image");
		for(int i=0;i<data_imgnames.size();i++)
		{
			
			vector<string> name = read_format(data_imgnames[i]," ");
			img=cv::imread(data_path+"leftImg/left_"+name[1]+".png",cv::IMREAD_UNCHANGED);
			if(i==0)
			{
			image_size.width = img.cols;
			image_size.height = img.rows;
			cv::initUndistortRectifyMap(K_cv,distParameter_res,Mat::eye(3,3,CV_32FC1),K_cv,image_size,CV_32FC1,mapx,mapy);
			}
			cv::Mat rectify_img;
			cv::remap(img,rectify_img,mapx,mapy,INTER_LINEAR);
				
			vector<vector<float>> cloud_points = FileIO::ReadTxt2Float(data_path+"lidar/"+name[0]+".txt");
			pcl::PointCloud<pcl::PointXYZ> framecloud;
			for(int j=0;j<cloud_points.size();j++)
			{
			pcl::PointXYZ temp;
			temp.x = cloud_points[j][0]; 
			temp.y = cloud_points[j][1];
			temp.z = cloud_points[j][2];
			framecloud.push_back(temp);
			}
			
			//然后调用绘制函数进行绘图
			pcl::PointCloud<pcl::PointXYZRGB> colPoints;//发布和图像融合后的彩色点
			cv::Mat dra_img = DrawImage::DrawPointCloud(framecloud,rectify_img,Rcl_float,tcl_float,K,colPoints);
			cv::imshow("Image",dra_img);
			cv::waitKey(waittime);
		}
		cv::destroyAllWindows();
    }
}
