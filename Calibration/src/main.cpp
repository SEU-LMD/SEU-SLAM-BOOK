#include <stdio.h>    
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include <thread>
#include <mutex>
#include <condition_variable>

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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
 
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

#include "Segment.h"

#include <opencv/cv.h>    
#include <opencv2/imgproc.hpp>
#include <opencv/highgui.h>    
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> //为了求特征值

#include "DealString.h"
#include "FileIO.h"
#include "Segment.h"
#include "ICP.h"
#include "PCLlib.h"
#include "Camera.h"

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

using namespace std;

#define corner_height 4
#define corner_width 6
#define N_SCANS 16

cv::Mat distParameter;//摄像机的畸变系数
cv::Mat intrincMatrix;
map<int,vector<Eigen::Vector4f>> lidar_planes;
map<int,vector<Eigen::Vector4f>> cam_planes;//根据world plane和相机的位姿矩阵得到相机坐标系下的平面坐标，顺序是右左下
vector<vector<Eigen::Vector3f>> lidarCenters;
vector<int> camera_valid_frame;
int max_thread;

string path_root;
string lidar_config_path;
string path_image;
std::mutex mutexpcd;
bool use_NDT=true;
bool first_frame=true;
bool showPCD=true;
bool FirstLidarFrame=true;

vector<pcl::PointXYZ> board_points;
struct Data
{
    pcl::PointCloud<pcl::PointXYZI> global_map;
	vector<Eigen::Matrix4f> lidar_poses;
    vector<int> indexs;
};

class PCData
{
    private:
        std::mutex m;
        Data data;
    public:
        void pose_push(Eigen::Matrix4f pose)
        {
            lock_guard<std::mutex> guard1(m);
            data.lidar_poses.push_back(pose);
        };
        void index_push(int i)
        {
            lock_guard<std::mutex> guard1(m);
            data.indexs.push_back(i);
        };
        int get_index_size()
        {
            lock_guard<std::mutex> guard1(m);
            return data.indexs.size();
        };
        int get_pose_size()
        {
            lock_guard<std::mutex> guard1(m);
            return data.lidar_poses.size();
        };
        int need_to_ndt()
        {
            lock_guard<std::mutex> guard1(m);
            return data.indexs.size()-data.lidar_poses.size();
        };
        int get_index(int i)
        {
            lock_guard<std::mutex> guard1(m);
            return data.indexs[i];
        };
        Eigen::Matrix4f get_pose(int i)
        {
            lock_guard<std::mutex> guard1(m);
            return data.lidar_poses[i];
        };
        bool isfinish()
        {
            lock_guard<std::mutex> guard1(m);
            if(data.indexs.size()==data.lidar_poses.size())
                return true;
            else
                return false;
        }
};

PCData pcdata1;
std::condition_variable pcd_cond,pcd_cond2;

void SolveNDT()
{
    PCLlib::NDT ndt(lidar_config_path);
    std::unique_lock<std::mutex> sn(mutexpcd);
    vector<string> imgnames = FileIO::ReadTxt2String(path_image,false);
    while(pcdata1.get_index_size()==0)
    {
        pcd_cond.wait(sn);
    }
    //sn.lock();
    cout<<"Thread:1"<<endl;
    cout<<"Start PointClouds NDT"<<endl<<endl;
    sn.unlock();
    
    pcd_cond2.notify_one();
    showPCD=false;
    int nNDT=0;
    while(use_NDT)
    {
        if(pcdata1.need_to_ndt())
        {
            cout<<"ndt:"<<pcdata1.get_index(nNDT)<<endl;
            vector<string> name = read_format(imgnames[pcdata1.get_index(nNDT)]," ");//读取点云文件//i
            string filename = path_root+"/lidar/"+name[0]+".txt";
            pcl::PointCloud<pcl::PointXYZI> cloud=Load_PointCloud(filename,pcdata1.get_index(nNDT));
            Eigen::Matrix4f Twl;
            ndt.AddCurrentCloud(cloud,Twl);
            pcdata1.pose_push(Twl);
            nNDT++;
        }
    }
}

void ShowPointCloud()
{
    vector<string> imgnames = FileIO::ReadTxt2String(path_image,false);
    std::unique_lock<std::mutex> sn(mutexpcd);
    while(showPCD)
    {
        pcd_cond2.wait(sn);
    }
    cout<<"Thread:2"<<endl;
    cout<<"Show GlobalPointClouds"<<endl<<endl;
    sn.unlock();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());//当前帧变换到世界地图中的坐标
    int nGPC=0;
    pcl::PointCloud<pcl::PointXYZI> global_map;
    while(use_NDT||nGPC==pcdata1.get_index_size())
    {
        if(nGPC<=pcdata1.get_pose_size())
        {
            int index=pcdata1.get_index(nGPC);
            vector<string> name = read_format(imgnames[index]," ");//读取点云文件//i
            string filename = path_root+"/lidar/"+name[0]+".txt";
            pcl::PointCloud<pcl::PointXYZI> cloud=Load_PointCloud(filename,index);
            //Eigen::Matrix4f Twl=pcdata1.get_pose(index);
            //pcl::transformPointCloud(cloud, *transformed_scan_ptr, Twl);
            //global_map += *transformed_scan_ptr;//将当前帧点云融合到世界坐标当中
            //ros::Time ros_timestamp = ros::Time::now();
            nGPC++;

            // ros::Time ros_timestamp = ros::Time::now();
			// if(map_pub.getNumSubscribers()>0)//全局地图
			// {
			// 	sensor_msgs::PointCloud2 map_msg;
			// 	pcl::toROSMsg(global_map,map_msg);
			// 	map_msg.header.frame_id = "/velodyne";
			// 	map_msg.header.stamp = ros::Time::now();
			// 	map_pub.publish(map_msg);
			// }
			// //发布位姿信息
			// geometry_msgs::PoseStamped ndtpose_msg;
			// ndtpose_msg.header.frame_id = "/velodyne";
			// ndtpose_msg.header.stamp = ros::Time::now();
			// ndtpose_msg.pose.position.x = Twl(0,3);
			// ndtpose_msg.pose.position.y = Twl(1,3);
			// ndtpose_msg.pose.position.z = Twl(2,3);
			// Eigen::Matrix3f R_ndt = Twl.block<3,3>(0,0);
			// Eigen::Quaternionf q(R_ndt);
			// ndtpose_msg.pose.orientation.x = q.x();
			// ndtpose_msg.pose.orientation.y = q.y();
			// ndtpose_msg.pose.orientation.z = q.z();
			// ndtpose_msg.pose.orientation.w = q.w();
			
			// path_msg.poses.push_back(ndtpose_msg); 
			// if(pose_pub.getNumSubscribers()>0)//ndt实时的位姿
			// {
			// 	pose_pub.publish(ndtpose_msg);
			// }
			// if(path_pub.getNumSubscribers()>0)//ndt的轨迹
			// {
			// 	path_msg.header.frame_id = "/velodyne";
			// 	path_msg.header.stamp = ros::Time::now();
			// 	path_msg.poses.push_back(ndtpose_msg);
			// 	path_pub.publish(path_msg);
			// }
        }
        else
            std::this_thread::sleep_for(std::chrono::minutes(1));
    }
}

void waypointCallback(const geometry_msgs::PointStampedConstPtr& waypoint)
{
  //成功接收到了点的坐标信息
  pcl::PointXYZ board_point;
  board_point.x=waypoint->point.x;
  board_point.y=waypoint->point.y;
  board_point.z=waypoint->point.z;
  board_points.push_back(board_point);
  cout<<board_point<<endl;
}

void calculateExtrinsicError(string planes_path,string extrinsic_path)
{
    vector<vector<float>> content = FileIO::ReadTxt2Float(extrinsic_path,true);//path_Ext
    Eigen::Matrix3f Rcl_float;
    Eigen::Vector3f tcl_float;
    cv::Mat K_cv = Mat(3,3,CV_64FC1,Scalar::all(0));//摄像机的畸变系数
    cv::Mat distParameter_res = Mat(1,5,CV_64FC1,Scalar::all(0));//摄像机的畸变系数
    Rcl_float(0,0) = content[0][0];Rcl_float(0,1) = content[0][1];Rcl_float(0,2) = content[0][2];
    Rcl_float(1,0) = content[1][0];Rcl_float(1,1) = content[1][1];Rcl_float(1,2) = content[1][2];
    Rcl_float(2,0) = content[2][0];Rcl_float(2,1) = content[2][1];Rcl_float(2,2) = content[2][2];
    tcl_float[0] = content[3][0];
    tcl_float[1] = content[3][1];
    tcl_float[2] = content[3][2];

    vector<vector<float>> cal_planes = FileIO::ReadTxt2Float(planes_path);
    vector<Eigen::Vector4f> temp_pc,temp_pl;
    vector<vector<Eigen::Vector4f>> lidarPlanes;
	vector<vector<Eigen::Vector4f>> camPlanes;
    for(int j=0;j<cal_planes.size()/3;j++)
    {
        for(int i=0;i<3;i++)
        {
            float a = cal_planes[3*j+i][0]; 
            float b = cal_planes[3*j+i][1];
            float c = cal_planes[3*j+i][2];
            float d = cal_planes[3*j+i][3];
            temp_pl.push_back(Eigen::Vector4f(a,b,c,d));
            a = cal_planes[3*j+i][4]; 
            b = cal_planes[3*j+i][5];
            c = cal_planes[3*j+i][6];
            d = cal_planes[3*j+i][7];
            temp_pc.push_back(Eigen::Vector4f(a,b,c,d));
        }
        lidarPlanes.push_back(temp_pl);
        camPlanes.push_back(temp_pc);
        temp_pl.clear();
        temp_pc.clear();
    }
    int n=lidarPlanes.size();
    float n_error=0,d_error=0,maxN=0,maxD=0,ind1,ind2;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<3;j++)
        {
            Eigen::Vector3f nl(lidarPlanes[i][j][0],lidarPlanes[i][j][1],lidarPlanes[i][j][2]);
            Eigen::Vector3f nc(camPlanes[i][j][0],camPlanes[i][j][1],camPlanes[i][j][2]);
            Eigen::Vector3f nlc=Rcl_float*nl;
            float axb,a,b;
            axb=nlc[0]*nc[0]+nlc[1]*nc[1]+nlc[2]*nc[2];
            a=sqrt(nlc[0]*nlc[0]+nlc[1]*nlc[1]+nlc[2]*nlc[2]);
            b=sqrt(nc[0]*nc[0]+nc[1]*nc[1]+nc[2]*nc[2]);
            float error=acos(axb/(a*b));
            n_error+=error;

            float cam_D = lidarPlanes[i][j][3]+(nl[0]*tcl_float[0]+nl[1]*tcl_float[1]+nl[2]*tcl_float[2]);
            float error2=camPlanes[i][j][3]-cam_D;
            d_error+=abs(error2);
            cout<<i<<" "<<j<<" "<<error/PI*180<<" "<<error2<<endl;
            if(error>maxN)
            {
                ind1=i;
                ind2=j;
                maxN=error;
            }
            if(error2>maxD)
            {
                maxD=error2;
            }
        }
        cout<<endl;
    }
    cout<<"mean error normal="<<(n_error/3/n)/PI*180<<endl;
    cout<<"mean error distance="<<d_error/3/n<<endl;
    cout<<"index:"<<ind1<<"boards:"<<ind2<<"max error normal="<<maxN/PI*180<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_calibration");
	ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/MapCloud",1);//聚类的点云
    nav_msgs::Path path_msg;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/Path", 1);//轨迹
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/Pose", 1);//ndt输出得到的pose
    //设置路径
	n.getParam("/camera_calibration/root", path_root);//根目录
    string path_coor = path_root+"/3D.txt";
	string path_coor_planar = path_root+"/3D_planar.txt";
	path_image = path_root+"/names.txt";
    //设置程序标定模式
    int getIntrincMatrix,getExtrinsic;
    n.getParam("/camera_calibration/IntrincMatrix", getIntrincMatrix);//是否标定相机
	n.getParam("/camera_calibration/Extrinsic", getExtrinsic);//
    n.getParam("/camera_calibration/lidar_config_path", lidar_config_path);
    
    //max_thread=thread::hardware_concurrency();    
    //thread NDT1(SolveNDT),ShowPC(ShowPointCloud);
    //NDT1.detach();
    //ShowPC.detach();

    //标定
    if(getIntrincMatrix)
    {
        CAMERA::Camera Cam(corner_height,corner_width,path_root);
        vector<string> imgnames = FileIO::ReadTxt2String(path_image,false);
        for(int i=0;i<imgnames.size();i++)//载入标定图像imgnames.size()
        {
            vector<string> name = read_format(imgnames[i]," ");
            string filename=path_root+"/leftImg/left_"+name[1]+".png";
            bool ischoose=Cam.add(filename);
            if(ischoose)
            {
                camera_valid_frame.push_back(i);
        //         //cout<<i<<"is choosen"<<endl;
        //         pcdata1.index_push(i);
        //         if(first_frame)
        //         {
        //             pcd_cond.notify_one();
        //             first_frame=!first_frame;
        //         }
            }
        }
        Cam.calibration();//标定相机内参
        Cam.show();//显示标定结果
        Cam.GetIntrincMatrix(intrincMatrix);
        Cam.GetDistParameter(distParameter);
        Cam.GetPlanesModels(cam_planes);//获取标定板平面模型
        cv::destroyAllWindows();
        // while(!pcdata1.isfinish())
        // {
        //     std::this_thread::sleep_for(std::chrono::minutes(1));
        // }
        // use_NDT=false;
        // cout<<"finish NDT"<<endl;
    }

    //lidar
    if(getExtrinsic)
    {
        // camera_valid_frame.push_back(0);
        // camera_valid_frame.push_back(10);
        // camera_valid_frame.push_back(20);
        ros::Subscriber waypoint_sub_ = n.subscribe("/clicked_point", 100, waypointCallback);
        pcl::PointCloud<pcl::PointXYZI> global_map,target,source,choose_points;
        Eigen::Matrix4f Twl,Tlw;
        Eigen::Vector4f plane_model;
        Eigen::Vector3f center;
        PCLlib::NDT ndt(lidar_config_path);//ndt初始化构造函数
        std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);
        vector<string> imgnames = FileIO::ReadTxt2String(path_image,false);
        vector<Eigen::Vector4f> tmp_plane_params;
        vector<Eigen::Vector3f> plane_center;
        for(int i=0;i<camera_valid_frame.size();i++)
        {
            int ind=camera_valid_frame[i];
            vector<string> name = read_format(imgnames[ind]," ");//读取点云文件
            string filename=path_root+"/lidar/"+name[0]+".txt";
            vector<vector<float>> cloud_points = FileIO::ReadTxt2Float(filename);
            laserCloudScans=GetScanPointCloud(cloud_points,N_SCANS,90,270);//按照线对点云排序
            global_map.clear();
            
            if(!FirstLidarFrame)
            {
                target=Load_PointCloud(filename,ind);
                //cout<<source.size()<<" "<<target.size()<<endl;
                cout<<"start transform index:"<<ind<<endl;
                cout<<i<<" frames has transformed,num of need to NDT:"<<camera_valid_frame.size()-i<<endl;
                ndt.AddCurrentCloud(target,Twl);
                Tlw=Twl.inverse();
                pcl::PointCloud<pcl::PointXYZI>::Ptr trans_ptr(new pcl::PointCloud<pcl::PointXYZI>());//当前帧变换到世界地图中的坐标
                //pcl::PointCloud<pcl::PointXYZI>::Ptr trans_ptr2(new pcl::PointCloud<pcl::PointXYZI>());//当前帧变换到世界地图中的坐标
			    pcl::transformPointCloud(choose_points, *trans_ptr, Tlw);//将当前帧的点云scan_ptr变化到世界地图下得到transformed_scan_ptr
                //pcl::transformPointCloud(target, *trans_ptr2, Twl);//将当前帧的点云scan_ptr变化到世界地图下得到transformed_scan_ptr

                for(int j=0;j<3;j++)
                {
                    pcl::PointXYZ Bpoint;
                    Bpoint.x=trans_ptr->points[j].x;
                    Bpoint.y=trans_ptr->points[j].y;
                    Bpoint.z=trans_ptr->points[j].z;
                    // cout<<choose_points[j]<<endl;
                    // cout<<Bpoint<<endl;
                    PCLlib::DetectPlane(plane_model,laserCloudScans,Bpoint,center);
                    tmp_plane_params.push_back(plane_model);
                    plane_center.push_back(center);
                    global_map.clear();
                    for(int j=0;j<N_SCANS;j++)
                    {
                        global_map +=laserCloudScans[j];
                    }
                    if(ros::ok())//全局地图
                    {
                        sensor_msgs::PointCloud2 map_msg;
                        pcl::toROSMsg(global_map,map_msg);
                        map_msg.header.frame_id = "/velodyne";
                        map_msg.header.stamp = ros::Time::now();
                        map_pub.publish(map_msg);
                        //std::this_thread::sleep_for(std::chrono::seconds(1));
                        
                    }
                }
                if(PCLlib::CheckBoardPlane(tmp_plane_params))
                {
                    cout<<i<<" index:"<<ind<<" is choosen"<<endl<<endl;
                    lidar_planes.insert(pair<int,vector<Eigen::Vector4f>>(ind,tmp_plane_params));
                    lidarCenters.push_back(plane_center);
                    cout<<"The numbers of lidar-camera-calibration now:"<<lidar_planes.size()<<endl;
                }
                tmp_plane_params.clear();
                plane_center.clear();
            }
            while(FirstLidarFrame)
            {
                for(int j=0;j<N_SCANS;j++)
                {
                    global_map +=laserCloudScans[j];
                }
                ros::Time ros_timestamp = ros::Time::now();
                while(map_pub.getNumSubscribers()<=0){//等待rviz完全启动
                std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                if(ros::ok())//全局地图
                {
                    sensor_msgs::PointCloud2 map_msg;
                    pcl::toROSMsg(global_map,map_msg);
                    map_msg.header.frame_id = "/velodyne";
                    map_msg.header.stamp = ros::Time::now();
                    map_pub.publish(map_msg);
                }
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if(board_points.size()>0&&board_points.size()!=tmp_plane_params.size())
                {
                    PCLlib::DetectPlane(plane_model,laserCloudScans,board_points[tmp_plane_params.size()],center);
                    plane_center.push_back(center);
                    tmp_plane_params.push_back(plane_model);
                    global_map.clear();
                    for(int j=0;j<N_SCANS;j++)
                    {
                        global_map +=laserCloudScans[j];
                    }
                    if(ros::ok())//全局地图
                    {
                        sensor_msgs::PointCloud2 map_msg;
                        pcl::toROSMsg(global_map,map_msg);
                        map_msg.header.frame_id = "/velodyne";
                        map_msg.header.stamp = ros::Time::now();
                        map_pub.publish(map_msg);
                    }
                }
                if(tmp_plane_params.size()==3)
                {
                    bool choose=PCLlib::CheckBoardPlane(tmp_plane_params);
                    if(choose)
                    {
                        lidar_planes.insert(pair<int,vector<Eigen::Vector4f>>(ind,tmp_plane_params));
                        lidarCenters.push_back(plane_center);
                    }
                    for(int i=0;i<3;i++)
                    {
                        pcl::PointXYZI p;
                        p.x=board_points[i].x;
                        p.y=board_points[i].y;
                        p.z=board_points[i].z;
                        choose_points.push_back(p);
                    }
                    FirstLidarFrame=false;
                    source=Load_PointCloud(filename,ind);
                    ndt.AddCurrentCloud(source,Twl);
                    tmp_plane_params.clear();
                    plane_center.clear();
                    cout<<"choose board points finish"<<endl;
                    break;
                }
            }
            for(int i=0;i<laserCloudScans.size();i++)
            {
                laserCloudScans[i].clear();
            }
        }
        cout<<"get valid lidar planes:"<<lidar_planes.size()<<endl;
        
        Eigen::Matrix3d Rcl_;
	    Eigen::Vector3d Tcl_;
        if(getExtrinsic)
        {
            cout<<"0.Start Extrinsic Optimization!!!!!!!!!!!"<<endl;
            //最后开始计算外参
            vector<vector<Eigen::Vector4f>> lidarPlanes;
            vector<vector<Eigen::Vector4f>> camPlanes;
            
            for(map<int,vector<Eigen::Vector4f>>::iterator iter=lidar_planes.begin();iter!=lidar_planes.end();iter++)
            {   vector<Eigen::Vector3f> lc;
                Eigen::Vector3f p;
                int i=iter->first;
                //这一frame同时检测到了相机和激光的平面，才会加入到优化中
                if((cam_planes.find(i)!=cam_planes.end())&&(lidar_planes.find(i)!=lidar_planes.end()))
                {

                    lidarPlanes.push_back(lidar_planes[i]);
                    camPlanes.push_back(cam_planes[i]);
                    //cout<<"The frame indx to join extrinsic optimization = "<<i<<endl;
                    // //cout<<" lidar plane norm= "<<lidar_planes[i][0].head<3>().norm()<<" "<<lidar_planes[i][1].head<3>().norm()<<endl;
                    // //cout<<" cam plane norm = "<<cam_planes[i][0].head<3>().norm()<<" "<<cam_planes[i][1].head<3>().norm()<<endl;
                    //cout<<" lidar plane = "<<lidar_planes[i][0].transpose()<<" "<<lidar_planes[i][1].transpose()<<" "<<lidar_planes[i][2].transpose()<<endl;
                    //cout<<" cam plane  = "<<cam_planes[i][0].transpose()<<" "<<cam_planes[i][1].transpose()<<" "<<cam_planes[i][2].transpose()<<endl;
                }
            }
            cout<<"Num of frames to optimize extrisinc = "<<lidarPlanes.size()<<endl;
            cout<<"lidarCenters.size():"<<lidarCenters.size()<<" "<<lidarCenters[0].size()<<endl;
            //为了计算得到tc,L和Rc,L
            //先拟合得到Rc,l 然后再线性计算得到tc,l
            if(lidarPlanes.size()!=0)
            {
                cout<<"1.Start Rotatoin Matrix Optimization!!!!!"<<endl;
                Eigen::Matrix3d Rcl;
                Eigen::Vector3d tcl,pl,plc;
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
                        lidar_normals.push_back(cv::Point3d((double)lidarPlanes[i][j](0),(double)lidarPlanes[i][j](1),(double)lidarPlanes[i][j](2)));
                        cam_normals.push_back(cv::Point3d((double)camPlanes[i][j](0),(double)camPlanes[i][j](1),(double)camPlanes[i][j](2)));
                        A(3*i+j,0) = (double)camPlanes[i][j](0);
                        A(3*i+j,1) = (double)camPlanes[i][j](1);
                        A(3*i+j,2) = (double)camPlanes[i][j](2);
                        //b(3*i+j,0) = (double)lidarPlanes[i][j](3)-(double)camPlanes[i][j](3);
                    }
                }
                ICP::pose_estimation_3d3d (cam_normals,lidar_normals,Rcl,tcl);
                cout<<"ICP tcl(should be equal to zero) = "<<tcl.transpose()<<endl;
                cout<<"Final Rcl = "<<endl<<Rcl<<endl;
                cout<<"2.Start translation vector Optimization!!!!!"<<endl;
                for(int i=0;i<lidarPlanes.size();i++)
                {
                    for(int j=0;j<3;j++)
                    {
                        pl=Eigen::Vector3d((double)lidarCenters[i][j][0],(double)lidarCenters[i][j][1],(double)lidarCenters[i][j][2]);
                        // pl[0]=(double)lidarCenters[i][j](0);
                        // cout<<"1"<<endl;
                        // pl[1]=(double)lidarCenters[i][j][1];
                        // pl[2]=(double)lidarCenters[i][j][2];
                        plc=Rcl*pl;
                        cout<<"plc:"<<plc<<endl;
                        b(3*i+j,0) =-(double)camPlanes[i][j][3]-((double)camPlanes[i][j][0]*pl[0]+(double)camPlanes[i][j][1]*pl[1]+(double)camPlanes[i][j][2]*pl[2]);
                    }
                }
                Eigen::EigenSolver<Eigen::MatrixXd> es( A.transpose()*A );
                cout<<"A = "<<endl<<A<<endl;
                cout<<"ATA = "<<endl<<A.transpose()*A<<endl;
                cout<<"b = "<<endl<<b<<endl;

                Eigen::MatrixXd tcl_xd = A.colPivHouseholderQr().solve(b);
                
                cout<<"求解tcl 矩阵的特征值 = "<<endl<<es.eigenvalues()<<endl;
                tcl(0) = tcl_xd(0);
                tcl(1) = tcl_xd(1);
                tcl(2) = tcl_xd(2);
                cout<<"Num of frames to optimize extrisinc = "<<lidarPlanes.size()<<endl;
                cout<<"Final Tcl = "<<endl<<Rcl<<endl;
                cout<<tcl.transpose()<<endl;
                //
                string data_path;
                n.getParam("/calibration/path_Extrinsic", data_path);//根目录
                Rcl_=Rcl;
                Tcl_=tcl;
                //FileIO::WriteSenserParameters(Rcl,tcl,intrincMatrix,distParameter,data_path);
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
                FileIO::WriteSting2Txt("/home/xuan/catkin_ws/src/camera_calibration/Planes.txt",all_plane);

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
            }//能够找到配对的激光和视觉平面
        }//外参标定结束
        else
            cout<<"Can not find matched lidar plane and camera plane"<<endl;
        

    }
    
    //lidar
    string path1,path2;
    path1="/home/xuan/catkin_ws/src/camera_calibration/Planes.txt";
    path2="/home/xuan/catkin_ws/src/camera_calibration/conf/Extrinsic.txt";
    calculateExtrinsicError(path1,path2);
}