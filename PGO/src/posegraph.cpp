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

#include <opencv2/core/core.hpp>//为了使用它的读取yaml文件函数
#include <opencv2/calib3d/calib3d.hpp>//这个函数的作用引入罗德里格斯公式和pnp的代码
#include <opencv2/highgui/highgui.hpp>//为了图像的显示
#include <opencv/cv.h>   
#include <opencv2/opencv.hpp>

#include <string.h>
#include <math.h>
#include <iostream>
//用于新建目录
#include <sys/stat.h>
#include <ctime> //用于随机数字的初始化
#include <cstdlib>//用于生成随机数
#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include "FileIO.h"
#include "G2OEdge.h"
#include "types.h"
#include "Optimizer.h"
#include "Error.h"
#include "iSAM_posegraph.h"
#include "Ceres_posegraph.h"
#include "LAGO_linear.h"
#include "ICP.h"
using namespace std;
using namespace cv;
using namespace posegraph;

//kitti数据集提供了姿态的真值
//KITTI00输入的参数是 /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_00_vo.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_00_gt.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_00_cs.g2o
//KITTI02          /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_02_vo.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_02_gt.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/KITTI_02_cs.g2o
//下面的4个数据集没有姿态的真值
//Pittsburgh_A数据集 /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_A_vo.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_A_gt.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_A_cs.g2o
//Pittsburgh_B数据集 /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_B_vo.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_B_gt.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_B_cs.g2o
//Pittsburgh_C数据集 /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_C_vo.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_C_gt.g2o /home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/Pittsburgh_C_cs.g2o
//下面是 tum数据跑得的结果(注意了tum的数据集只存在单个回环)
//freiburg1_room数据集:/home/fyy/Documents/my_code/posegraph/data/freiburg1_room_vo.txt /home/fyy/Documents/my_code/posegraph/data/freiburg1_room_gt.txt /home/fyy/Documents/my_code/posegraph/data/freiburg1_room_CameraTrajectory.txt
//freiburg2_desk数据集:/home/fyy/Documents/my_code/posegraph/data/freiburg2_desk_vo.txt /home/fyy/Documents/my_code/posegraph/data/freiburg2_desk_gt.txt /home/fyy/Documents/my_code/posegraph/data/freiburg2_desk_CameraTrajectory.txt
//freiburg3_long_office_household数据集:/home/fyy/Documents/my_code/posegraph/data/freiburg3_long_office_vo.txt /home/fyy/Documents/my_code/posegraph/data/freiburg3_long_office_gt.txt /home/fyy/Documents/my_code/posegraph/data/freiburg3_long_office_CameraTrajectory.txt

//为了测gtsam时使用读取gtsam中的数据集合/home/fyy/Documents/lib/gtsam4.0/examples/Data/pose3example.txt
int main(int argc, char **argv)
{
  //首选删除数据目录中的所有文件
  Delete_path("/home/fyy/Documents/my_code/posegraph/build/subset_result");
  Delete_path("/home/fyy/Documents/my_code/posegraph/build/whole_result");
  
  //首选读取g2o文件
  string filename_vo = argv[1];
  vector<pose> poses_vo,pose_gt;
  vector<edge> edges_vo,edges_gt;//实际上edges_gt应该为空
  int mistake_vo = read_G2O_file(filename_vo,edges_vo,poses_vo);
  /*我们截取部分的数据进行优化
 for(vector<pose>::iterator iter = poses_vo.begin();iter!=poses_vo.end();)
 {
    if(iter->id>13282)
    {
      iter = poses_vo.erase(iter);
    }
    else
    {
      iter++;
    }
 }
 for(vector<edge>::iterator iter = edges_vo.begin();iter!=edges_vo.end();)
 {
    if((iter->id1>13282)||(iter->id2>13282))
    {
      iter = edges_vo.erase(iter);
    }
    else
    {
      iter++;
    }
 }*/
  /*
  vector<double> distances;//这个变量的序号是帧的序号，数值是相机一共走过的距离
  double acc_distances=0;
  for(int i=0;i<poses_vo.size();i++)
  {
    if(i==0)
    {
      distances.push_back(acc_distances);
    }
    else
    {
      double dist = (poses_vo[i].t-poses_vo[i-1].t).norm();//这一帧和上一帧的距离
      acc_distances = acc_distances+dist;
      distances.push_back(acc_distances);
    }
  }*/
  /*
  //打印出读取到的顶点位姿态
  for(int i=0;i<poses_vo.size();i++)
  {
    Eigen::Vector3d t = poses_vo[i].t;
    Eigen::Quaterniond q = poses_vo[i].q;
    //cout<<poses_vo[i].id<<" "<<t(0)<<" "<<t(1)<<" "<<t(2)<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;  
  }*/
  //打印出得到的边的数据
  for(int i=0;i<edges_vo.size();i++)
  {
    Eigen::Vector3d t = edges_vo[i].delta_t;
    Eigen::Quaterniond q = edges_vo[i].delta_q;
    cout<<"原始数据的边 = "<<edges_vo[i].id1<<" "<<edges_vo[i].id2<<" "<<t(0)<<" "<<t(1)<<" "<<t(2)<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl; 
    Eigen::Matrix3d R1 = poses_vo[edges_vo[i].id1].q.toRotationMatrix();
    Eigen::Matrix3d R2 = poses_vo[edges_vo[i].id2].q.toRotationMatrix();
    Eigen::Vector3d t1 = poses_vo[edges_vo[i].id1].t;
    Eigen::Vector3d t2 = poses_vo[edges_vo[i].id2].t;
    Eigen::Matrix3d R_delta = R1.transpose()*R2;
    Eigen::Quaterniond q_delta ;
    q_delta = R_delta;
    Eigen::Vector3d t_delta = R1.transpose()*(t2-t1);
    cout<<"计算得到的边 = "<<edges_vo[i].id1<<" "<<edges_vo[i].id2<<" "<<t_delta(0)<<" "<<t_delta(1)<<" "<<t_delta(2)<<" "<<q_delta.x()<<" "<<q_delta.y()<<" "<<q_delta.z()<<" "<<q_delta.w()<<endl; 
  }
  
  //读取真值 并将真值写入到True.g2o文件中中
  string filename_gt = argv[2];
  if(filename_gt.find("freiburg")==filename_gt.npos)//对于弗莱堡数据集我们不读取真值,因为弗莱堡数据集中真值的时间戳没有对齐,需要使用这个数据集自己提供的python工具进行评测
  {
      int mistake_gt = read_G2O_file(filename_gt,edges_gt,pose_gt);
      /*从真值中裁剪掉回环最长的那个数据段
      for(vector<pose>::iterator iter = pose_gt.begin();iter!=pose_gt.end())
      {
	  if(iter->id>13282)
	  {
	    iter = pose_gt.erase(iter);
	  }
	  else
	  {
	    iter++;
	  }
      }*/
      
      if(filename_gt.find("Pittsburgh")!=filename_gt.npos)//将两个数据集进行align操作 只有匹兹堡数据集才使用,因为匹兹堡数据集真值和vo数据没有对齐
      {
	vector<pose> pose_gt_vo= AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	
	string path_gt = filename_gt.substr(0,filename_gt.find(".g2o"));
	path_gt = path_gt + "_align.g2o";
	cout<<path_gt<<endl;
	write_G2O_file(path_gt,edges_vo,pose_gt_vo);
      }
      string path_true = "/home/fyy/Documents/my_code/posegraph/build/whole_result/True.g2o";
      write_G2O_file(path_true,edges_vo,pose_gt);
  }
  
  srand((int)time(0));
  Add_noise(edges_vo,filename_vo);//加有噪声的边的信息
  //读取copslam的结果并得到误差指标
  /*
  string path_copslam_result = argv[3];
  vector<pose> poses_cop;
  vector<edge> edges_cop;
  read_G2O_file(path_copslam_result,edges_cop,poses_cop);
  ERRORS error_cop;
  cout<<pose_gt.size()<<" "<<poses_vo.size()<<" "<<poses_cop.size()<<endl;
  vector<pose> pose_gt_cop= AlignPose(poses_cop,pose_gt);//将真值的位移变换到vo坐标下
  error_cop = produce_error(pose_gt_cop,poses_vo,poses_cop,"COP-SLAM误差：");*/
  
  //将我们得到的边的数据加上随机噪声
  
  
  //if((mistake_gt==1)&&(mistake_vo==1)&&(pose_gt.size()==poses_vo.size()))//vo文件读取没有问题 gt文件读取文件没有问题 并且两个文件的顶点数量相同
  {
      //输出我们提出的初始化结果:
      /*
      vector<pose> initial_pose;
      initial_pose = Optimize_SE3Partition(poses_vo,edges_vo,20);
      vector<pose> pose_gt_initial= AlignPose(initial_pose,pose_gt);
      produce_error(pose_gt_initial,poses_vo,initial_pose,"我们的初始化方法误差：");*/
      
       
      //使用的是ceres代码中的优化边 使用的是参数化
      /*
      cout<<"使用CERES的边优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> optimized_pose_ceres;
      optimized_pose_ceres = Ceres_posegraph(poses_vo,edges_vo,"LM",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_ceres,"CERES_LM.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_ceres,"CERES_LM：");
      optimized_pose_ceres = Ceres_posegraph(poses_vo,edges_vo,"GN",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_ceres,"CERES_GN.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_ceres,"CERES_GN：");
      optimized_pose_ceres = Ceres_posegraph(poses_vo,edges_vo,"DL",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_ceres,"CERES_DL.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_ceres,"CERES_DL：");
      return 1;*/
      //string path_ceres = "/home/fyy/Documents/my_code/posegraph/build/whole_result/ceres.g2o";
      //string path_ceres_error = "/home/fyy/Documents/my_code/posegraph/build/whole_result/ceresError.txt";
      //write_G2O_file(path_ceres,edges_vo,optimized_pose_ceres);
      //write_error(path_ceres_error,error_ceres);
      
      /*
      //使用的是orb代码中的优化边 这个边是数值解
      cout<<"使用ORB的边优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> optimized_pose_SE3ORB;
      optimized_pose_SE3ORB = Optimize_SE3ORB(poses_vo,edges_vo,"LM",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3ORB,"ORB_LM.txt");//
      else
	  produce_error(pose_gt,poses_vo,optimized_pose_SE3ORB,"ORB_LM.txt");
      optimized_pose_SE3ORB = Optimize_SE3ORB(poses_vo,edges_vo,"GN",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3ORB,"ORB_GN.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_SE3ORB,"ORB_GN.txt");
      optimized_pose_SE3ORB = Optimize_SE3ORB(poses_vo,edges_vo,"DL",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3ORB,"ORB_DL.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_SE3ORB,"ORB_DL.txt");
      return 1;*/
      //string path_SE3G2O = "/home/fyy/Documents/my_code/posegraph/build/whole_result/orb.g2o";
      //string path_ORB_error = "/home/fyy/Documents/my_code/posegraph/build/whole_result/orbError.txt";
      //write_G2O_file(path_SE3G2O,edges_vo,optimized_pose_SE3ORB);
      //write_error(path_ORB_error,error_ORB);
      
      
      /*
      //使用的是slam14讲中的pose graph 优化方法
      cout<<"使用slam14_SE3优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> optimized_pose_SE3Lie;
      optimized_pose_SE3Lie = Optimize_SE3Lie(poses_vo,edges_vo,"LM",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3Lie,"StateRobot_LM.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_SE3Lie,"State_LM");
      optimized_pose_SE3Lie = Optimize_SE3Lie(poses_vo,edges_vo,"GN",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3Lie,"StateRobot_GN.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_SE3Lie,"State_GN");
      optimized_pose_SE3Lie = Optimize_SE3Lie(poses_vo,edges_vo,"DL",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],optimized_pose_SE3Lie,"StateRobot_DL.txt");//
      else
	produce_error(pose_gt,poses_vo,optimized_pose_SE3Lie,"State_DL");
      return 1;*/
      //string path_slam14 = "/home/fyy/Documents/my_code/posegraph/build/whole_result/slam14.g2o";
      //string path_slam14_error = "/home/fyy/Documents/my_code/posegraph/build/whole_result/slam14Error.txt";
      //write_G2O_file(path_slam14,edges_vo,optimized_pose_SE3Lie);
     //write_error(path_slam14_error,error_slam14);
     
      /*
      //使用gtsam中进行优化
      cout<<"使用gtsam优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> poses_gtsam;
      poses_gtsam = iSAM_full(poses_vo,edges_vo,"LM",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_gtsam,"GTSAM_LM.txt");//
      else
        produce_error(pose_gt,poses_vo,poses_gtsam,"GTSAM_LM:");
     
      poses_gtsam = iSAM_full(poses_vo,edges_vo,"GN",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_gtsam,"GTSAM_GN.txt");//
      else
	produce_error(pose_gt,poses_vo,poses_gtsam,"GTSAM_GN:");
      poses_gtsam = iSAM_full(poses_vo,edges_vo,"DL",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_gtsam,"GTSAM_DL.txt");//
      else
	produce_error(pose_gt,poses_vo,poses_gtsam,"GTSAM_DL:");
      return 1;*/
      //string path_gtsam = "/home/fyy/Documents/my_code/posegraph/build/whole_result/gtsam.g2o";
      //string path_gtsamfull_error =  "/home/fyy/Documents/my_code/posegraph/build/whole_result/gtsamError.txt";
      //write_G2O_file(path_gtsam, edges_vo,poses_gtsam);
      //write_error(path_gtsamfull_error,error_gtsamfull);
      
      /*
      vector<pose> initial_pose;
      initial_pose = Optimize_SE3Partition(poses_vo,edges_vo,20);
      cout<<"使用fi+t(fyy)优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> poses_fyy;
      */
      /*poses_fyy = Optimize_SE3FYY(initial_pose,edges_vo,"LM",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_fyy,"Our_LM.txt");//
      else
	produce_error(pose_gt,initial_pose,poses_fyy,"FYY_LM:");
      poses_fyy = Optimize_SE3FYY(initial_pose,edges_vo,"GN",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_fyy,"Our_GN.txt");//
      else
	produce_error(pose_gt,initial_pose,poses_fyy,"FYY_GN:");*/
      /*
      poses_fyy = Optimize_SE3FYY(initial_pose,edges_vo,"DL",50);
      if(filename_vo.find("freiburg")!=filename_vo.npos)
	write_TUM_file(argv[3],poses_fyy,"Our_DL.txt");//
      else
      {
	if(filename_vo.find("Pittsburgh")!=filename_vo.npos)//是匹兹堡数据集需要align数据
	{
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_fyy,pose_gt);//将真值的位移变换到vo坐标下
	  cout<<"优化之前位置绝对平均误差 = "<<t_Error(poses_vo_gtalign,poses_vo)/pose_gt.size()<<endl;
	  cout<<"优化之后位置绝对平均误差 = "<<t_Error(poses_fyy_gtalign,poses_fyy)/pose_gt.size()<<endl;
	}
	else//使用的是kitti数据集
	{
	  cout<<"没有进行对齐的结果 = "<<endl;
	  produce_error(pose_gt,poses_vo,poses_fyy,"FYY_DL:");
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_fyy,pose_gt);//将真值的位移变换到优化之后的坐标下
	  cout<<"对齐的结果 ="<<endl;
	  cout<<"VO的结果 = "<<endl;
	  produce_error(poses_vo_gtalign,poses_vo,poses_fyy,"FYY_DL:");
	  cout<<"优化之后的的结果 = "<<endl;
	  produce_error(poses_fyy_gtalign,poses_vo,poses_fyy,"FYY_DL:");
	}
      }*/
      
      //string path_fyy = "/home/fyy/Documents/my_code/posegraph/build/whole_result/fyy.g2o";
      //string path_fyy_error =  "/home/fyy/Documents/my_code/posegraph/build/whole_result/fyyError.txt";
      //write_G2O_file(path_fyy, edges_vo,poses_fyy);
      //write_error(path_fyy_error,error_fyy);
      
      
    
      /*
      //g2o自带的边 这个边有解析解的雅克比
      cout<<"使用G2O_SE3优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> optimized_pose_SE3G2O;
      optimized_pose_SE3G2O = Optimize_SE3G2O(poses_vo,edges_vo,50);//用这种方法迭代速度较慢 lm does not work well but gn works good
      ERRORS error_g2o = produce_error(pose_gt,poses_vo,optimized_pose_SE3G2O);
      //string path_SE3G2O = "/home/fyy/Documents/my_code/posegraph/build/whole_result/g2o.g2o";
      //string path_g2o_error = "/home/fyy/Documents/my_code/posegraph/build/whole_result/g2oError.txt";
      //write_G2O_file(path_SE3G2O,edges_vo,optimized_pose_SE3G2O);
      //write_error(path_g2o_error,error_g2o);
      */
      
      
      
      
      //使用isam进行优化
      cout<<"使用isam优化的结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> poses_isam;
      poses_isam = iSAM_margi(poses_vo,edges_vo,"DL");
      cout<<"ISAM-DL结果:"<<endl;
      if(filename_vo.find("Pittsburgh")!=filename_vo.npos)//是匹兹堡数据集需要align数据
      {   
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_isam,pose_gt);//将真值的位移变换到vo坐标下
	  cout<<"优化之前位置绝对平均误差 = "<<t_Error(poses_vo_gtalign,poses_vo)/pose_gt.size()<<endl;
	  cout<<"优化之后位置绝对平均误差 = "<<t_Error(poses_fyy_gtalign,poses_isam)/pose_gt.size()<<endl;
      }
      else
      {
	  cout<<"没有进行对齐的结果 = "<<endl;
	  produce_error(pose_gt,poses_vo,poses_isam,"ISAM_DL:");
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_isam,pose_gt);//将真值的位移变换到优化之后的坐标下
	  cout<<"对齐的结果 ="<<endl;
	  cout<<"VO的结果 = "<<endl;
	  produce_error(poses_vo_gtalign,poses_vo,poses_isam,"ISAM_DL:");
	  cout<<"优化之后的的结果 = "<<endl;
	  produce_error(poses_fyy_gtalign,poses_vo,poses_isam,"ISAM_DL:");
      }
      //produce_error(pose_gt,poses_vo,poses_isam,"ISAM DL误差结果 137-4459 = ",137,4459);
      //produce_error(pose_gt,poses_vo,poses_isam,"ISAM DL误差结果 10-4459 = ",10,4459);
      poses_isam = iSAM_margi(poses_vo,edges_vo,"GN");
      cout<<"ISAM-GN结果:"<<endl;
      if(filename_vo.find("Pittsburgh")!=filename_vo.npos)//是匹兹堡数据集需要align数据
      {   
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_isam,pose_gt);//将真值的位移变换到vo坐标下
	  cout<<"优化之前位置绝对平均误差 = "<<t_Error(poses_vo_gtalign,poses_vo)/pose_gt.size()<<endl;
	  cout<<"优化之后位置绝对平均误差 = "<<t_Error(poses_fyy_gtalign,poses_isam)/pose_gt.size()<<endl;
      }
      else
      {
	  cout<<"没有进行对齐的结果 = "<<endl;
	  produce_error(pose_gt,poses_vo,poses_isam,"FYY_GN:");
	  vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	  vector<pose> poses_fyy_gtalign = AlignPose(poses_isam,pose_gt);//将真值的位移变换到优化之后的坐标下
	  cout<<"对齐的结果 ="<<endl;
	  cout<<"VO的结果 = "<<endl;
	  produce_error(poses_vo_gtalign,poses_vo,poses_isam,"FYY_GN:");
	  cout<<"优化之后的的结果 = "<<endl;
	  produce_error(poses_fyy_gtalign,poses_vo,poses_isam,"FYY_GN:");
      }
     
      //produce_error(pose_gt,poses_vo,poses_isam,"ISAM GN误差结果 137-4459 = ",137,4459);
      //produce_error(pose_gt,poses_vo,poses_isam,"ISAM GN误差结果 10-4459 = ",10,4459);
      
      //string path_isam = "/home/fyy/Documents/my_code/posegraph/build/whole_result/isam.g2o";
      //string path_isam_error = "/home/fyy/Documents/my_code/posegraph/build/whole_result/isamError.txt";
      //write_G2O_file(path_isam, edges_vo,poses_isam);
      //write_error(path_isam_error,error_isam);
      
      

      //然后识别出哪些是回环边哪些是pose-chain边
      vector<edge> closeedges;
      for(int i=0;i<edges_vo.size();i++)
      {
	if(abs(edges_vo[i].id1-edges_vo[i].id2)!=1)//是回环边
	    closeedges.push_back(edges_vo[i]);
      }
      int numofCloseLoop = closeedges.size();
      //对回环进行重新排序 按照id2的大小升序
      sort(closeedges.begin(),closeedges.end(),comp);
      
      /*
      //这里是计算重定位误差-为54所项目进行验证
      for(int i=0;i<closeedges.size();i++)
      {
	int id1 = closeedges[i].id1;
	int id2 = closeedges[i].id2;
	//提取出真实位姿在id1和id2的位姿
	Eigen::Vector3d t_id1_gt = pose_gt[id1].t;
	Eigen::Quaterniond q_id1_gt = pose_gt[id1].q;
	Eigen::Vector3d t_id2_gt = pose_gt[id2].t;
	Eigen::Quaterniond q_id2_gt = pose_gt[id2].q;
        //提取出vo在id1的位姿
	Eigen::Vector3d t_id1_vo = pose_gt[id1].t;
	Eigen::Quaterniond q_id1_vo = pose_gt[id1].q;
	//提取出边的测量值
	Eigen::Vector3d delta_t = closeedges[i].delta_t;
	Eigen::Quaterniond delta_q = closeedges[i].delta_q;
	//计算真实位姿的差值T1^-1*T2
	Eigen::Quaterniond delta_q_gt; 
	delta_q_gt = q_id1_gt.toRotationMatrix().transpose()*q_id2_gt.toRotationMatrix();
	Eigen::Vector3d delta_t_gt = q_id1_gt.toRotationMatrix().transpose()*(t_id2_gt-t_id1_gt);
	double errorR = ErrorR_KITTI(delta_q_gt,delta_q);
	double errort = (delta_t_gt-delta_t).norm();
	cout<<"第"<<i<<"个回环边重定位相对误差 = "<<errorR<<" "<<errort<<endl;
	Eigen::Quaterniond q_id2_reckon;
	q_id2_reckon = q_id1_gt.toRotationMatrix()*delta_q.toRotationMatrix();
	Eigen::Vector3d t_id2_reckon = q_id1_gt.toRotationMatrix()*delta_t+t_id1_gt;
	errorR = ErrorR_KITTI(q_id2_reckon,q_id2_gt);
	errort =(t_id2_reckon-t_id2_gt).norm();
	cout<<"第"<<i<<"个回环边重定位绝对误差 = "<<errorR<<" "<<errort<<endl;
      }*/
      
      
      /*
       //模仿真实采集数据时的效果碰到一个回环然后进行优化，碰到一个回环再进行优化，不考虑边缘化。有点类似于ORB-slam代码中对回环的处理
      cout<<"顺序优化结果-----------------------------------------------------------------------------"<<endl; 
      vector<pose> optimized_seq_poses;//存储的是顺序优化的结果
      for(int i=0;i<numofCloseLoop;i++)
      {
	  int id1 = closeedges[i].id1;
	  int id2 = closeedges[i].id2;
	  cout<<"回环段 = " <<id1<<" "<<id2<<endl;
	  if(i==0)
	  {
	    id1 = 0;
	  }
	  if(i==numofCloseLoop-1)
	  {
	    id2 = poses_vo.size()-1;
	  }
	  
	  vector<pose> unoptimized_poses;//待优化的姿态
	  int last_id = 0;
	  if(optimized_seq_poses.size()!=0)//表示已经初始化完成了
	  {
	    last_id = optimized_seq_poses.size()-1;
	  
	  }
	  //更新要估计的绝对位姿态：如果这个回环段中存在之前的优化的姿态则直接使用之前优化的姿态，对于那些不在优化姿态中的则计算与上一次优化最后一个姿态的差值，用这个差值来更新vo的姿态
	  for(int j=id1;j<=id2;j++)
	  {
	    if((j<=last_id)&&(last_id!=0))//使用之前优化过的位姿来进行优化
	    {
	      pose unoptimized_pose;
	      unoptimized_pose.id =j -id1;
	      unoptimized_pose.t = optimized_seq_poses[j].t;
	      unoptimized_pose.q = optimized_seq_poses[j].q;
	      unoptimized_poses.push_back(unoptimized_pose);
	    }
	    else//这个是新的位姿没有优化过则
	    {
	      pose delta_pose;
	      delta_pose = poses_vo[last_id].inverse()*poses_vo[j];
	      pose posej;
	      if(optimized_seq_poses.size()!=0)
	      {
		posej=optimized_seq_poses[last_id]*delta_pose;
	      }
	      else
	      {
		posej = delta_pose;
	      }
	      posej.id =j-id1;
	      unoptimized_poses.push_back(posej);
	    }
	  }
	  vector<edge> unoptimized_edges;//待优化的bian
	  //分为和之前优化重叠的边 和不重叠的边
	  for(int j=id1;j<id2;j++)
	  {
	      edge edge_j;
	      edge_j.id1 = j-id1;
	      edge_j.id2 = j+1-id1;
	      pose delta_pose;
	     if(j<last_id)//重叠的边
	     {
		delta_pose = optimized_seq_poses[j].inverse()*optimized_seq_poses[j+1];//计算得到当前帧与上一次优化最后一个帧的姿态变化
	     }
	     else//没有重叠的边 使用vo的测量数据
	     {
	       delta_pose = poses_vo[j].inverse()*poses_vo[j+1];//计算得到当前帧与上一次优化最后一个帧的姿态变化
	     }
	     edge_j.delta_q = delta_pose.q;
	     edge_j.delta_t = delta_pose.t;
	     unoptimized_edges.push_back(edge_j);
	  }
	  //最后将回环边插入到待优化的边中
	  edge closeedge = closeedges[i];
	  closeedge.id1 = closeedge.id1-id1;
	  closeedge.id2 = closeedge.id2-id1;
	  unoptimized_edges.push_back(closeedge);
	  
	  //然后开始进行优化
	  vector<pose> optimized_subset_poses;//待优化位姿
	  optimized_subset_poses = Optimize_SE3Lie(unoptimized_poses,unoptimized_edges);//！！！！！！！！！！！！！！！！！！！！！！！！
	  //optimized_subset_poses = iSAM_full(unoptimized_poses,unoptimized_edges,20);
	  //optimized_subset_poses = Optimize_SE3FYY(unoptimized_poses,unoptimized_edges,20);
	  //optimized_subset_poses = Optimize_SE3G2O(unoptimized_poses,unoptimized_edges,20);
	  //更新我们优化的变量
	  for(int j=0;j<optimized_subset_poses.size();j++)
	  {
	    if((j+id1<=last_id)&&(last_id!=0))//update
	    {
	      optimized_seq_poses[j+id1].t = optimized_subset_poses[j].t;
	      optimized_seq_poses[j+id1].q = optimized_subset_poses[j].q;
	    }
	    else//join
	    {
	      pose new_pose;
	      new_pose.id = j+id1;
	      new_pose.t = optimized_subset_poses[j].t;
	      new_pose.q = optimized_subset_poses[j].q;
	      optimized_seq_poses.push_back(new_pose);
	    }
	  }
	  //更新到下个回环开始的姿态
	  if(i<numofCloseLoop-1)//对最后一个回环段不进行更新后续的姿态
	  {
	    for(int j=id2+1;j<=closeedges[i+1].id1;j++)
	    {
	      Eigen::Vector3d delta_t;
	      Eigen::Quaterniond delta_q;
	      pose delta_pose;
	      delta_pose = poses_vo[id2].inverse()*poses_vo[j];//计算得到当前帧与上一次优化最后一个帧的姿态变化
	      pose posei;
	      posei = optimized_seq_poses[id2]*delta_pose;
	      posei.id = j;
	      optimized_seq_poses.push_back(posei);
	    }
	  }
	  cout<<"已经优化了的位姿个数 = "<<optimized_seq_poses.size()<<endl;
      }//遍历不同的回环段结束
      produce_error(pose_gt,poses_vo,optimized_seq_poses,"错误的顺序PGO = ",10,4459);
      */
      
      
      //test sequence
      //iSAM_full,Optimize_SE3FYY,Optimize_SE3ORB,Optimize_SE3G2O,Optimize_SE3Lie,Ceres_posegraph,Optimize_SE3Partition
      string method_names[7] = {"iSAM_full","FYY","ORB","G2O","SLAM14","CERES","Partition"};
      Create_Folders("TRUE",false);
      Create_Folders("VO",false);
      cout<<"生成TRUE和VO文件夹"<<endl;
      
      cout<<"分段优化结果-----------------------------------------------------------------------------"<<endl; 
      vector<map<int,edge>> allof_deltaposes;//存储的是所有帧与帧之间的相对变化
      map<int,double> subseterorrs,constrainterrors;
      int initial_l = 1;
      for(int l=initial_l;l<7;l++)//遍历不同的方法
      {
	   Create_Folders(method_names[l]);
	   cout<<"生成"<<method_names[l]<<"文件夹"<<endl;
	   string methodname = method_names[l];
	   vector<ERRORS> errors;//用于保存所有回环段的误差结果
	  //对回环逐个的进行优化 并将优化的结果和真值写入到subset_result结果中
	  for(int i=0;i<numofCloseLoop;i++)
	  {
		int id1 = closeedges[i].id1;
		int id2 = closeedges[i].id2;
		cout<<"回环段 = "<<id1<<" "<<id2<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
		//加入下面这两个条件的原因是保证所有的点都进行了优化，其中主要包括的是起始的点和结束的点
		if(i==0)
		{
		  id1 =0;
		}
		if(i==numofCloseLoop-1)
		{
		  id2 = poses_vo.size()-1;
		}
		
		vector<pose> subset_vo_poses;//将某段的回环的相对姿态保存起来 根据vo得到的
		vector<pose> subset_gt_poses;//将某段的回环的相对姿态保存起来 根据真值得到的
		vector<edge> subset_edges;//将某段的回环测量值保存起来
		Eigen::Matrix<double,6,6> informatrix;
		informatrix = Eigen::Matrix<double,6,6>::Identity();//权重等于1
		
		//得到某段回环的初始帧的位姿
		pose origin;
		origin.id = 0;
		origin.t = Eigen::Vector3d(0,0,0);
		origin.q = Eigen::Matrix3d::Identity();
		subset_vo_poses.push_back(origin);//这个subset中vo的位姿 都是Tstart.inverse*Ti,Tstart是起始端的位姿
		subset_gt_poses.push_back(origin);//这个subset中真值的位姿 都是Tstart.inverse*Ti,Tstart是起始端的位姿
		
		Eigen::Vector3d t_vo_ori = poses_vo[id1].t;//闭环起始端的位姿，vo得到的
		Eigen::Matrix3d R_vo_ori = poses_vo[id1].q.toRotationMatrix();
		
		Eigen::Vector3d t_gt_ori = pose_gt[id1].t;//闭环起始端的位姿，真值得到的
		Eigen::Matrix3d R_gt_ori = pose_gt[id1].q.toRotationMatrix();
		
		
		{
		  Sophus::SE3 delta_se3_vo,delta_se3_loop;
		  //先找到那个回环边的测量值
		  for(int k=0;k<edges_vo.size();k++)
		  {
		    if( (edges_vo[k].id1==closeedges[i].id1)&&(edges_vo[k].id2==closeedges[i].id2) )
		    {
		      delta_se3_loop = Sophus::SE3(edges_vo[k].delta_q,edges_vo[k].delta_t);
		      break;
		    }
		  }
		  Sophus::SE3 se3_id1(poses_vo[id1].q,poses_vo[id1].t);
		  Sophus::SE3 se3_id2(poses_vo[id2].q,poses_vo[id2].t);
		  Sophus::SE3 se3_constraint = delta_se3_loop.inverse()*se3_id1.inverse()*se3_id2;
		  Eigen::Vector3d error_fi_constraint = se3_constraint.so3().log();
		  Eigen::Vector3d error_t_constraint = se3_constraint.translation();
		  Eigen::Matrix<double,6,1> error;
		  error<<error_fi_constraint,error_t_constraint;
		  constrainterrors.insert( pair<int,double>(i,error.norm()/(id1-id2)) );
		}
		//更新某回环段的vo姿态subset_vo_poses和真值姿态subset_gt_poses
		for(int j=id1+1;j<id2+1;j++)//遍历范围是从id1+1到id2
		{
		    //更新某段回环的vo位姿 =Tstart.inverse*Ti
		    Eigen::Matrix3d R_vo_i = R_vo_ori.transpose()*poses_vo[j].q.toRotationMatrix();
		    Eigen::Vector3d t_vo_t = R_vo_ori.transpose()*(poses_vo[j].t-t_vo_ori);
		    pose vo_pose;//更新之后的subset中的vo位姿
		    vo_pose.q = R_vo_i;
		    vo_pose.t = t_vo_t;
		    vo_pose.id = poses_vo[j].id-id1;
		   
		    subset_vo_poses.push_back(vo_pose);
		    
		    //更新某段回环的位姿真值
		    Eigen::Matrix3d R_gt_i = R_gt_ori.transpose()*pose_gt[j].q.toRotationMatrix();
		    Eigen::Vector3d t_gt_t = R_gt_ori.transpose()*(pose_gt[j].t-t_gt_ori);
		    pose gt_pose;
		    gt_pose.q = R_gt_i;
		    gt_pose.t = t_gt_t;
		    gt_pose.id = pose_gt[j].id-id1;
		    subset_gt_poses.push_back(gt_pose);
		}
		
		
		//遍历所有的边 ,将某段回环边保存到subset_edges
		for(int j=0;j<edges_vo.size();j++)
		{
		  if( (edges_vo[j].id1<=id2)&&(edges_vo[j].id1>=id1) &&  (edges_vo[j].id2<=id2)&&(edges_vo[j].id2>=id1) )
		  {
		    //if( (edges_vo[j].id2-edges_vo[j].id1==1) || ((edges_vo[j].id1==closeedges[i].id1)&&(edges_vo[j].id2==closeedges[i].id2)) )//表示我们不考虑这个subset中的其他回环边
		    //{
			edge sub_edge;
			sub_edge.id1 = edges_vo[j].id1-id1;
			sub_edge.id2 = edges_vo[j].id2-id1;
			sub_edge.delta_t = edges_vo[j].delta_t;
			sub_edge.delta_q = edges_vo[j].delta_q;
			subset_edges.push_back(sub_edge);
		    //}
		  }
		}
		
	       //进行优化 这里我们可采取不同的优化策略 
	      //iSAM_full,Optimize_SE3FYY,Optimize_SE3ORB,Optimize_SE3G2O,Optimize_SE3Lie,Ceres_posegraph
	      vector<pose> subset_opt_poses;//将某段的回环的相对姿态保存起来 优化得到的
	      switch (l)
	      {
		case 0:
		   subset_opt_poses = iSAM_full(subset_vo_poses,subset_edges);//iSAM_full
		  break;
		case 1:
		   subset_opt_poses = Optimize_SE3FYY(subset_vo_poses,subset_edges);//Optimize_SE3FYY
		   break; 
		case 2:
		   subset_opt_poses = Optimize_SE3ORB(subset_vo_poses,subset_edges);//Optimize_SE3ORB
		   break; 
		case 3:
		   subset_opt_poses = Optimize_SE3G2O(subset_vo_poses,subset_edges);//Optimize_SE3G2O
		   break; 
		case 4:
		   subset_opt_poses = Optimize_SE3Lie(subset_vo_poses,subset_edges);//Optimize_SE3Lie
		   break; 
		case 5:
		  subset_opt_poses = Ceres_posegraph(subset_vo_poses,subset_edges);//Ceres_posegraph
		  break; 
		case 6:
		  subset_opt_poses = Optimize_SE3Partition(subset_vo_poses,subset_edges);//Optimize_SE3Partition
		  break;
	      }
	      //下面计算优化之后的残差
	      double subset_residual_error = EdgeSE3_findt::Produce_Error(subset_opt_poses,subset_edges);
	      double subset_avg_residual_error = sqrt(subset_residual_error)/subset_edges.size();
	      cout<<"回环的边数 = "<<subset_edges.size()<<endl;
	      cout<<"优化之后的残差平均值 = "<<subset_avg_residual_error<<endl;
	      subseterorrs.insert(pair<int,double>(i,subset_avg_residual_error));
	      //根据残差更新权重矩阵
	      informatrix = informatrix*(subset_residual_error/subset_edges.size());
	      informatrix = informatrix*informatrix*informatrix;
	      //informatrix = informatrix*(subset_edges.size()/subset_residual_error);//残差越小则权重越大
	      //计算和真值的误差
	      ERRORS error;
	      error = produce_error(subset_gt_poses,subset_vo_poses,subset_opt_poses);
	      errors.push_back(error);
	      //将某个回环段的优化结果和误差保存起来
	      string path_optimized = "/home/fyy/Documents/my_code/posegraph/build/subset_result/"+methodname+"/Pose/opti_";
	      string path_error = "/home/fyy/Documents/my_code/posegraph/build/subset_result/"+methodname+"/Error/error_";
	      path_optimized = path_optimized+to_string(id1)+"_"+to_string(id2)+".g2o";
	      path_error = path_error+to_string(id1)+"_"+to_string(id2)+".txt";
	      write_G2O_file(path_optimized,subset_edges,subset_opt_poses);
	      write_error(path_error,error);
	      
	      if(l==initial_l)
	      {
		//把真值的结果保存在txt文件中 
		string path_true = "/home/fyy/Documents/my_code/posegraph/build/subset_result/TRUE/true_subset_";
		string path_true_subset = path_true+to_string(id1)+"_"+to_string(id2)+".g2o";
		write_G2O_file(path_true_subset,subset_edges,subset_gt_poses);
		
		//将vo的值保存在保存在txt文件中
		string path_vo = "/home/fyy/Documents/my_code/posegraph/build/subset_result/VO/vo_subset_";
		string path_vo_subset = path_vo+to_string(id1)+"_"+to_string(id2)+".g2o";
		write_G2O_file(path_vo_subset,subset_edges,subset_vo_poses);
	      }
	      
	      
	      //下面开始保存这个subset优化之后帧与帧之间的相对位姿
	      map<int,edge> subset_delta;//优化完之后计算位姿的差值
	      for(int j=0;j<subset_opt_poses.size()-1;j++)
	      {
		Sophus::SE3 se3_first(subset_opt_poses[j].q,subset_opt_poses[j].t);
		Sophus::SE3 se3_second(subset_opt_poses[j+1].q,subset_opt_poses[j+1].t);
		Sophus::SE3 se3_delta = se3_first.inverse()*se3_second;
		edge edge_delta;
		edge_delta.delta_q = se3_delta.unit_quaternion();
		edge_delta.delta_t = se3_delta.translation();
		edge_delta.id1 = j+id1;
		edge_delta.id2 = j+id1+1;
		edge_delta.cov = informatrix;//更新信息矩阵
		edge_delta.avg_error = subset_avg_residual_error;
		edge_delta.indexofloop = i;//属于哪个回环段优化得到的
		subset_delta.insert(pair<int,edge>(j+id1,edge_delta));
	      }
	      allof_deltaposes.push_back(subset_delta);  	      
	      cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
	  }//结束逐个优化闭环段
	  cout<<"完成每段闭环的优化-----------------------------------------------------------------------------------------------"<<endl;
	 
	  //便利每个输出的结果
	  cout<<"优化之后平均残差 = "<<endl;
	  map<int,double>::iterator iter;
	  for(iter = subseterorrs.begin();iter!=subseterorrs.end();iter++)
	  {
	    cout<<closeedges[iter->first].id1<<" "<<closeedges[iter->first].id2<<" "<<iter->second<<endl;
	  }
	  cout<<"constraint 平均误差 = "<<endl;
	  for(iter = constrainterrors.begin();iter!=constrainterrors.end();iter++)
	  {
	    cout<<closeedges[iter->first].id1<<" "<<closeedges[iter->first].id2<<" "<<iter->second<<endl;
	  }
	   //一次性输出所有的闭环段的误差数据 并计算每个闭环段的平均误差
	  double unopt_pose_error=0;//
	  double unopt_t_error=0;
	  double opt_pose_error=0;
	  double opt_t_error=0;
	  double unopt_pose_avgerror=0;
	  double unopt_t_avgerror=0;
	  double opt_pose_avgerror=0;
	  double opt_t_avgerror=0;
	  for(int i=0;i<numofCloseLoop;i++)
	  {
	    cout<<"闭环段 = "<<closeedges[i].id1<<" "<<closeedges[i].id2<<endl;
	    int numofpose = closeedges[i].id2-closeedges[i].id1+1;
	    cout<<"优化之前姿态误差 = "<<errors[i].i_sum_R/numofpose<<endl;  
	    unopt_pose_error = unopt_pose_error+errors[i].i_sum_R/numofpose;
	    cout<<"优化之后姿态误差 = "<<errors[i].opt_sum_R/numofpose<<endl;
	    opt_pose_error = opt_pose_error+errors[i].opt_sum_R/numofpose;
	    cout<<"优化之前位置误差 = "<<errors[i].i_sum_t/numofpose<<endl;  
	    unopt_t_error = unopt_t_error+errors[i].i_sum_t/numofpose;
	    cout<<"优化之后位置误差 = "<<errors[i].opt_sum_t/numofpose<<endl;
	    opt_t_error = opt_t_error+ errors[i].opt_sum_t/numofpose;
	    cout<<"优化之前的平均误差 ： "<<"位置平均误差 = "<<errors[i].i_avg_t<<" 姿态平均误差 = "<<errors[i].i_avg_R<<endl;
	    cout<<"优化之后的平均误差 ： "<<"位置平均误差 = "<<errors[i].opt_avg_t<<" 姿态平均误差 = "<<errors[i].opt_avg_R<<endl;
	    cout<<"---------------------------------------------------------------------------------"<<endl;
	    unopt_t_avgerror =unopt_t_avgerror+ errors[i].i_avg_t;
	    unopt_pose_avgerror=unopt_pose_avgerror+errors[i].i_avg_R;
	    opt_t_avgerror=opt_t_avgerror+errors[i].opt_avg_t;
	    opt_pose_avgerror=opt_pose_avgerror+errors[i].opt_avg_R;
	  }
	  cout<<"总误差 = "<<endl;
	  cout<<"优化之前姿态误差 = "<<unopt_pose_error<<endl;  
	  cout<<"优化之后姿态误差 = "<<opt_pose_error<<endl;
	  cout<<"优化之前位置误差 = "<<unopt_t_error<<endl;  
	  cout<<"优化之后位置误差 = "<<opt_t_error<<endl;
	  cout<<"优化之前的平均误差 ： "<<"位置平均误差 = "<<unopt_t_avgerror<<" 姿态平均误差 = "<<unopt_pose_avgerror<<endl;
	  cout<<"优化之后的平均误差 ： "<<"位置平均误差 = "<<opt_t_avgerror<<" 姿态平均误差 = "<<opt_pose_avgerror<<endl;
	  string path_avg_subset = "/home/fyy/Documents/my_code/posegraph/build/subset_result/"+methodname+"/error_avg.txt";
	  ERRORS erro_subset_avg;
	  erro_subset_avg.i_sum_R = unopt_pose_error;
	  erro_subset_avg.i_sum_t = unopt_t_error;
	  erro_subset_avg.i_avg_R = unopt_pose_avgerror;
	  erro_subset_avg.i_avg_t = unopt_t_avgerror;
	  erro_subset_avg.opt_sum_R = opt_pose_error;
	  erro_subset_avg.opt_sum_t = opt_t_error;
	  erro_subset_avg.opt_avg_R = opt_pose_avgerror;
	  erro_subset_avg.opt_avg_t = opt_t_avgerror;
	  write_error(path_avg_subset,erro_subset_avg);//将平均的subset结果保存
	  
	  //已经遍历完所有的闭环开始进行优化	  
	  //遍历所有的vo边=顶点数-1
	  vector<edge> opt_deltapose;//经过优化之后的所有相对位姿
	  for(int i=0;i<poses_vo.size()-1;i++)
	  {
	      //1.首先得到初值
	      edge initial_delta;	      
	      for(int j=0;j<edges_vo.size();j++)
	      {
		if((edges_vo[j].id1==i)&&(edges_vo[j].id2==i+1))
		{
		  initial_delta  = edges_vo[j];//使用的是vo的相对位姿作为更新
		  break;
		}
	      }
	      //2.将之前得到的和i至i+1边所有相对位姿的值都压入到edgei
	      vector<edge> edgei;//第i到i+1边的所有测量值
	      for(int j=0;j<allof_deltaposes.size();j++)//遍历所有的subset
	      {
		map<int,edge>::iterator iter;
		iter = allof_deltaposes[j].find(i);
		if(iter!=allof_deltaposes[j].end())//找到了
		{
		  edgei.push_back(iter->second);
		}
	      }
	      //3.进行优化
	      if(edgei.size()>1)
	      {
		//for(int j=0;j<edgei.size();j++)
		//{
		  cout<<"optimized edge = "<<initial_delta.id1<<" "<<initial_delta.id2<<endl;
		  //然后归一化系数
		  double sum_k=0;
		  for(int j=0;j<edgei.size();j++)
		  {
		    sum_k = sum_k + edgei[j].cov(0,0);
		  }
		   for(int j=0;j<edgei.size();j++)
		  {
		    edgei[j].cov = edgei[j].cov/sum_k;
		  }
		  //我们将Vo的数据也加入到优化中
		  //initial_delta.cov = Eigen::Matrix<double,6,6>::Identity()/edgei.size();
		  //edgei.push_back(initial_delta);		
		  edge optimized_edge = optimize_Unary_Lie(edgei,initial_delta,20);
		  //edge optimized_edge =  optimize_Unary_fyy(edgei,initial_delta,20);//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  //Eigen::Vector3d t_ori,t_opt;
		  //Eigen::Quaterniond q_ori,q_opt;
		  //t_ori = initial_delta.delta_t; q_ori = initial_delta.delta_q;
		  //t_opt = optimized_edge.delta_t; q_opt = optimized_edge.delta_q;
		  //cout<<"初始值 = "<<t_ori(0)<<" "<<t_ori(1)<<" "<<t_ori(2)<<q_ori.w()<<" "<<q_ori.x()<<" "<<q_ori.y()<<" "<<q_ori.z()<<endl;
		  //cout<<"优化后的 = "<<t_opt(0)<<" "<<t_opt(1)<<" "<<t_opt(2)<<q_opt.w()<<" "<<q_opt.x()<<" "<<q_opt.y()<<" "<<q_opt.z()<<endl;
		  opt_deltapose.push_back(optimized_edge);
		//}
	      }
	      else if(edgei.size()==1)
	      {
		cout<<"one pose edge = "<<initial_delta.id1<<" "<<initial_delta.id2<<endl;
		opt_deltapose.push_back(edgei[0]);
	      }
	  }
	  cout<<"完成G-PGO优化-----------------------------------------------------------------------------------------------"<<endl;
	  
	  
	  //根据得到的相对位姿变化 更新绝对位姿
	  Sophus::SE3 se3_accpose(poses_vo[0].q,poses_vo[0].t);//这个是根据相对位姿变化得到的累积结果
	  vector<pose> fyy_poses;//optimized pose
	  fyy_poses.push_back(poses_vo[0]);
	  for(int i=1;i<poses_vo.size();i++)
	  {
	      //1. 首先在opt_deltapose结构中找到i-1至i的边
	      bool find=false;
	      for(int j=0;j<opt_deltapose.size();j++)
	      {
		if((opt_deltapose[j].id1==i-1)&&(opt_deltapose[j].id2==i))
		{
		  Sophus::SE3 se3_delta(opt_deltapose[j].delta_q,opt_deltapose[j].delta_t);
		  se3_accpose = se3_accpose * se3_delta;
		  find = true;
		  break;
		}
	      }
	      //2.更新i的位姿
	      if(find==true)
	      {
		  pose posei;
		  posei.q = se3_accpose.unit_quaternion();
		  posei.t = se3_accpose.translation();
		  posei.id = i;
		  fyy_poses.push_back(posei);
	      }
	  }//绝对位姿更新完毕
	  ERRORS error;
	  //error = produce_error(pose_gt,poses_vo,fyy_poses,"G-PGO 137-4459：",137,4459);
	  //error = produce_error(pose_gt,poses_vo,fyy_poses,"G-PGO 10-4459：",10,4459);
	   cout<<"G-PGO结果:"<<endl;
	  if(filename_vo.find("Pittsburgh")!=filename_vo.npos)//是匹兹堡数据集需要align数据
	  {   
	      vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	      vector<pose> poses_fyy_gtalign = AlignPose(fyy_poses,pose_gt);//将真值的位移变换到vo坐标下
	      cout<<"优化之前位置绝对平均误差 = "<<t_Error(poses_vo_gtalign,poses_vo)/pose_gt.size()<<endl;
	      cout<<"优化之后位置绝对平均误差 = "<<t_Error(poses_fyy_gtalign,fyy_poses)/pose_gt.size()<<endl;
	      string path_gpgo = "/home/fyy/Documents/my_code/posegraph/data/G-PGO.g2o";
	      string path_gpgo_gt = "/home/fyy/Documents/my_code/posegraph/data/G-PGO-GT.g2o";
	      vector<edge> empty_edge;
	      write_G2O_file(path_gpgo,empty_edge,fyy_poses);
	      write_G2O_file(path_gpgo_gt,empty_edge,poses_fyy_gtalign);
	  }
	  else
	  {
	    cout<<"没有进行对齐的结果 = "<<endl;
	    produce_error(pose_gt,poses_vo,fyy_poses,"FYY_DL:");
	    vector<pose> poses_vo_gtalign = AlignPose(poses_vo,pose_gt);//将真值的位移变换到vo坐标下
	    vector<pose> poses_fyy_gtalign = AlignPose(fyy_poses,pose_gt);//将真值的位移变换到优化之后的坐标下
	    cout<<"对齐的结果 ="<<endl;
	    cout<<"VO的结果 = "<<endl;
	    produce_error(poses_vo_gtalign,poses_vo,fyy_poses,"FYY_DL:");
	    cout<<"优化之后的的结果 = "<<endl;
	    produce_error(poses_fyy_gtalign,poses_vo,fyy_poses,"FYY_DL:");
	  }
	  cout<<"finish"<<endl;
	  return 1;
	  //string path_fyyoptimized = "/home/fyy/Documents/my_code/posegraph/build/whole_result/opti_"+methodname+"_G-PGO.g2o";
	  //string path_fyyerror =     "/home/fyy/Documents/my_code/posegraph/build/whole_result/error_" + methodname + "_G-PGO.txt";
	  //write_G2O_file(path_fyyoptimized,edges_vo,fyy_poses);
	  //write_error(path_fyyerror,error);
       }//每个方法都遍历结束了
     
      
      
      
  }//读取数据没有问题的大if括号

}