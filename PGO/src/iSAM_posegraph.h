#ifndef _iSAM_
#define _iSAM_
//pose graph代码
#include <gtsam/slam/dataset.h>//用于读取g2o文件
#include <gtsam/slam/BetweenFactor.h>//pose graph的边
#include <gtsam/slam/PriorFactor.h>//先验因子 基本上所有程序都需要
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>//高斯牛顿非线性优化
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>//lm非线性优化
#include <gtsam/nonlinear/DoglegOptimizer.h>//dogleg
#include <gtsam/inference/Symbol.h>//gtsam中顶点序号的类型
#include <gtsam/nonlinear/ISAM2.h>//主要用到的isam2库
#include <gtsam/nonlinear/NonlinearFactorGraph.h>//gtsam中的图类
#include <gtsam/nonlinear/Values.h>//因子图中使用到的值
#include <gtsam/geometry/Pose3.h>//顶点类型
#include <gtsam/linear/NoiseModel.h>//包含噪声模型

#include <fstream>
#include <string.h>

#include <sophus/se3.h>
#include <sophus/so3.h>//在使用gtsam提供的雅克比时用到

#include "types.h"
#include "FileIO.h"
#include "G2OEdge.h"//为了使用里面的Hat函数


using namespace std;
using namespace gtsam;
//这个文件的作用是使用gtsam的isam模块和非线性模块完成位姿图优化
namespace posegraph
{ 
  //将gtsam库中的 Poe3数据类型转换成Eigen类型
  void Pose_gtsamToEigen(Pose3 pose,Eigen::Vector3d &t,Eigen::Quaterniond &q)
  {
    Point3 tpose = pose.translation();
    Rot3 Rpose = pose.rotation();
    t= Eigen::Vector3d(tpose.x(),tpose.y(),tpose.z());
    q = Eigen::Quaterniond(Rpose.toQuaternion().w(),Rpose.toQuaternion().x(),Rpose.toQuaternion().y(),Rpose.toQuaternion().z());
  }
  //将eigen数据类型 转换成gtsam类型的Pose3
  void Pose_EigenTogtsam(Eigen::Vector3d t,Eigen::Quaterniond q, Pose3 &pose)
  {
    Point3 tdelta= Point3(t(0),t(1),t(2));
    Rot3 Rdelta = Rot3::quaternion(q.w(),q.x(),q.y(),q.z());
    pose = Pose3(Rdelta,tdelta);
  }
  //我们gtsam得到的优化之后的数据类型Value转换成vector<pose>
  vector<pose> Value_gtsamToEigen(const Values& estimate)
  {
    vector<pose> poses;
    for(const auto& key_value:estimate)
    {
      auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
      if(!p) continue;
      const Pose3& pose_gtsam = p->value();
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      Pose_gtsamToEigen(pose_gtsam,t,q);
      int id = key_value.key;
      pose posei;
      posei.id = id;
      posei.t = t;
      posei.q = q;
      poses.push_back(posei);
    }
    return poses;
  }
  //定义回环边的排序顺序 升序
  bool comp(const edge &e1,const edge &e2)
  {
    return e1.id2<e2.id2;
  }
  
  //输入的是初始的位姿 和里程计的输入信息
  vector<pose> iSAM_margi(vector<pose> poses, vector<edge> edges_vo,string method="DL")
  {
    cout<<"开始iSAM_margi优化------------------------------------------------------------------"<<endl;
    //1.首先对ISam2这个类进行初始化
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    if(method =="DL")
    {
      parameters.optimizationParams = ISAM2DoglegParams();//ISAM2GaussNewtonParams ISAM2DoglegParams
    }
    else if(method=="GN")
    {
      parameters.optimizationParams = ISAM2GaussNewtonParams();//ISAM2GaussNewtonParams ISAM2DoglegParams
    }
    ISAM2 isam(parameters);
    
    //2.构造因子图的类型
    NonlinearFactorGraph graph;
    Values initialEstimate;
    
    //3.初始化先验因子协方差矩阵
    Pose3 pose0;
    Pose_EigenTogtsam(poses[0].t,poses[0].q,pose0);
    noiseModel::Diagonal::shared_ptr priorModel =  noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());//先是位置后是姿态
    //noiseMoel::Isotropic::shared_ptr priormodel = noiseModel::Isotropic::Sigma(6,1.0);
    //gtsam::Matrix mprior(6,6);
    //mprior = eye(6);// gtsam::Matri等于Eigen::MatrixXd
    //noiseModel::Base::shared_ptr priormodel =noiseModel::Gaussian::Information(mprior);
    graph.add(PriorFactor<Pose3>(0, pose0, priorModel));//向图中加入先验
    //graph.push_back(boost::make_shared<PriorFactor<Pose3>>(0, Pose3(R0,t0), poseNoise));//iSAM中使用的加入先验因子的方法
    //initialEstimate.insert(0, pose0);//输入的第一个参数也可以是Symbol('x', 0)
    
    //4.对回环结果进行排序
    vector<edge> closeedges;
    for(int i=0;i<edges_vo.size();i++)
    {
	if(abs(edges_vo[i].id2-edges_vo[i].id1)!=1)//是回环边
	  closeedges.push_back(edges_vo[i]);
    }
    int numofCloseLoop = closeedges.size();
    //对回环进行重新排序 按照id2的大小升序
    sort(closeedges.begin(),closeedges.end(),comp);
    for(int i=0;i<closeedges.size();i++)
    {
      cout<<"排序之后的回环边 = "<<closeedges[i].id1<<" "<<closeedges[i].id2<<endl;
    }
    //5.对回环逐个的进行优化
    vector<pose> optimized_poses;//这里存储的是优化之后的结果
    Pose3 lastPose;
    int lastid2=0;
    int id1,id2;//存储的是每个回环边的两个id
    for(int i=0;i<numofCloseLoop;i++)
    {
	    id1 = closeedges[i].id1;
	    id2 = closeedges[i].id2;
	    //这里要注意第一个回环和最后一个回环需要进行特殊处理
	    if(i==0)
	    {
		    id1 =0;
		    lastid2 = id1;
	    }
	    if(i==numofCloseLoop-1)
	    {
	      id2 = poses.size()-1;
	    }
	    //cout<<"开始处理边->"<<id1<<" "<<id2<<endl;
	    //插入边
	    for(int j=0;j<edges_vo.size();j++)
	    {
		    if( ((edges_vo[j].id1>=lastid2)&&(edges_vo[j].id2<=id2)) || ((edges_vo[j].id2-edges_vo[j].id1!=1)&&(edges_vo[j].id2==closeedges[i].id2)) )//保证回环边要加入到图中 并且vo边不会重复加入
		    {
			    //cout<<"    插入的边="<<edges_vo[j].id1<<" "<<edges_vo[j].id2<<endl;
			    Pose3 delta;
			    Pose_EigenTogtsam(edges_vo[j].delta_t,edges_vo[j].delta_q,delta);
			    gtsam::Matrix mvo;
			    mvo = eye(6);
			    //对信息矩阵进行赋值
			    SharedNoiseModel vomodel = noiseModel::Gaussian::Information(mvo);
			    graph.add(BetweenFactor<Pose3>(edges_vo[j].id1, edges_vo[j].id2, delta, vomodel));//向图中加入边add和push_back是等价的
			    //4.0中使用的是 graph.emplace_shared<BetweenFactor<Pose3>>(id);
			    //cout<<"插入的边 = "<<edges_vo[j].id1<<" "<<edges_vo[j].id2<<endl;
		    }
	    }
	    //cout<<"    插入边完成"<<endl;
	    //插入顶点的初始值
	    if(i==0)//第一个回环
	    {
	      for(int j=lastid2;j<id2+1;j++)
	      {	
		      //cout<<"    插入的顶点="<<j<<endl;
		      Point3 ti= Point3(poses[j].t(0),poses[j].t(1),poses[j].t(2));
		      Rot3 Ri = Rot3::quaternion(poses[j].q.w(),poses[j].q.x(),poses[j].q.y(),poses[j].q.z());
		      initialEstimate.insert(j,Pose3(Ri,ti));
		      //cout<<"插入的顶点="<<j<<endl;
	      }
	    }
	    else//不是第一个回环
	    {
	       Eigen::Vector3d t1 = poses[lastid2].t;
	       Eigen::Matrix3d R1 = poses[lastid2].q.toRotationMatrix();
	      for(int j=lastid2+1;j<id2+1;j++)
	      {	
		      //cout<<"    插入的顶点="<<j<<endl;
		      Eigen::Vector3d t2 = poses[j].t;
		      Eigen::Matrix3d R2 = poses[j].q.toRotationMatrix();
		      Eigen::Vector3d delta_t = R1.transpose()*(t2-t1);
		      Eigen::Quaterniond delta_q;
		      delta_q= R1.transpose()*R2;
		      Pose3 delta;
		      Pose_EigenTogtsam(delta_t,delta_q,delta);
		      initialEstimate.insert(j,lastPose*delta);//第二个回环的状态值使用第一个回环的最后一个状态进行更新，
		      //cout<<"插入的顶点="<<j<<endl;
	      }
	    }
	    //cout<<"    插入顶点完成"<<endl;
	    isam.update(graph, initialEstimate);
	    Values currentEstimate = isam.calculateEstimate();// At any time, calculateEstimate() may be called to obtain the current* estimate of all variables.
	    isam.printStats();//输出状态
	    lastPose = currentEstimate.at<Pose3>(id2);
	    lastid2 = id2;
	    graph.resize(0);//对图清空
	    initialEstimate.clear();//对值清空
	    
	    if(i==numofCloseLoop-1)//最后一个回环优化结束输出优化的值
	    {
		optimized_poses = Value_gtsamToEigen(currentEstimate);
		if(optimized_poses.size()-1==id2)
		{
		  cout<<"顶点个数正确"<<endl;
		}
	    }
    }//遍历每段回环结束
    
    //6.优化结束 写入结果
    cout<<"结束iSAM_margi优化------------------------------------------------------------------"<<endl;
    return optimized_poses;
  }
  
  //使用gtsam提供的非线性优化进行计算
  vector<pose> iSAM_full(vector<pose> poses, vector<edge> edges_vo,string method ="DL",int iteratioin =20)
  {
    vector<pose> optimized_poses;
    //1.构造因子图的类型
    NonlinearFactorGraph graph;
    Values initialEstimate;
    //2.加入先验因子
    noiseModel::Diagonal::shared_ptr priorModel =  noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());//先是位置后是姿态
    Pose3 pose0;
    Pose_EigenTogtsam(poses[0].t,poses[0].q,pose0);
    graph.add(PriorFactor<Pose3>(0, pose0, priorModel));
    
    //边的协方差矩阵
    noiseModel::Diagonal::shared_ptr odoModel =  noiseModel::Diagonal::Variances((Vector(6) << 1, 1, 1, 1, 1, 1).finished());//先是位置后是姿态
    //5.向图中插入边
    for(int i=0;i<edges_vo.size();i++)
    {
      Pose3 deltaT;
      Pose_EigenTogtsam(edges_vo[i].delta_t,edges_vo[i].delta_q,deltaT);
      graph.add(BetweenFactor<Pose3>(edges_vo[i].id1,edges_vo[i].id2,deltaT,odoModel));
    }
    
     //4.插入初值的数据
    for(int i=0;i<poses.size();i++)
    {
      Pose3 Ti;
      Pose_EigenTogtsam(poses[i].t,poses[i].q,Ti);
      initialEstimate.insert(poses[i].id,Ti);
    }
    
    //5.开始进行优化 
    Values result;
    if(method=="DL")
    {
        DoglegParams params;
        params.setVerbosity("ERROR");//打印输出每一步的迭代结果
	//params.setVerbosity("TERMINATION");//打印输出停止的条件
	//params.setVerbosity("SUMMARY");//打印输出停止的条件
	params.setMaxIterations(iteratioin);
	//params.setLinearSolverType("MULTIFRONTAL_QR");
	
	DoglegOptimizer optimizer(graph,initialEstimate,params);
	result = optimizer.optimize();
    }
    else if(method=="GN")
    {
        GaussNewtonParams params;
        params.setVerbosity("ERROR");//打印输出每一步的迭代结果
	//params.setVerbosity("TERMINATION");//打印输出停止的条件
	//params.setVerbosity("SUMMARY");//打印输出停止的条件
	params.setMaxIterations(iteratioin);
	//params.setLinearSolverType("MULTIFRONTAL_QR");
	
	GaussNewtonOptimizer optimizer(graph,initialEstimate,params);
	result = optimizer.optimize();
    }
    else if(method=="LM")
    {
       LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");//打印输出每一步的迭代结果
	//params.setVerbosity("TERMINATION");//打印输出停止的条件
	//params.setVerbosity("SUMMARY");//打印输出停止的条件
	params.setMaxIterations(iteratioin);
	//params.setLinearSolverType("MULTIFRONTAL_QR");
	
	LevenbergMarquardtOptimizer optimizer(graph,initialEstimate,params);
	result = optimizer.optimize();
    }
        
    //GaussNewtonOptimizer optimizer(graph,initialEstimate,params);
    //DoglegOptimizer optimizer(graph,initialEstimate,params);
    //LevenbergMarquardtOptimizer optimizer(graph,initialEstimate,params);
    
    //6.优化之后提取出姿态并输出
    optimized_poses=Value_gtsamToEigen(result);
    
    //7.优化结束 写入结果
    return optimized_poses;
  }
  
  //这里我们从gtsam代码中提取出来了 gtsam的雅克比求导公式 详见我自己写的gtsam笔记
  //这里一定要注意gtsam中位姿李代数的顺序是先 旋转 后 translation 与g2o相同
  //计算得到的是gtsam中定义的伴随矩阵
  void AdjointT()
  {
    
  }
  //measures_是测量值 residual error = log[measures_.inverse()*Ti.inverse()*Tj]
  //如果accuracy为true 表示计算local coordinate雅克比
  Eigen::Matrix<double,6,6> iSAM_Jacobian_Posegraph(Sophus::SE3 measures_,Sophus::SE3 Ti,Sophus::SE3 Tj,bool accuracy=false)
  {
    
  }
  //这个是姿态的雅克比
  void iSAM_Jacobian_R()
  {
    
  }
}
#endif