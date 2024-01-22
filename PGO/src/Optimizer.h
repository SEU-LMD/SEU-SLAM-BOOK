#ifndef _OPTIMIZER_
#define _OPTIMIZER_

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构
//#include <boost/concept_check.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>//块求解器
#include <g2o/core/optimization_algorithm_levenberg.h>//非线性求解方法
#include <g2o/core/optimization_algorithm_dogleg.h>//dogleg算法
#include <g2o/core/optimization_algorithm_gauss_newton.h>//高斯牛顿算法
#include <g2o/solvers/eigen/linear_solver_eigen.h>//线性求解器的头文件
#include <g2o/solvers/dense/linear_solver_dense.h>//线性求解器的头文件
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>//线性求解器的头文件
#include <g2o/types/sba/types_six_dof_expmap.h>//定义了我们需要的边
#include <g2o/core/robust_kernel_impl.h>//用于设置鲁邦核函数
#include <g2o/types/slam3d/edge_se3.h>//g2o提供的pose graph 的边
#include <g2o/types/slam3d/vertex_se3.h>//g2o提供的顶点用于pose graph优化
#include <g2o/types/sim3/types_seven_dof_expmap.h>//orb使用的pose graph优化

#include <sophus/se3.h>
#include <sophus/so3.h>

#include "G2OEdge.h"
#include "types.h"
#include "LAGO_linear.h"//

using namespace std;
//这个文件的作用是使用g2o库执行pose graph优化
namespace posegraph
{
  
    //只对姿态进行优化 使用的顶点是我们自己定义的左乘顶点VertexSO3LieAlgebra
    //边也是我们自己定义的EdgeSO3Base或者EdgeSO3Lie
    vector<pose> Optimize_Rotation(vector<pose> poses,vector<edge> edges,int iterations=20)
    {
	  cout<<"开始优化姿态------------------------------------------------------------------"<<endl;
	  //1.首先进行g2o的参数设置
	  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;  // 每个误差项优化变量维度为3和3,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	  Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	  
	  Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	  
	  //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	  //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	  g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

	  
	  g2o::SparseOptimizer optimizer;     // 图模型
	  optimizer.setAlgorithm( solver );   // 设置求解器
	  optimizer.setVerbose( true );       // 打开调试输出
	  
	  //边中最小的id设置为不进行优化
	  int fixid=-1;
	  for(int i=0;i<edges.size();i++)
	  {
	    int minid = min(edges[i].id1,edges[i].id2);
	    if(fixid==-1)
	    {
	      fixid=minid;
	    }
	    else
	    {
	      if(minid<fixid)
	      {
		fixid = minid;
	      }
	    }
	  }
	  //2.定义定点
	  vector<VertexSO3LieAlgebra*> vectices;
	  for(int i=0;i<poses.size();i++)
	  {
	      VertexSO3LieAlgebra* v = new VertexSO3LieAlgebra();
	      v->setEstimate(Sophus::SO3(poses[i].q));
	      v->setId( poses[i].id );
	      //v->setMarginalized(false);
	      optimizer.addVertex(v);
	      vectices.push_back(v);
	      if(poses[i].id==fixid)
	      {
		v->setFixed(true);
	      }
	  }
	  //3.将边加入到优化模型中***************************************这里我们可以改变边的类型
	  typedef EdgeSO3Base Edgetype;//我们提供了两种边 EdgeSO3Base 和 EdgeSO3Lie
	  //typedef EdgeSO3Lie Edgetype;
	  for(int i=0;i<edges.size();i++)
	  {
		Edgetype* e =  new Edgetype();//两个相机都看到了
		e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
		e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
		e->setMeasurement(Sophus::SO3(edges[i].delta_q));
		e->setId( i );
		e->setInformation(Edgetype::InformationMatrix());
	
		/*if(bRobust)
		{
		    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		    e->setRobustKernel(rk);
		    rk->setDelta(sqrt(7.815));//这个参数的设置是根据orb的源码得到的
		}*/
		optimizer.addEdge(e);
	  }
	  //4.开始优化
	  cout<<"优化之前的误差 = "<<Edgetype::Produce_Error(poses,edges)<<endl;
	  optimizer.initializeOptimization();
	  optimizer.optimize(iterations);
	  
	  //5.对状态进行更新
	  vector<pose> optimized_poses;//优化过的姿态位置 只对姿态进行更新
	  for(int i=0;i<poses.size();i++)
	  {
	     VertexSO3LieAlgebra* v_result = static_cast<VertexSO3LieAlgebra*>(optimizer.vertex(poses[i].id));
	     Sophus::SO3 so3_v =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  poses[i].t;
	     pose_v.q =  so3_v.unit_quaternion();
	     optimized_poses.push_back(pose_v);
	  }
	  cout<<"优化之后的误差 = "<<Edgetype::Produce_Error(optimized_poses,edges)<<endl;
	  cout<<"姿态优化结束------------------------------------------------------------------"<<endl;
	  return optimized_poses;  
    }
  
    //只对位置进行优化 使用的是自己定义的顶点Vertext 自定义的边EdgePositionOnly
    vector<pose> Optimize_t(vector<pose> poses,vector<edge> edges,int iterations)
    {
	  cout<<"开始优化位置------------------------------------------------------------------"<<endl;
	  //1.首先进行g2o的参数设置
	  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;  // 每个误差项优化变量维度为3和3,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	  Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	  Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	  //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	  //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	  g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );
	  g2o::SparseOptimizer optimizer;     // 图模型
	  optimizer.setAlgorithm( solver );   // 设置求解器
	  optimizer.setVerbose( true );       // 打开调试输出
	  //边中最小的id设置为不进行优化
	  int fixid=-1;
	  for(int i=0;i<edges.size();i++)
	  {
	    int minid = min(edges[i].id1,edges[i].id2);
	    if(fixid==-1)
	    {
	      fixid=minid;
	    }
	    else
	    {
	      if(minid<fixid)
	      {
		fixid = minid;
	      }
	    }
	  }
	  //2.定义顶点
	  for(int i=0;i<poses.size();i++)
	  {
	      Vertext* v = new Vertext();
	      v->setId( poses[i].id );
	      v->setEstimate(poses[i].t);
	      optimizer.addVertex(v);
	      if(poses[i].id ==fixid)
	      {
		v->setFixed(true);
	      } 
	  }
	  //3.将边加入到优化模型中
	  for(int i=0;i<edges.size();i++)
	  {
	    EdgePositionOnly* e = new EdgePositionOnly();
	    e->setId( i );
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
	    //如果我们先优化了姿态然后直接将姿态值带入测量方程中，而不是将其看做待优化变量
	    Eigen::Matrix3d R1 = poses[edges[i].id1].q.toRotationMatrix();
	    e->setMeasurement(R1*edges[i].delta_t);
	    e->setInformation(Eigen::Matrix<double,3,3>::Identity());
	    optimizer.addEdge(e);
	  }
	  
	  //4.开始优化
	  cout<<"优化之前误差= "<<EdgePositionOnly::Produce_Error(poses,edges)<<endl;
	  optimizer.initializeOptimization();
	  optimizer.optimize(iterations);
	  //5.更新优化的位姿
	  vector<pose> optimized_poses;
	  for(int i=0;i<poses.size();i++)
	  {
	     Vertext* v_result = static_cast<Vertext*>(optimizer.vertex(poses[i].id));
	     Eigen::Vector3d vector_v =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  vector_v;
	     pose_v.q =  poses[i].q;
	     optimized_poses.push_back(pose_v);
	  }
	  cout<<"优化之后误差= "<<EdgePositionOnly::Produce_Error(optimized_poses,edges)<<endl;
	  cout<<"位置优化结束------------------------------------------------------------------"<<endl;
	  return optimized_poses;
	 
    }
    
    //这个一般用于得到初始值 先计算得到姿态 然后再计算得到位置
    vector<pose> Optimize_SE3Partition(vector<pose> poses,vector<edge> edges,int iterations=20)
    {
      vector<pose> optimized_pose_onlyR;
      vector<pose> optimized_pose;
      optimized_pose_onlyR = Optimize_Rotation(poses,edges,iterations);//这个函数只对姿态优化 Optimize_Rotation
      optimized_pose = Optimize_t(optimized_pose_onlyR,edges,iterations);//
      //optimized_pose = linear_optimize_t(optimized_pose_onlyR,edges);
      return optimized_pose;
    }
  
    //使用slam14讲的posegraph方法进行联合优化
    vector<pose> Optimize_SE3Lie(vector<pose> poses,vector<edge> edges, string method="DL",int iterations=20)
    {
        cout<<"开始优化SE3------------------------------------------------------------------"<<endl;
        //1.首先进行g2o的参数设置
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // 每个误差项优化变量维度为6和6,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	
	g2o::OptimizationAlgorithmWithHessian* solver;//OptimizationAlgorithmWithHessian是父类
	if(method=="DL")
	{
	  solver = new g2o::OptimizationAlgorithmDogleg ( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="LM")
	{
	  solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="GN")
	{
	  solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	}
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出
	//边中最小的id设置为不进行优化
	int fixid=-1;
	for(int i=0;i<edges.size();i++)
	{
	  int minid = min(edges[i].id1,edges[i].id2);
	  if(fixid==-1)
	  {
	    fixid=minid;
	  }
	  else
	  {
	    if(minid<fixid)
	    {
	      fixid = minid;
	    }
	  }
	}
	 //2.定义定点
	 for(int i=0;i<poses.size();i++)
	 {
	   VertexSE3LieAlgebra* v = new VertexSE3LieAlgebra();
	   v->setId( poses[i].id );
	   v->setEstimate(Sophus::SE3(poses[i].q,poses[i].t));
	   optimizer.addVertex(v);
	   if(poses[i].id ==fixid)
	   {
	     v->setFixed(true);
	   } 
	 }
    
         //3. 设置边的信息
         for(int i=0;i<edges.size();i++)
	 {
	    EdgeSE3LieAlgebra* e = new EdgeSE3LieAlgebra();
	    e->setId( i );
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
	    e->setMeasurement(Sophus::SE3(edges[i].delta_q,edges[i].delta_t));
	    e->setInformation(Eigen::Matrix<double,6,6>::Identity());
	    optimizer.addEdge(e);
	 }
        
	//4.开始优化
	cout<<"优化之前的误差 = "<<EdgeSE3LieAlgebra::Produce_Error(poses,edges)<<endl;
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
	
	 //5.对状态进行更新
	  vector<pose> optimized_poses;//优化过的姿态位置 只对姿态进行更新
	  for(int i=0;i<poses.size();i++)
	  {
	     VertexSE3LieAlgebra* v_result = static_cast<VertexSE3LieAlgebra*>(optimizer.vertex(poses[i].id));
	     Sophus::SE3 se3_v =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  se3_v.translation();
	     pose_v.q =  se3_v.unit_quaternion();
	     optimized_poses.push_back(pose_v);
	  }
	 cout<<"优化之后的误差 = "<<EdgeSE3LieAlgebra::Produce_Error(optimized_poses,edges)<<endl;
	 cout<<"SE3优化结束------------------------------------------------------------------"<<endl;
	 return optimized_poses;
    }
    
    //使用g2o提供的VertexSE3顶点和EdgeSE3边进行优化 EdgeSE3是有雅克比公式的误差方程=四元数的虚部
    vector<pose> Optimize_SE3G2O(vector<pose> poses,vector<edge> edges, int iterations=20)
    {
        cout<<"开始优化SE3------------------------------------------------------------------"<<endl;
        //1.首先进行g2o的参数设置
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // 每个误差项优化变量维度为6和6,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出
	//边中最小的id设置为不进行优化
	int fixid=-1;
	for(int i=0;i<edges.size();i++)
	{
	  int minid = min(edges[i].id1,edges[i].id2);
	  if(fixid==-1)
	  {
	    fixid=minid;
	  }
	  else
	  {
	    if(minid<fixid)
	    {
	      fixid = minid;
	    }
	  }
	}
	 //2.定义定点
	 for(int i=0;i<poses.size();i++)
	 {
	   g2o::VertexSE3* v = new g2o::VertexSE3();//VertexSE3=(x,y,z,qx,qy,qz)
	   v->setId( poses[i].id );
	   Eigen::Isometry3d T;
	   T = poses[i].q;
	   T.translation() = poses[i].t;
	   v->setEstimate(T);
	   optimizer.addVertex(v);
	   if(poses[i].id ==fixid)
	   {
	     v->setFixed(true);
	   } 
	 }
    
         //3. 设置边的信息
         for(int i=0;i<edges.size();i++)
	 {
	    g2o::EdgeSE3* e = new g2o::EdgeSE3();
	    e->setId( i );
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
	    Eigen::Isometry3d delta_T;
	    delta_T= edges[i].delta_q;
	    delta_T.translation() = edges[i].delta_t;
	    e->setMeasurement(delta_T);
	    e->setInformation(Eigen::Matrix<double,6,6>::Identity());
	    optimizer.addEdge(e);
	 }
        
	//4.开始优化
	//cout<<"优化之前的误差 = "<<g2o::Edge_SE3::Produce_Error(poses,edges)<<endl;
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
	
	 //5.对状态进行更新
	  vector<pose> optimized_poses;//优化过的姿态位置 只对姿态进行更新
	  for(int i=0;i<poses.size();i++)
	  {
	     g2o::VertexSE3* v_result = static_cast<g2o::VertexSE3*>(optimizer.vertex(poses[i].id));
	     Eigen::Isometry3d isometry_v =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  isometry_v.translation();
	     pose_v.q =  isometry_v.matrix().topLeftCorner<3,3>();
	     optimized_poses.push_back(pose_v);
	  }
	 //cout<<"优化之后的误差 = "<<EdgeSE3LieAlgebra::Produce_Error(optimized_poses,edges)<<endl;
	 cout<<"SE3优化结束------------------------------------------------------------------"<<endl;
	 return optimized_poses;
    }
    
    //使用orb提供的VertexSim3Expmap顶点和EdgeSim3边进行优化 使用的是数值雅克比并没有解析形式 结算的速度很慢而且不容易收敛
    vector<pose> Optimize_SE3ORB(vector<pose> poses,vector<edge> edges, string method ="DL",int iterations=50)
    {
        cout<<"开始优化SE3------------------------------------------------------------------"<<endl;
        //1.首先进行g2o的参数设置
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<7,3> > Block;  // 每个误差项优化变量维度为7或3,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	
	g2o::OptimizationAlgorithmWithHessian* solver;//OptimizationAlgorithmWithHessian是父类
	if(method=="DL")
	{
	  solver = new g2o::OptimizationAlgorithmDogleg ( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="LM")
	{
	  solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="GN")
	{
	  solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	}
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出
	//边中最小的id设置为不进行优化
	int fixid=-1;
	for(int i=0;i<edges.size();i++)
	{
	  int minid = min(edges[i].id1,edges[i].id2);
	  if(fixid==-1)
	  {
	    fixid=minid;
	  }
	  else
	  {
	    if(minid<fixid)
	    {
	      fixid = minid;
	    }
	  }
	}
	 //2.定义定点
	 for(int i=0;i<poses.size();i++)
	 {
	   g2o::VertexSim3Expmap* v = new g2o::VertexSim3Expmap();//这个顶点也是左乘更新
	   v->setId( poses[i].id );
	   g2o::Sim3 sim3(poses[i].q.toRotationMatrix(),poses[i].t,1.0);
	   v->setEstimate(sim3);
	   optimizer.addVertex(v);
	   if(poses[i].id ==fixid)
	   {
	     v->setFixed(true);
	   } 
	   v->setMarginalized(false);//不对这个顶点进行边缘话求解
	   v->_fix_scale = true;//不估计尺度
	 }
    
         //3. 设置边的信息
         for(int i=0;i<edges.size();i++)
	 {
	    g2o::EdgeSim3* e = new g2o::EdgeSim3();
	    e->setId( i );
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
	    g2o::Sim3 delta_T(edges[i].delta_q.toRotationMatrix(),edges[i].delta_t,1.0);
	    e->setMeasurement(delta_T);
	    e->setInformation(Eigen::Matrix<double,7,7>::Identity());
	    optimizer.addEdge(e);
	 }
        
	//4.开始优化
	//cout<<"优化之前的误差 = "<<g2o::Edge_SE3::Produce_Error(poses,edges)<<endl;
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
	
	 //5.对状态进行更新
	  vector<pose> optimized_poses;//优化过的姿态位置 只对姿态进行更新
	  for(int i=0;i<poses.size();i++)
	  {
	     g2o::VertexSim3Expmap* v_result = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(poses[i].id));
	     g2o::Sim3 sim3 =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  sim3.translation();
	     pose_v.q =  sim3.rotation();
	     optimized_poses.push_back(pose_v);
	  }
	 //cout<<"优化之后的误差 = "<<EdgeSE3LieAlgebra::Produce_Error(optimized_poses,edges)<<endl;
	 cout<<"SE3优化结束------------------------------------------------------------------"<<endl;
	 return optimized_poses;
    }
    
    
    //使用的是我们自己定义的代价函数和顶点 详见我们自己写的笔记
    //顶点使用的是我们自己定义的 VertexSE3_fiandt 边使用的是我们自己定义的 EdgeSE3_findt
     vector<pose> Optimize_SE3FYY(vector<pose> poses,vector<edge> edges, string method ="DL",int iterations=20)
    {
        cout<<"开始优化SE3------------------------------------------------------------------"<<endl;
        //1.首先进行g2o的参数设置
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // 每个误差项优化变量维度为7或3,一定要注意这个变量的设置!!!!!!!!!!!!动态的话就是-1 -1
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
	//Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
	Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
	
	g2o::OptimizationAlgorithmWithHessian* solver;//OptimizationAlgorithmWithHessian是父类
	if(method=="DL")
	{
	  solver = new g2o::OptimizationAlgorithmDogleg ( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="LM")
	{
	  solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );
	}
	else if(method=="GN")
	{
	  solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	}
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
	//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
	//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出
	//边中最小的id设置为不进行优化
	int fixid=-1;
	for(int i=0;i<edges.size();i++)
	{
	  int minid = min(edges[i].id1,edges[i].id2);
	  if(fixid==-1)
	  {
	    fixid=minid;
	  }
	  else
	  {
	    if(minid<fixid)
	    {
	      fixid = minid;
	    }
	  }
	}
	 //2.定义定点
	 for(int i=0;i<poses.size();i++)
	 {
	   VertexSE3_fiandt* v = new VertexSE3_fiandt();//这个顶点也是左乘更新
	   v->setId( poses[i].id );
	   v->setEstimate(Sophus::SE3(poses[i].q,poses[i].t));
	   optimizer.addVertex(v);
	   if(poses[i].id ==fixid)
	   {
	     v->setFixed(true);
	   } 
	   //v->setMarginalized(false);//不对这个顶点进行边缘话求解
	 }
    
         //3. 设置边的信息
         for(int i=0;i<edges.size();i++)
	 {
	    EdgeSE3_findt* e = new EdgeSE3_findt();
	    e->setId( i );
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id1)));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(edges[i].id2)));
	    e->setMeasurement(Sophus::SE3(edges[i].delta_q,edges[i].delta_t));
	    e->setInformation(Eigen::Matrix<double,6,6>::Identity());
	    optimizer.addEdge(e);
	 }
        
	//4.开始优化
	cout<<"优化之前的误差 = "<<EdgeSE3_findt::Produce_Error(poses,edges)<<endl;
	optimizer.initializeOptimization();
	optimizer.optimize(iterations);
	
	 //5.对状态进行更新
	  vector<pose> optimized_poses;//优化过的姿态位置 只对姿态进行更新
	  for(int i=0;i<poses.size();i++)
	  {
	     VertexSE3_fiandt* v_result = static_cast<VertexSE3_fiandt*>(optimizer.vertex(poses[i].id));
	     Sophus::SE3 se3_v =  v_result->estimate();
	     pose pose_v;
	     pose_v.id = poses[i].id;
	     pose_v.t =  se3_v.translation();
	     pose_v.q =  se3_v.unit_quaternion();
	     optimized_poses.push_back(pose_v);
	  }
	 cout<<"优化之后的误差 = "<<EdgeSE3_findt::Produce_Error(optimized_poses,edges)<<endl;
	 cout<<"SE3优化结束------------------------------------------------------------------"<<endl;
	 return optimized_poses;
    }
    
    
    //一元边, 第一个参数是测量值 第二个参数是初值,第三个参数是信息矩阵，第四个参数是我们迭代多少次 informationmatrix先是fi后是位移
    edge optimize_Unary_Lie(vector<edge> poses,edge initial_pose,int iterations=20)
    {
      cout<<initial_pose.id1<<"-"<<initial_pose.id2<<" start optimization"<<endl;
      //1.首先进行g2o的参数设置
      typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // 误差项优化变量维度为6 误差值维度为6
      Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
      //Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
      //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
      Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
      //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
      //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
      g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

      g2o::SparseOptimizer optimizer;     // 图模型
      optimizer.setAlgorithm( solver );   // 设置求解器
      optimizer.setVerbose( true );       // 打开调试输出
      
      //2.定义顶点
      VertexSE3LieAlgebra* v = new VertexSE3LieAlgebra();//这个顶点也是左乘更新
      v->setId( 0 );
      v->setEstimate(Sophus::SE3(initial_pose.delta_q,initial_pose.delta_t));
      optimizer.addVertex(v);
	   
      //3. 设置边的信息
      for(int i=0;i<poses.size();i++)
      {
	EdgeUnary_SE3Lie* e = new EdgeUnary_SE3Lie();
	e->setId( i );
	e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
	e->setMeasurement(Sophus::SE3(poses[i].delta_q,poses[i].delta_t));
	e->setInformation( poses[i].cov);
	cout<<"edge "<<i<<" info matrix = "<<poses[i].cov(0,0)<<" "<<poses[i].cov(1,1)<<" "<<poses[i].cov(2,2)<<" "<<poses[i].cov(3,3)<<" "<<poses[i].cov(4,4)<<" "<<poses[i].cov(5,5)<<endl;
	optimizer.addEdge(e);
      }
      //4.开始优化
      optimizer.initializeOptimization();
      optimizer.optimize(iterations);
       //5.对状态进行更新
      VertexSE3LieAlgebra* v_result = static_cast<VertexSE3LieAlgebra*>(optimizer.vertex(0));
      Sophus::SE3 se3_v =  v_result->estimate();
      edge edge_v;
      edge_v.delta_t =  se3_v.translation();
      edge_v.delta_q =  se3_v.unit_quaternion();
      edge_v.id1 = initial_pose.id1;
      edge_v.id2 = initial_pose.id2;
      
      return edge_v;
    }
    //与上面一个函数有所区别的是我们使用的是我们自己定义的边进行优化的 详见我们的笔记,informationmatrix先是fi后是位移
    edge optimize_Unary_fyy(vector<edge> poses,edge initial_pose,int iterations=20)
    {
      cout<<initial_pose.id1<<"-"<<initial_pose.id2<<" start optimization"<<endl;
      //1.首先进行g2o的参数设置
      typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // 误差项优化变量维度为6 误差值维度为6
      Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); //线性方程求解器 
      //Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //线性方程求解器，
      //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器 
      Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器,Block的初始化变量必须是unique_ptr
      //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr) );//OptimizationAlgorithmLevenberg函数的初始化变量必须是unique_ptr
      //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::unique_ptr<Block>(solver_ptr) );
      g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::unique_ptr<Block>(solver_ptr) );

      g2o::SparseOptimizer optimizer;     // 图模型
      optimizer.setAlgorithm( solver );   // 设置求解器
      optimizer.setVerbose( true );       // 打开调试输出
      
      //2.定义顶点
      VertexSE3_fiandt* v = new VertexSE3_fiandt();//这个顶点也是左乘更新
      v->setId( 0 );
      v->setEstimate(Sophus::SE3(initial_pose.delta_q,initial_pose.delta_t));
      optimizer.addVertex(v);
	   
      //3. 设置边的信息
      for(int i=0;i<poses.size();i++)
      {
	EdgeUnary_SE3Fyy* e = new EdgeUnary_SE3Fyy();
	e->setId( i );
	e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
	e->setMeasurement(Sophus::SE3(poses[i].delta_q,poses[i].delta_t));
	e->setInformation(poses[i].cov);
	cout<<"edge "<<i<<" info matrix = "<<poses[i].cov(0,0)<<" "<<poses[i].cov(1,1)<<" "<<poses[i].cov(2,2)<<" "<<poses[i].cov(3,3)<<" "<<poses[i].cov(4,4)<<" "<<poses[i].cov(5,5)<<endl;
	optimizer.addEdge(e);
      }
      //4.开始优化
      optimizer.initializeOptimization();
      optimizer.optimize(iterations);
       //5.对状态进行更新
      VertexSE3_fiandt* v_result = static_cast<VertexSE3_fiandt*>(optimizer.vertex(0));
      Sophus::SE3 se3_v =  v_result->estimate();
      edge edge_v;
      edge_v.delta_t =  se3_v.translation();
      edge_v.delta_q =  se3_v.unit_quaternion();
      edge_v.id1 = initial_pose.id1;
      edge_v.id2 = initial_pose.id2;
      return edge_v;
    }

}
#endif