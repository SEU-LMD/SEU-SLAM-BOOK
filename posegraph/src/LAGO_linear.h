#ifndef _LAGO_
#define _LAGO_

extern "C" {
#include "cs.h"//这个是suitesparse的头文件
}

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include <sophus/se3.h>
#include <sophus/so3.h>

#include "types.h"
#include "G2OEdge.h"
using namespace std;
//这个文件中包含了线性求解位移t和lago的方法
namespace posegraph
{
   /*
      //lago的调用方法
      cout<<"3.LAGO"<<endl;
      cout<<"优化之前与真值之间的姿态误差 = "<<R_Error(pose_gt,optimized_pose_Rt)<<endl;
      cout<<"优化之前与真值之间的位置误差 = "<<t_Error(pose_gt,optimized_pose_Rt)<<endl;
      vector<pose> lago_pose;
      lago_pose= LAGO(optimized_pose_Rt,edges_vo);
      cout<<"优化之后与真值之间的姿态误差 = "<<R_Error(pose_gt,lago_pose)<<endl;
      cout<<"优化之后与真值之间的位置误差 = "<<t_Error(pose_gt,lago_pose)<<endl;
      //string path_own = "/home/fyy/Documents/my_code/posegraph/build/whole_result/optimized_partition.g2o";
      //write_G2O_file(path_own,edges_vo,optimized_pose_Rt);
      */
  
 //这个函数将cs结构的列向量转换成指针结构 下面的函数用到了
    double * fromCompressedColumnVectorToDoubleArrayPointer(cs* T) 
    {
	//    declare a double array with all the elements set to be zeros;
	int size = T->m;
        int col = T->n;
	double * arrPtr = new double[size];
	for (int i = 0; i < size; i++) 
	{
		arrPtr[i] = 0;
	}

	int ptr = 0;
	int indexLow = *(T->p + ptr);//p是矩阵列的指针
	int indexHigh = *(T->p + ptr + 1) - 1;
	for (int idx = indexLow; idx <= indexHigh; idx++) 
	{
		int row = *(T->i + idx);
		double val = *(T->x + idx);
		arrPtr[row] = val;
	}

	return arrPtr;
     }
     
     cs *constructIdentity(int n)
     {
       cs * I = cs_spalloc(n,n,n,1,1);
       for(int i=0;i<n;i++)
       {
	 cs_entry(I,i,i,1);
       }
       return I;
     }
     
     cs * combineTwoSparseMatrixLeftAndRight(cs*L, cs*R) 
     {
	int rowL = L->m;
	int colL = L->n;
	int rowR = R->m;
	int colR = R->n;
	int row = rowL;
	int col = colL + colR;

	if (rowL != rowR) {
		cerr << "can't be combined...row number should match." << endl;
	}

	int nzmax = L->nzmax + R->nzmax;
	cs * M = cs_spalloc(row, col, nzmax, 1, 1);

	for (int ptr = 0; ptr < colL; ptr++) {
		int indexLow = *(L->p + ptr);
		int indexHigh = *(L->p + ptr + 1) - 1;
		for (int idx = indexLow; idx <= indexHigh; idx++) {
			int row = *(L->i + idx);
			double val = *(L->x + idx);
			cs_entry(M, row, ptr, val);
		}
	}
	for (int ptr = 0; ptr < colR; ptr++) {
		int indexLow = *(R->p + ptr);
		int indexHigh = *(R->p + ptr + 1) - 1;
		for (int idx = indexLow; idx <= indexHigh; idx++) {
			int row = *(R->i + idx);
			double val = *(R->x + idx);
			cs_entry(M, row, colL + ptr, val);
		}
	}

	return M;
    }

    cs * combineTwoSparseMatrixUpAndLow(cs* U, cs* D) 
    {
	    int rowU = U->m;
	    int colU = U->n;
	    int rowD = D->m;
	    int colD = D->n;
	    int row = rowU + rowD;
	    int col = colU;

	    if (colU != colD) {
		    cerr << "can't be combined...column number should match." << endl;
	    }

	    int nzmax = U->nzmax + D->nzmax;
	    cs * M = cs_spalloc(row, col, nzmax, 1, 1);

	    for (int ptr = 0; ptr < colU; ptr++) {
		    int indexLow = *(U->p + ptr);
		    int indexHigh = *(U->p + ptr + 1) - 1;
		    for (int idx = indexLow; idx <= indexHigh; idx++) {
			    int row = *(U->i + idx);
			    if (row >= rowU)
				    cout << "error: row= " << row << endl;
			    double val = *(U->x + idx);
			    cs_entry(M, row, ptr, val);
		    }
	    }
	    for (int ptr = 0; ptr < colD; ptr++) {
		    int indexLow = *(D->p + ptr);
		    int indexHigh = *(D->p + ptr + 1) - 1;
		    for (int idx = indexLow; idx <= indexHigh; idx++) {
			    int row = *(D->i + idx);
			    double val = *(D->x + idx);
			    cs_entry(M, row + rowU, ptr, val);
		    }
	    }

	    return M;
    }
    //使用线性的方法求得 t Ax=b
    //这里输入的poses姿态已经得到优化 返回的是位置已经得到优化的位姿
    vector<pose> linear_optimize_t(vector<pose> poses,vector<edge> edges)
    {
      cout<<"优化之前误差= "<<EdgePositionOnly::Produce_Error(poses,edges)<<endl;
      cout<<"线性方法优化位置------------------------------------------------------------------"<<endl;
      int n = poses.size();
      int m = edges.size();
      
      //1. 首先构造相关的矩阵
      cs *b_cs = cs_spalloc(3*m,1,3*m,1,1);
      cs *A_cs = cs_spalloc(3*m,3*(n-1),9*m*(n-1),1,1);
      Eigen::VectorXd x = Eigen::VectorXd::Zero(3*(n-1));
      
      //2.对相关的矩阵进行赋值
      for(int i=0;i<edges.size();i++)
      {
	int id1 = edges[i].id1;
	int id2 = edges[i].id2;
	Eigen::Matrix3d R1 = poses[id1].q.toRotationMatrix();
	Eigen::Vector3d t_delta = R1*edges[i].delta_t;
	cs_entry(b_cs,3*i,1, t_delta(0));
	cs_entry(b_cs,3*i+1,1, t_delta(1));
	cs_entry(b_cs,3*i+2,1, t_delta(2));
	//对A矩阵进行赋值
	if(id1!=0)
	{
	  cs_entry(A_cs,3*(id1-1),3*(id1-1),-1);
	  cs_entry(A_cs,3*(id1-1)+1,3*(id1-1)+1,-1);
	  cs_entry(A_cs,3*(id1-1)+2,3*(id1-1)+2,-1);
	}
	if(id2!=0)
	{
	  cs_entry(A_cs,3*(id2-1),3*(id2-1),1);
	  cs_entry(A_cs,3*(id2-1)+1,3*(id2-1)+1,1);
	  cs_entry(A_cs,3*(id2-1)+2,3*(id2-1)+2,1);
	}
      }
      cs *b_ccf = cs_compress(b_cs);
      cs *A_ccf = cs_compress(A_cs);
      //3.线性求解 x
      cs *AT_ccf = cs_transpose(A_ccf,1);
      cs *ATA_ccf = cs_multiply(AT_ccf,A_ccf);
      cs *ATb_ccf = cs_multiply(AT_ccf,b_ccf);
      double * b_pointer = fromCompressedColumnVectorToDoubleArrayPointer(ATb_ccf);
      cs_cholsol(1,ATA_ccf,b_pointer);
      double * poseEstimated = b_pointer;
      for(int i=0;i<3*(n-1);i++)
      {
	x(i) = poseEstimated[i];
      }
      //x = (A.transpose()*A).llt().solve(A.transpose()*b);
      
      //4.根据解算得到的t更新位姿中的位置
      vector<pose> linear_result_pose;
      pose pose0;
      pose0.id = poses[0].id;
      pose0.t =  poses[0].t;
      pose0.q =  poses[0].q;
      linear_result_pose.push_back(pose0);
      for(int i=0;i<n-1;i++)
      {
	pose posei;
	Eigen::Vector3d t(x(3*i),x(3*i+1),x(3*i+2));
	posei.id = poses[i+1].id;
	posei.t = t;
	posei.q = poses[i+1].q;
	linear_result_pose.push_back(posei);
      }
      cout<<"优化之后误差= "<<fixed<<EdgePositionOnly::Produce_Error(linear_result_pose,edges)<<endl;
      return linear_result_pose;
    }
    
    //得到次优解后 然后使用lago算法得到最优解
    vector<pose> LAGO(vector<pose> poses,vector<edge> edges)
    {
      int m = edges.size();
      int n = poses.size();
      cs *Jneg_cs = cs_spalloc(3*m,3*(n-1),9*m*(n-1),1,1);
      cs *B_cs = cs_spalloc(3*m+3*(n-1),6*(n-1),6*(n-1)*(3*m+3*(n-1)),1,1);
      cs *R_cs = cs_spalloc(3*m,3*m,9*m*m,1,1);
      cs *z_cs = cs_spalloc(3*m+3*(n-1),1,3*m+3*(n-1),1,1);
    
      //1.下面进行赋值
      //1.1对R矩阵进行赋值
      for(int i=0;i<edges.size();i++)
      {
	int id1 = edges[i].id1;
	Eigen::Matrix3d Ri = poses[id1].q.toRotationMatrix();
	cs_entry(R_cs,3*i,3*i,Ri(0,0));  cs_entry(R_cs,3*i,3*i+1,Ri(0,1));    cs_entry(R_cs,3*i,3*i+2,Ri(0,2));
	cs_entry(R_cs,3*i+1,3*i,Ri(1,0));cs_entry(R_cs,3*i+1,3*i+1,Ri(1,1));  cs_entry(R_cs,3*i+1,3*i+2,Ri(1,2));
	cs_entry(R_cs,3*i+2,3*i,Ri(2,0));cs_entry(R_cs,3*i+2,3*i+1,Ri(2,1));  cs_entry(R_cs,3*i+2,3*i+2,Ri(2,2));
      }
      cs *R_ccf = cs_compress(R_cs);
      //1.2对J矩阵赋值
      for(int i=0;i<edges.size();i++)
      {
	int id1 = edges[i].id1;
	if(id1!=0)
	{
	  Eigen::Vector3d Rp = poses[id1].q.toRotationMatrix()*edges[i].delta_t;
	  Eigen::Matrix3d Jineg = Hat(Rp);
	  cs_entry(Jneg_cs,3*i,  3*(id1-1),Jineg(0,0));cs_entry(Jneg_cs,3*i, 3*(id1-1)+1, Jineg(0,1));cs_entry(Jneg_cs,3*i,  3*(id1-1)+2,Jineg(0,2));
	  cs_entry(Jneg_cs,3*i+1,3*(id1-1),Jineg(1,0));cs_entry(Jneg_cs,3*i+1,3*(id1-1)+1,Jineg(1,1));cs_entry(Jneg_cs,3*i+1,3*(id1-1)+2,Jineg(1,2));
	  cs_entry(Jneg_cs,3*i+2,3*(id1-1),Jineg(2,0));cs_entry(Jneg_cs,3*i+2,3*(id1-1)+1,Jineg(2,1));cs_entry(Jneg_cs,3*i+2,3*(id1-1)+2,Jineg(2,2));
	}
      }
      cs *Jneg_ccf = cs_compress(Jneg_cs);
      cs* JnegT_ccf = cs_transpose(Jneg_ccf,1);
      cs *JTJ = cs_multiply(JnegT_ccf,Jneg_ccf);
      //1.3对B矩阵进行赋值
       //先对A矩阵赋值
	for(int i=0;i<edges.size();i++)
	{
	  int id1 = edges[i].id1;
	  int id2 = edges[i].id2;
	  //对A矩阵进行赋值
	  if(id1!=0)
	  {
	    cs_entry(B_cs,3*(id1-1),3*(id1-1),-1);
	    cs_entry(B_cs,3*(id1-1)+1,3*(id1-1)+1,-1);
	    cs_entry(B_cs,3*(id1-1)+2,3*(id1-1)+2,-1);
	  }
	  if(id2!=0)
	  {
	    cs_entry(B_cs,3*(id2-1),3*(id2-1),1);
	    cs_entry(B_cs,3*(id2-1)+1,3*(id2-1)+1,1);
	    cs_entry(B_cs,3*(id2-1)+2,3*(id2-1)+2,1);
	  }
	}
       //然后对单位矩阵赋值
       for(int i=0;i<3*(n-1);i++)
       {
	 cs_entry(B_cs,3*m+i,3*(n-1)+i,1);
       }
      cs *B_ccf = cs_compress(B_cs);
      cs *BT_ccf = cs_transpose(B_ccf,1);
      //1.4对测量值z赋值
	//先赋值位移的测量值
	for( int i=0;i<edges.size();i++)
	{
	  Eigen::Vector3d delta_t = edges[i].delta_t;
	  cs_entry(z_cs,3*i,1,delta_t(0));
	  cs_entry(z_cs,3*i+1,1,delta_t(1));
	  cs_entry(z_cs,3*i+2,1,delta_t(2));
	}
	//再赋值李代数
	for(int i=0;i<poses.size();i++)
	{
	  if(i!=0)
	  {
	    Sophus::SO3 so3(poses[i].q);
	    Eigen::Vector3d fi = so3.log();
	    cs_entry(z_cs,3*m+3*(i-1),1,fi(0));
	    cs_entry(z_cs,3*m+3*(i-1)+1,1,fi(1));
	    cs_entry(z_cs,3*m+3*(i-1)+2,1,fi(2));
	  }
	}
      cs * z_ccf = cs_compress(z_cs);
      //1.5对Pzinv赋值
      cs *I3n_cs = constructIdentity(3*(n-1));
      cs *I3n_ccf = cs_compress(I3n_cs);
      cs *IaddJTJ = cs_add(I3n_ccf,JTJ,1,1);
      cs *I3m_cs = constructIdentity(3*m);
      cs *I3m_ccf = cs_compress(I3m_cs);
      cs * UP = combineTwoSparseMatrixLeftAndRight(I3m_ccf,Jneg_ccf);
      cs * Down = combineTwoSparseMatrixLeftAndRight(JnegT_ccf,IaddJTJ);
      cs * Pzinv_cs = combineTwoSparseMatrixUpAndLow(UP,Down);
      cs * Pzinv_ccf = cs_compress(Pzinv_cs);
      
      //2.进行矩阵的计算
      cs * BTPzinv = cs_multiply(BT_ccf,Pzinv_ccf);
      cs * BTPzinvB = cs_multiply(BTPzinv,B_ccf);
      
      cs * BTPzinvz = cs_multiply(BTPzinv,z_ccf);
      double * bpointer = fromCompressedColumnVectorToDoubleArrayPointer(BTPzinvz);
      //3.进行矩阵的求解
      cs_cholsol(1,BTPzinvB,bpointer);
      double * estimatedPose = bpointer;
      //4.进行状态更新
      vector<pose> optimized_poses;
      pose posei;
      posei.t = poses[0].t;
      posei.q = poses[0].q;
      posei.id = 0;
      optimized_poses.push_back(posei);
      
      for(int i=0;i<n-1;i++)
      {
	Eigen::Vector3d ti(estimatedPose[3*i],estimatedPose[3*i+1],estimatedPose[3*i+2]);
	Eigen::Vector3d fi(estimatedPose[3*(n-1)+3*i],estimatedPose[3*(n-1)+3*i+1],estimatedPose[3*(n-1)+3*i+2]);
	Sophus::SO3 so3 = Sophus::SO3::exp(fi);
	pose posei;
	posei.t = ti;
	posei.q = so3.unit_quaternion();
	posei.id = i+1;
        optimized_poses.push_back(posei);
      }
      return optimized_poses;
    }
}
#endif