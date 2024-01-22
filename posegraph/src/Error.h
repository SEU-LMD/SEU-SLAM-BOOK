#ifndef _Error_
#define _Error_
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构
#include <boost/concept_check.hpp>
#include <ctime>
#include <cstdlib>//用于生成随机数
#include <math.h>
#include "types.h"

using namespace std;
//这个文件的作用是读取文件到程序中
namespace posegraph
{
  //使用kitti数据集的方法来判断两个姿态的误差
  double ErrorR_KITTI(Eigen::Matrix3d Ra,Eigen::Matrix3d Rb)
  {
    Eigen::Matrix3d R_delta = Ra.transpose()*Rb;
    double d = 0.5*(R_delta(0,0)+R_delta(1,1)+R_delta(2,2)-1);
    double error = acos( max( min(d,1.0),-1.0 ) );//输出范围位0 pi
     if(error<0)
    {
      cout<<"error!"<<endl;
    }
    return error;
  }
  double ErrorR_KITTI(Eigen::Quaterniond qa,Eigen::Quaterniond qb)
  {
    Eigen::Matrix3d Ra = qa.toRotationMatrix();
    Eigen::Matrix3d Rb = qb.toRotationMatrix();
    Eigen::Matrix3d R_delta = Ra.transpose()*Rb;
    double d = 0.5*(R_delta(0,0)+R_delta(1,1)+R_delta(2,2)-1);
    double error = acos( max( min(d,1.0),-1.0 ) );//输出范围位0 pi
    if(error<0)
    {
      cout<<"error!"<<endl;
    }
    return error;
  }
  
  //和真值比较姿态的总误差
  double R_Error(vector<pose> poses_tru,vector<pose> poses_vo)
  {
    double sum_error=0;
    if(poses_tru.size()==poses_vo.size())
    {
      for(int i=0;i<poses_tru.size();i++)
      {
	Eigen::Matrix3d R_tru = poses_tru[i].q.toRotationMatrix();
	Eigen::Matrix3d R_vo = poses_vo[i].q.toRotationMatrix();
	sum_error = sum_error + ErrorR_KITTI(R_vo,R_tru);
      }
    }
    else
    {
      cout<<"输入的评测序列大小不同!"<<endl;
    }
    return sum_error;
  }
  
  //和真值比较位置的总误差
  double t_Error(vector<pose> poses_tru,vector<pose> poses_vo)
  {
    double sum_error=0;
    if(poses_tru.size()==poses_vo.size())
    {
      for(int i=0;i<poses_tru.size();i++)
      {
	Eigen::Vector3d t_delta = poses_tru[i].t-poses_vo[i].t;
	double error = t_delta.norm();
	sum_error = sum_error + error;
      }
    }
    else
    {
      cout<<"输入的评测序列大小不同!"<<endl;
    }
    return sum_error;
  }
  
  //这个函数在KITTI_Evaluate中用到表示找到 表示找到距离pose[id]刚大于100米的位姿序号
  int find_distn(vector<pose> poses,int id,double distance =100)
  {
    int id_result=0;
    Eigen::Vector3d t0 = poses[id].t;
    Eigen::Quaterniond q0 = poses[id].q;
    for(int i=id+1;i<poses.size();i++)
    {
      Eigen::Vector3d ti =poses[i].t;
      Eigen::Quaterniond qi = poses[i].q;
      Eigen::Vector3d delta_t;
      delta_t = q0.toRotationMatrix()*(ti-t0);
      if(delta_t.norm()>distance)
      {
	id_result = i;
	break;
      }
    }
    return id_result;
  }
  //根据我们自己定义的pose类型 计算两个姿态的差值 T1.inverse*T2 此函数用于KITTI_Evaluate函数中
  void T_delta(pose T1,pose T2,Eigen::Vector3d &delta_t,Eigen::Quaterniond &delta_q)
  {
    Eigen::Vector3d t1 = T1.t;
    Eigen::Vector3d t2 = T2.t;
    Eigen::Quaterniond q1= T1.q;
    Eigen::Quaterniond q2 = T2.q;
    delta_q = q1.toRotationMatrix().transpose()*q2.toRotationMatrix();
    delta_t = q1.toRotationMatrix().transpose()*(t2-t1);
  }
  /*
  void T_mul(pose T1,pose T2,Eigen::Vector3d &mul_t,Eigen::Quaterniond &mul_q)//T1×T2
  {
    Eigen::Vector3d t1 = T1.t;
    Eigen::Vector3d t2 = T2.t;
    Eigen::Quaterniond q1= T1.q;
    Eigen::Quaterniond q2 = T2.q;
    mul_q = q1.toRotationMatrix()*q2.toRotationMatrix();
    mul_t = q1.toRotationMatrix()*t2+t1;
  }*/
  //使用kiit评价指标 详见我们自己写的评价指标方法 
  //输出的是平均位置误差和平均姿态误差
  void KITTI_Evaluate(vector<pose> poses_tru,vector<pose> poses_vo,double &error_t,double &error_q)
  {
    if(poses_tru.size()==poses_vo.size())
    {
      int count = 0;//一共计算了多少个姿态的误差
      double sum_R= 0;//存储姿态的总误差
      double sum_t =0;//存储位置的总误差
      for(int i=0;i<poses_vo.size()-1;i=i+10)//每隔10个坐标计算一次
      {
	int id  = find_distn(poses_tru,i);//找到大于100米的id
	if(id!=0)
	{
	  count++;
	  Eigen::Vector3d delta_true_t;
	  Eigen::Quaterniond delta_true_q;
	  Eigen::Vector3d delta_vo_t;
	  Eigen::Quaterniond delta_vo_q;
	  T_delta(poses_tru[i],poses_tru[id],delta_true_t,delta_true_q);
	  T_delta(poses_vo[i],poses_vo[id],delta_vo_t,delta_vo_q);
	  sum_R = sum_R + ErrorR_KITTI(delta_vo_q,delta_true_q);
	  sum_t = sum_t + (delta_vo_q.toRotationMatrix().transpose()*(delta_true_t-delta_vo_t)).norm();
	}
      }
      error_t = sum_t/count;
      error_q = sum_R/count;
    }
    else
    {
      cout<<"输入的评测序列大小不同"<<endl;
    }
  }
  //生成误差
  //start 和 end 都为 0 时 默认估计范围是全部
  ERRORS produce_error(vector<pose> poses_tru,vector<pose> poses_vo,vector<pose> poses_opt,string error_kind="误差结果:",int start=0,int end = 0)
  {
    if((poses_tru.size()==poses_vo.size())&&(poses_vo.size()==poses_opt.size()))
    {
     
     ERRORS error;
     if((start==0)&&(end==0))//评价的范围是全部
     {
	int numofpose = poses_tru.size();
	error.i_sum_t = t_Error(poses_tru,poses_vo)/numofpose;
	error.opt_sum_t = t_Error(poses_tru,poses_opt)/numofpose;
	if( (poses_tru[0].q.x()!=poses_tru[20].q.x()) && (poses_tru[0].q.y()!=poses_tru[20].q.y()) )//表示姿态存在真值
	{
	  error.i_sum_R = R_Error(poses_tru,poses_vo)/numofpose;
	  error.opt_sum_R = R_Error(poses_tru,poses_opt)/numofpose;
	  KITTI_Evaluate(poses_tru,poses_vo, error.i_avg_t,  error.i_avg_R);
	  KITTI_Evaluate(poses_tru,poses_opt,error.opt_avg_t,error.opt_avg_R);
	}
	
     }
     else//估计范围不是所有而是从start 到 end
     {
       int numofpose = end-start+1;
       Eigen::Quaterniond q_start_tru = poses_tru[start].q;//起始估计帧的姿态真值
       Eigen::Vector3d t_start_tru = poses_tru[start].t;
       
       Eigen::Quaterniond q_start_vo = poses_vo[start].q;//起始估计帧的姿态Vo值
       Eigen::Vector3d t_start_vo = poses_vo[start].t;
       
       Eigen::Quaterniond q_start_opt = poses_opt[start].q;//起始估计帧的姿态优化后的值
       Eigen::Vector3d t_start_opt = poses_opt[start].t;
       vector<pose> poses_tru_subset;
       vector<pose> poses_vo_subset;
       vector<pose> poses_opt_subset;
       for(int i=start;i<=end;i++)
       {
	 //cout<<i<<endl;
	 
	  pose posei_tru,posei_vo,posei_opt;
	  posei_tru.q = q_start_tru.toRotationMatrix().transpose()*poses_tru[i].q.toRotationMatrix();
	  posei_tru.t = q_start_tru.toRotationMatrix().transpose()*(poses_tru[i].t-t_start_tru);
	  poses_tru_subset.push_back(posei_tru);
	  
	  posei_opt.q = q_start_opt.toRotationMatrix().transpose()*poses_opt[i].q.toRotationMatrix();
	  posei_opt.t = q_start_opt.toRotationMatrix().transpose()*(poses_opt[i].t-t_start_opt);
	  poses_opt_subset.push_back(posei_opt);
	  
	  posei_vo.q = q_start_vo.toRotationMatrix().transpose()*poses_vo[i].q.toRotationMatrix();
	  posei_vo.t = q_start_vo.toRotationMatrix().transpose()*(poses_vo[i].t-t_start_vo);
	  poses_vo_subset.push_back(posei_vo);
	 /*
	  poses_tru_subset.push_back(poses_tru[i]);
	  poses_opt_subset.push_back(poses_opt[i]);
	  poses_vo_subset.push_back(poses_vo[i]);*/
       }
       error.i_sum_R = R_Error(poses_tru_subset,poses_vo_subset)/numofpose;
       error.i_sum_t = t_Error(poses_tru_subset,poses_vo_subset)/numofpose;
       error.opt_sum_R = R_Error(poses_tru_subset,poses_opt_subset)/numofpose;
       error.opt_sum_t = t_Error(poses_tru_subset,poses_opt_subset)/numofpose;
       if( (poses_tru[0].q.x()!=poses_tru[20].q.x()) && (poses_tru[0].q.y()!=poses_tru[20].q.y()) )//表示姿态存在真值
       {
	  KITTI_Evaluate(poses_tru_subset,poses_vo_subset, error.i_avg_t,  error.i_avg_R);
	  KITTI_Evaluate(poses_tru_subset,poses_opt_subset,error.opt_avg_t,error.opt_avg_R);
       }
     }
     cout<<error_kind<<endl;
     cout<<"优化之前的绝对误差 ： "<<"位置绝对误差 = "<<error.i_sum_t;
     if( (poses_tru[0].q.x()!=poses_tru[20].q.x()) && (poses_tru[0].q.y()!=poses_tru[20].q.y()) )//表示姿态存在真值,针对匹兹堡数据集
      cout<<" 姿态绝对误差 = "<<error.i_sum_R<<endl;
     else
       cout<<endl;
     cout<<"优化之后的绝对误差 ： "<<"位置绝对误差 = "<<error.opt_sum_t;
     if( (poses_tru[0].q.x()!=poses_tru[20].q.x()) && (poses_tru[0].q.y()!=poses_tru[20].q.y()) )//表示姿态存在真值)
	cout<<" 姿态绝对误差 = "<<error.opt_sum_R<<endl;
     else
       cout<<endl;
     if( (poses_tru[0].q.x()!=poses_tru[20].q.x()) && (poses_tru[0].q.y()!=poses_tru[20].q.y()) )//表示姿态存在真值
     {
	cout<<"优化之前的相对误差 ： "<<"位置相对误差 = "<<error.i_avg_t<<" 姿态相对误差 = "<<error.i_avg_R<<endl;
	cout<<"优化之后的相对误差 ： "<<"位置相对误差 = "<<error.opt_avg_t<<" 姿态相对误差 = "<<error.opt_avg_R<<endl;
     }
     return error;
    }
    else
    {
      cout<<"输入的评测序列大小不同"<<endl;
    }
  }
  
  
   void produe_random_error(Eigen::Vector3d& delta_t,Eigen::Quaterniond& delta_q)
  {
    double x = rand()%100;
    double y = rand()%100;
    double z = rand()%100;
    double a = rand()%100;
    double b = rand()%100;
    double c = rand()%100;
    double d = rand()%100;
    cout<<x<<" "<<y<<" "<<z<<" "<<a<<" "<<b<<" "<<c<<" "<<d<<endl;
    double norm = sqrt(a*a+b*b+c*c+d*d);
    a= a/norm;
    b = b/norm;
    c = c/norm;
    d = d/norm;
    delta_t = Eigen::Vector3d(x,y,z);
    delta_q = Eigen::Quaterniond(a,b,c,d);
  }
  void Add_noise(vector<edge>& edges,string s)
  {
    string::size_type position;
    edge noise_edge;
    int start,end;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    if(s.find("KITTI_00")!=s.npos)
    {
       start = 2000;
       end = 3000;
    }
    if(s.find("KITTI_02")!=s.npos)
    {
      start = 2000;
      end = 4000;
    }
    if(s.find("Pittsburgh_A")!=s.npos)
    {
      start = 3500;
      end = 4000;
    }
    if(s.find("Pittsburgh_B")!=s.npos)
    {
      start = 2000;
      end = 4000;
    }
    if(s.find("Pittsburgh_C")!=s.npos)
    {
      start = 2000;
      end = 6000;
    }
    produe_random_error(t,q);
    noise_edge.id1 = start;
    noise_edge.id2 = end;
    noise_edge.delta_q = q;
    noise_edge.delta_t = t;
    edges.push_back(noise_edge);
  }
  
 
}
#endif