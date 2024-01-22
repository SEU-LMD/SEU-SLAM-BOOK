#ifndef _FILEIO_
#define _FILEIO_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
//用于删除目录
#include <dirent.h>  
//用于新建目录
#include <sys/stat.h>

#include "deal_string.h"
#include "types.h"
using namespace std;
//这个文件的作用是读取文件到程序中
namespace posegraph
{       
	//输入读取的文件名filename输出为边和定点的位姿
	int read_G2O_file(string filename,vector<edge>& vEdges,vector<pose>& poses)
	{
	    int mistake =1;
	    ifstream ifs;
	    ifs.open(filename.c_str());
	    if (!ifs.is_open()) {
		    cout << "Can't open file " << filename << endl;
		    return -1;
	    }
	    ifs.precision(numeric_limits<double>::digits10 + 1);
	    vector < pose > vPoses;
	    int id;
	    double x, y, z,q_x,q_y,q_z,q_w;//输入的顶点的姿态数值  
	    pose tmpPose;
	    int id1, id2;
	    double measx, measy,measz, measq_x,measq_y,measq_z,measq_w;//边的测量值
	    double cov[21]={0};//边的协方差
	    edge tmpEdge;
	    string tmpStr;
	    string strTmp;
	    int prev_id=-1;
	    while (getline(ifs, tmpStr)) 
	    {
		    stringstream ss;
		    ss << tmpStr;
		    ss >> strTmp;
		    if (strTmp == "VERTEX_SE3:QUAT") 
		    {
			    ss >> id >> x >> y >> z>> q_x>>q_y>>q_z>>q_w;
			    tmpPose.id = id;
			    tmpPose.t = Eigen::Vector3d(x,y,z);
			    tmpPose.q = Eigen::Quaterniond(q_w,q_x,q_y,q_z);
			    poses.push_back(tmpPose);
			    if(id-prev_id!=1)
			    {
			      mistake = -1;
			      cout<<"姿态顺序错误!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			      break;
			    }
			    else
			    {
			       prev_id = id;
			    }
		    } 
		    else if (strTmp == "EDGE_SE3:QUAT") 
		    {
			    ss >> id1 >> id2 >> measx >> measy >> measz >> measq_x >> measq_y >> measq_z>> measq_w >> cov[0] >> cov[1]>>cov[2]>>cov[3]>>cov[4]>>cov[5]>>cov[6]>>cov[7]>>cov[8]>>cov[9]>>cov[10]>>cov[11]>>cov[12]>>cov[13]>>cov[14]>>cov[15]>>cov[16]>>cov[17]>>cov[18]>>cov[19]>>cov[20];
			    if(id1<id2)//这里保证id1一定小于id2
			    {
			      tmpEdge.id1 = id1;
			      tmpEdge.id2 = id2;

			      tmpEdge.delta_t = Eigen::Vector3d(measx,measy,measz);
			      tmpEdge.delta_q = Eigen::Quaterniond(measq_w,measq_x,measq_y,measq_z);
			    }
			    else
			    {
			       tmpEdge.id1 = id2;
			       tmpEdge.id2 = id1;
			       Eigen::Quaterniond q =  Eigen::Quaterniond(measq_w,measq_x,measq_y,measq_z);
			       Eigen::Matrix3d RT = q.toRotationMatrix().transpose();
			       tmpEdge.delta_q = RT;
			       tmpEdge.delta_t = -RT*Eigen::Vector3d(measx,measy,measz);
			     }
			    
			    
			    Eigen::Matrix<double,6,6> Cov;//输入的是协方差矩阵
			    
			    Cov(0,0) = cov[0];
			    
			    Cov(0,1) = cov[1];
			    Cov(1,0) = cov[1];
			    
			    Cov(0,2) = cov[2];
			    Cov(2,0) = cov[2];
			    
			    Cov(0,3) = cov[3];
			    Cov(3,0) = cov[3];
			    
			    Cov(0,4) = cov[4];
			    Cov(4,0) = cov[4];
			    
			    Cov(0,5) = cov[5];
			    Cov(5,0) = cov[5];
			    
			    Cov(1,1) = cov[6];
			    
			    Cov(1,2) = cov[7];
			    Cov(2,1) = cov[7];
			    
			    Cov(1,3) = cov[8];
			    Cov(3,1) = cov[8];
			    
			    Cov(1,4) = cov[9];
			    Cov(4,1) = cov[9];
			    
			    Cov(1,5) = cov[10];
			    Cov(5,1) = cov[10];
			    
			    Cov(2,2) = cov[11];
			    
			    Cov(2,3) = cov[12];
			    Cov(3,2) = cov[12];
			    
			    Cov(2,4) = cov[13];
			    Cov(4,2) = cov[13];
			    
			    Cov(2,5) = cov[14];
			    Cov(5,2) = cov[14];
			    
			    Cov(3,3) = cov[15];
			    
			    Cov(3,4) = cov[16];
			    Cov(4,3) = cov[16];
			    
			    Cov(3,5) = cov[17];
			    Cov(5,3) = cov[17];
			    
			    Cov(4,4) = cov[18];
			    
			    Cov(4,5) = cov[19];
			    Cov(5,4) = cov[19];
			    
			    Cov(5,5) = cov[20];
			    
			    tmpEdge.cov = Cov;
			    vEdges.push_back(tmpEdge);
		    }
	    }
	    ifs.close();
	    cout<<"文件读取完毕"<<endl;
	    return mistake;
	}
	
	//将优化的结果输出到g2o文件中 
	void write_G2O_file(string filename,vector<edge> edges,vector<pose> poses)
	{
	  ofstream ofs;
	  ofs.open(filename);
	  ofs.precision(numeric_limits<double>::digits10 + 1);//设置输出的精度
	  //先输出顶点
	  for (unsigned int idx = 0; idx < poses.size(); idx++) 
	  {
	      Eigen::Vector3d t =poses[idx].t;
	      Eigen::Quaterniond q = poses[idx].q;
	      ofs << "VERTEX_SE3:QUAT" << "\t" <<idx<<"\t"<< t[0]<< "\t"<<t[1]<<"\t"<<t[2]<<"\t"<<q.x()<<"\t"<<q.y()<<"\t"<<q.z()<<"\t"<<q.w() << "\r\n";
	  }
	  //再输出边
	  for(int idx=0;idx<edges.size();idx++)
	  {
	    int id1 = edges[idx].id1;
	    int id2 = edges[idx].id2;
	    Eigen::Vector3d delta_t = edges[idx].delta_t;
	    Eigen::Quaterniond delta_q = edges[idx].delta_q;
	    Eigen::Matrix<double,6,6> cov = edges[idx].cov;//输入的协方差矩阵
	    ofs << "EDGE_SE3:QUAT" << "\t" <<id1<<"\t"<<id2<<"\t"<< delta_t[0]<< "\t"<<delta_t[1]<<"\t"<<delta_t[2]<<"\t"<<delta_q.x()<<"\t"<<delta_q.y()<<"\t"<<delta_q.z()<<"\t"<<delta_q.w()
	                           <<"\t"<<cov(0,0)<<"\t"<<cov(0,1)<<"\t"<<cov(0,2)<<"\t"<<cov(0,3)<<"\t"<<cov(0,4)<<"\t"<<cov(0,5)<<"\t"<<cov(1,1)<<"\t"<<cov(1,2)<<"\t"<<cov(1,3)<<"\t"<<cov(1,4)<<"\t"<<cov(1,5)<<"\t"<<cov(2,2)<<"\t"<<cov(2,3)<<"\t"<<cov(2,4)<<"\t"<<cov(2,5)<<"\t"<<cov(3,3)<<"\t"<<cov(3,4)<<"\t"<<cov(3,5)<<"\t"<<cov(4,4)<<"\t"<<cov(4,5)<<"\t"<<cov(5,5)<<"\r\n";
	  }
	  ofs.close();
	}
	
	//设置输出的误差大小
	void write_error(string filename,ERRORS error_)
	{
	  ofstream ofs;
	  ofs.open(filename);
	  ofs.precision(numeric_limits<double>::digits10 + 1);//设置输出的精度
	  ofs<<"优化之前的绝对误差： 位置绝对误差 = "<<error_.i_sum_t<<" 姿态绝对误差 = "<<error_.i_sum_R<<endl;
	  ofs<<"优化之后的绝对误差： 位置绝对误差 = "<<error_.opt_sum_t<<" 姿态绝对误差 = "<<error_.opt_sum_R<<endl;
	  ofs<<"优化之前的相对误差 ： "<<"位置相对误差 = "<<error_.i_avg_t<<" 姿态相对误差 = "<<error_.i_avg_R<<endl;
	  ofs<<"优化之后的相对误差 ： "<<"位置相对误差 = "<<error_.opt_avg_t<<" 姿态相对误差 = "<<error_.opt_avg_R<<endl;
	  ofs.close();
	}
	
	//删除文件目录中的所有文件
	void Delete_path(const char* path)
	{  
	  DIR *pDir = NULL;  
	  struct dirent *dmsg;  
	  char szFileName[128];  
	  char szFolderName[128];  
	
	  strcpy(szFolderName, path);  
	  strcat(szFolderName, "/%s");  
	  if ((pDir = opendir(path)) != NULL)  
	  {  
	      // 遍历目录并删除文件  
	      while ((dmsg = readdir(pDir)) != NULL)  
	      {  
		  if (strcmp(dmsg->d_name, ".") != 0 && strcmp(dmsg->d_name, "..") != 0)  
		  {  
		      sprintf(szFileName, szFolderName, dmsg->d_name);  
		      string tmp = szFileName;  
		      //如果是文件夹，名称中不包含"."  
		      if (tmp.find(".") == -1){  
			  Delete_path(szFileName);  
		      }  
		      remove(szFileName);  
		  }  
	      }
	  }  
	
	  if (pDir != NULL)  
	  {  
	      closedir(pDir);  
	  }  
	  cout<<"delete "<<path<<"all files"<<endl;
	}  
	//为subset文件夹生成文件夹
	void Create_Folders(string name,bool create = true)
	{
	    string path = "/home/fyy/Documents/my_code/posegraph/build/subset_result/";
	    string path_error,path_pose;
	    path  = path+name;
	    path_error = path+"/Error/";
	    path_pose = path+"/Pose/";
	    mkdir(path.c_str(),0777);
	    if(create == true)
	    {
	      mkdir(path_error.c_str(),0777);
	      mkdir(path_pose.c_str(),0777);
	    }
	}
	
	void write_TUM_file(string path,vector<pose> poses,string filename)
	{
	  //首先读取path路径中的文件 为了提取时间戳
	  ifstream f_in;
	  f_in.open(path.c_str());
	  if (!f_in.is_open())
	  {
		  cout << "Can't open file " << path << endl;
		  return ;
	  }
	  string content;
	  vector<string> timestamps;
	  while (getline(f_in, content)) 
	  {
	    vector<string> messages;
	    messages = read_format(content," ");
	    timestamps.push_back(messages[0]);
	    //cout<<messages[0]<<endl;
	  }
	  f_in.close();
	  
	  //2.输出结果
	  ofstream ofs;
	  string path_out = path.substr(0,path.find_last_of("/")+1)+filename;
	  cout<<path_out<<endl;
	  ofs.open(path_out);
	  ofs.precision(numeric_limits<double>::digits10 + 1);//设置输出的精度
	  for(int i=0;i<poses.size();i++)
	  {
	    Eigen::Vector3d t = poses[i].t;
	    Eigen::Quaterniond q = poses[i].q;
	    ofs<<timestamps[i]<<" "<<t[0]<<" "<<t[1]<<" "<<t[2]<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
	  }
	  ofs.close();
	}
}
#endif