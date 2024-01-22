#ifndef _FILEIO_
#define _FILEIO_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
//用于删除目录
#include <dirent.h>  


using namespace std;
//这个文件的作用是读取文件到程序中
namespace TLS
{       
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
	void readnoise(string path,string& initialvalue,int& iterat,vector<double>&  noiselevel_,string& noisekind)
	{
	  
	  ifstream f_noise;
	   f_noise.open((path+"Noise.txt").c_str());
	   
	   string s_iterat;
	   getline(f_noise, s_iterat);
	   iterat = atoi(s_iterat.c_str());
	 
	   getline(f_noise, initialvalue);
	   
	   getline(f_noise, noisekind);
	   string s_noise;
	   for(int i=0;i<3;i++)
	   {
	      getline(f_noise, s_noise);
	      noiselevel_.push_back( atof(s_noise.c_str()) );
	   }	  
	   f_noise.close();
	}
}
#endif
