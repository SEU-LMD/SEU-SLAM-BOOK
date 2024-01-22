#ifndef _TicToc_
#define _TicToc_
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
using namespace std;
//这个文件的作用是读取文件到程序中
     
      class TicToc
      {
	public:
	  TicToc()
	  {
	    tic();
	  }
	  void tic()
	  {
	    start = std::chrono::system_clock::now();
	  }
	  double toc()
	  {
	    end = std::chrono::system_clock::now();
	    std::chrono::duration<double> seconds = end-start;
	    return seconds.count()*1000;//单位是毫秒
	  }
	private:
	  std::chrono::time_point<std::chrono::system_clock> start,end;
      };

#endif
