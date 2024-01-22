#ifndef LIBDEALSTRING_H_
#define LIBDEALSTRING_H_
#include <string>
#include <string>
#include <boost/iterator/iterator_concepts.hpp>
#include <vector>
#include<iostream>
using namespace std;
//content是输入的需要处理的字符串
//start是截取字符串的开始,如果是空字符则认为从字符串的开始截取
//end是截取字符串的结束,如果是空字符则认为是字符串结束停止截取
//spos是截取start字符串后向后移动几个字符开始截取
//epos是截取end字符串后向后移动几个字符结束截取
//例如:substr_start_end("loveSEU","o","E") = "oveS"
//substr_start_end("loveSEU","",E) = "loveS";
//substr_start_end("loveSEU,"o","E",1,0) = "veS"
string substr_start_end(const string content,const string start,string end="",int spos=0,int epos=0)
{
  int n_end,n_start;
  string result;
  if(start=="")
    n_start = 0;
  else
    n_start=content.find(start)+spos;

  if(end=="")
    n_end = content.size();
  else
    n_end=content.find(end)+epos;

  result = content.substr(n_start,n_end-n_start);
  return result;
}

string substr_start_from(const std::string content,const std::string from,const std::string start,std::string end,int spos=0,int epos=0)
{
  int n_end,n_start,n_from;
  string result;
  if(start=="")
    n_start = 0;
  else
    n_from = content.find(from);
    n_start=content.find(start,n_from)+spos;

  if(end=="")
    n_end = content.size();
  else
    n_end=content.find(end)+epos;

  result = content.substr(n_start,n_end-n_start);
  return result;
  
}

string substr_start_from(const std::string content,const int from,const std::string start,std::string end,int spos=0,int epos=0)
{
  int n_end,n_start;
  string result;
  if(start=="")
    n_start = 0;
  else
    n_start=content.find(start,from)+spos;

  if(end=="")
    n_end = content.size();
  else
    n_end=content.find(end)+epos;

  result = content.substr(n_start,n_end-n_start);
  return result;
  
}

//这个函数能够读取一行中用空格分隔的数字
//例如有一串字符"F:1 2 3 4",想要把1 2 3 4提取出来,这时就需要输入如下函数:
//read_sapce_string("F:1 2 3 4"," ",1)
vector<float> read_sapce_string(const std::string content,string s,int start)
{
  vector<float> result;
  float d_num = 0;
  int l = content.length();
  string line = content.substr(start+1,l-start);//左闭右开函数
  while(line.find(s)!=std::string::npos)
  {
    int locate = line.find(s);
    string s_num= line.substr(0,locate);
    d_num = std :: stof(s_num);
    result.push_back(d_num);
    l = line.length();
    line = line.substr(locate+1,l-locate-1);
  }
  d_num = std :: stof(line);
  result.push_back(d_num);
  return result;
}

vector<double> read_sapce_string(const std::string content,string s,int start,int end)
{
  vector<double> result;
  double d_num = 0;
  int l=0;
  string line = content.substr(start,end-start);//左闭右开函数
  //cout<<line<<endl;
  while(line.find(s)!=std::string::npos)
  {
    int locate = line.find(s);
    string s_num= line.substr(0,locate);
    d_num = atof(s_num.c_str());
    result.push_back(d_num);
    l = line.length();
    line = line.substr(locate+1,l-locate-1);
  }
  d_num = atof(line.c_str());
  //cout<<d_num<<endl;
  result.push_back(d_num);
  return result;
}

//定位字符s重复出现次数的在content中的位置
int locate_repeat(const std::string content,string s,int num)
{
  int locate=0;
  int glboal_locate = 0;
  int times = num;
  string s_content = content;
  while(times!=0)
  {
    locate = s_content.find(s);
    glboal_locate = glboal_locate + locate;
    //cout<<glboal_locate<<endl;
    times--;
    s_content  =  s_content.substr(locate+1, s_content.length()-locate-1);
    //cout<<s_content<<endl;
  }
  return glboal_locate+num-1;
}

//将字符分隔开假设我们输入的是 "{1,1,1,} {2,3,4} {4,6,7}"
//第2个参数是{ 第三个参数是} 第四个参数是" "
vector<string> read_format(string content,string s)
{
  int l=0;
  vector<string> result;
  while(content.find(s)!=content.npos)
  {
    int locate = content.find(s);
    string s_num= content.substr(0,locate);
    l = content.length();
    content = content.substr(locate+1,l-locate-1);
    result.push_back(s_num);
  }
  result.push_back(content);
  return result;
}


#endif
