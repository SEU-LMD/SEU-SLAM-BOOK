#ifndef _ICPTLS_
#define _ICPTLS_
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> //为了求特征值

#include <iostream>
#include "sophus/so3.h"
#include "sophus/se3.h"
#include <../../COP-SLAM_v0.1/inc/Eigen/src/Geometry/Translation.h>

#include "ceres/ceres.h"//ceres的头文件 用于自动求导
#include "ceres/rotation.h"
#include "Tic_Toc.h"
using namespace std;
namespace TLS
{
  //这个函数是用于求Rp对轴角自动求导的结果
  //返回的是对轴角求倒数后的雅克比
  //rotated_point是旋转后的点坐标
  Eigen::Matrix3d Cere_Autodiff(Eigen::Vector3d rotation_vector,Eigen::Vector3d p)
  {
    ceres::Jet<double,6> jets_rotationvector[3];//三个变量
    for(int i=0;i<3;i++)
    {
      jets_rotationvector[i].a = rotation_vector[i];
      jets_rotationvector[i].v.setZero();
      jets_rotationvector[i].v[i] = 1.0;
    }
    ceres::Jet<double,6> jets_point[3];//三个变量
    for(int i=0;i<3;i++)
    {
      jets_point[i].a = p[i];
      jets_point[i].v.setZero();
      jets_point[i].v[i+3] = 1.0;
    }
    ceres::Jet<double,6> result[3];
    ceres::AngleAxisRotatePoint<ceres::Jet<double,6>>(jets_rotationvector,jets_point,result);
    Eigen::Vector3d rotated_point(result[0].a,result[1].a,result[2].a);
    //cout<<"ceres rotated_point = "<<endl<<rotated_point<<endl;
    Eigen::AngleAxisd angeaxis(rotation_vector.norm(),rotation_vector/rotation_vector.norm());
    Eigen::Matrix3d R = angeaxis.toRotationMatrix();
    //cout<<"R = "<<endl<<R<<endl;
    //cout<<"轴角表示 = "<<endl<<rotation_vector<<endl;
    //cout<<"p = "<<endl<<p<<endl;
    
    //cout<<"rotated_point = "<<endl<<R*p<<endl;
    Eigen::Matrix3d jacobian_R;
    jacobian_R<<result[0].v[0],result[0].v[1],result[0].v[2],
	      result[1].v[0],result[1].v[1],result[1].v[2],
	      result[2].v[0],result[2].v[1],result[2].v[2];
    //cout<<"jacobian_R = "<<endl<<jacobian_R<<endl;
    Eigen::Matrix3d jacobian_point;
    jacobian_point<<result[0].v[3],result[0].v[4],result[0].v[5],
		    result[1].v[3],result[1].v[4],result[1].v[5],
		    result[2].v[3],result[2].v[4],result[2].v[5];
    //cout<<"jacobian_point = "<<endl<<jacobian_point<<endl;
    return jacobian_R;
  }
  
  //这个函数是专门用来计算kroncker积的
Eigen::MatrixXd kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  const long rA = A.rows(), cA = A.cols(),
             rB = B.rows(), cB = B.cols();
  Eigen::MatrixXd K(rA*rB, cA*cB);
  for (long i = 0; i < rA; ++i) {
    for (long j = 0; j < cA; ++j) {
      K.block(i*rB, j*cB, rB, cB) = A(i,j)*B;
    }
  }
  return std::move(K);
}
//将矩阵按列排列成为一个列矩阵
Eigen::VectorXd Vect(Eigen::MatrixXd A)
{
  Eigen::VectorXd vecB(Eigen::Map<Eigen::VectorXd>(A.data(),A.size()));
  return vecB;
}
//将向量变成矩阵
Eigen::MatrixXd inv_Vect(Eigen::VectorXd A,int n)
{
  int ro = A.size();
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n,3);
  for(int i=0;i<3;i++)
  {
    Eigen::VectorXd data_col = Eigen::VectorXd::Zero(n,1);
    for(int j=0;j<n;j++)
    {
      data_col[j] = A(j+i*n);
    }
    B.col(i) = data_col;
  }
  return B;
}
//这个函数的作用是将e来更新VA和VY
void updateVAVY(Eigen::VectorXd e,Eigen::MatrixXd &VA, Eigen::MatrixXd &VY)
{
  int n  =e.size()/6;
  Eigen::VectorXd eA  = Eigen::VectorXd::Zero(3*n);
  Eigen::VectorXd eY  = Eigen::VectorXd::Zero(3*n);
  for(int i=0;i<3*n;i++)
  {
    eA(i) = e(i);
    eY(i) = e(i+3*n);
  }
  VA = inv_Vect(eA,n);
  VY = inv_Vect(eY,n);
}
//用于保存每一步TLS的结果
struct Result
{
  int step;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double equnorm;
  double error;
};
//下面这个函数是使用整体最小二乘进行计算
//R和t是通过GMM模型计算得到的位姿变化
//pgps是全站仪的坐标点
//pslam是slam坐标系下的坐标点
//weight_slam和weight_gps是对应的优化权重，正常情况下应该是slam点的权重要大一些，且slam点距离原点越远其权重应该也越大。
//最终返回的值是优化之后的R和t还有最终的轴角表示
//计算得到的是slam坐标系到gps坐标系的变化
//返回的是ePe 参差
vector<Result> TLS_ICP(Eigen::Matrix3d &R,Eigen::Vector3d &t,vector<Eigen::Vector3d> pslam,vector<Eigen::Vector3d> pgps,vector<double > weight_slam,vector<double> weight_gps,int step=200,string s="AngleAxis")
{
   vector<Result> results;
   cout<<"总体最小二乘:"<<s<<endl;
   cout<<"优化之前的R = "<<endl;
   cout<<R<<endl;
   cout<<"优化之前的t = "<<endl;
   cout<<t<<endl;
   //以下注释中的n表示构造的方程个数
   int n = pslam.size();
   Eigen::MatrixXd VA = Eigen::MatrixXd::Zero(n,3);
   Eigen::MatrixXd VY = Eigen::MatrixXd::Zero(n,3);
   //其中[eA eY] 对应论文中的e
   Eigen::VectorXd eA =  Eigen::VectorXd::Zero(3*n);//对应论文中的vec(VA)
   Eigen::VectorXd eY =  Eigen::VectorXd::Zero(3*n);//对应论文中的vec(Vy)
   Eigen::VectorXd e =   Eigen::VectorXd::Zero(6*n);//维度是6n*1
   Eigen::VectorXd equ = Eigen::VectorXd::Zero(3*n);//限制方程的结果
   Eigen::VectorXd w =   Eigen::VectorXd::Zero(3*n);
   Eigen::VectorXd lam = Eigen::VectorXd::Zero(3*n);//维度是3n*1,对应论文中的拉格朗日乘子
   Eigen::MatrixXd ln =  Eigen::MatrixXd::Ones(n,1);//对应论文中的ln
   Eigen::MatrixXd A =   Eigen::MatrixXd::Zero(n,3);//由slam坐标更新
   Eigen::MatrixXd Y =   Eigen::MatrixXd::Zero(n,3);////由gps坐标更新
   //使用slam的坐标对A矩阵进行赋值
   for(int i=0;i<A.rows();i++)
   {
     A(i,0) = pslam[i](0);
     A(i,1) = pslam[i](1);
     A(i,2) = pslam[i](2);
   }
   //cout<<A<<endl;
   //使用gpsm的坐标对Y矩阵进行赋值
   for(int i=0;i<Y.rows();i++)
   {
     Y(i,0) = pgps[i](0);
     Y(i,1) = pgps[i](1);
     Y(i,2) = pgps[i](2);
   }
   //cout<<Y<<endl;
   Eigen::MatrixXd P = Eigen::MatrixXd::Identity(6*n,6*n);
   if((weight_gps.size()==n)&&(weight_slam.size()==n))
   {
     /*这里我们加的权重是按照slam 和 gps分类的
      for(int i=0;i<n;i++)
      {
	P(i,i) = weight_slam[i];
	P(i+n,i+n) = weight_slam[i];
	P(i+2*n,i+2*n) = weight_slam[i];
	
	P(i+3*n,i+3*n) = weight_gps[i];
	P(i+4*n,i+4*n) = weight_gps[i];
	P(i+5*n,i+5*n) = weight_gps[i];
      }
      cout<<"更新权重成功!!!"<<endl;
      */
     //这里我们加的权重是按照x y z
     for(int i=0;i<3;i++)
     {
       for(int j=0;j<n;j++)
       {
	 P(n*i+j,n*i+j) = weight_slam[i];
	 P(n*i+j+3*n,n*i+j+3*n) = weight_slam[i];
      }
    }
   }
   //cout<<P<<endl;
   Eigen::MatrixXd N = Eigen::MatrixXd::Zero(3*n,3*n);
   double error_new=0;
   double error_old = 0;
   for(int itestep=0;itestep<step;itestep++)//暂且我们人为控制迭代的步骤,最终的迭代步骤结束是由eT*Pe小于某个阈值而确定的
   {
        TLS::TicToc usetime;
	//首先计算得到B=限制方程对误差的导数,3n*6n
	//B0=[B1 B2],B1是限制方程对slam误差的导数 3n*3n
	//		 B2是限制方程对gps误差的导数 3n*3n
	//A0=[A1 A2],A1是限制方程对李代数/欧拉角误差的导数 3n*3
	//	         A2是限制方程对位置的导数 3n*3
	Eigen::MatrixXd B0 = Eigen::MatrixXd::Zero(3*n,6*n);//维度是3n*6n
	Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(3*n,6);//维度是3n*6
	Eigen::MatrixXd B1= kronecker(R,Eigen::MatrixXd::Identity(n,n));
	Eigen::MatrixXd B2 = -kronecker(Eigen::Matrix3d::Identity(),Eigen::MatrixXd::Identity(n,n));
	//得到B:
	for(int i=0;i<B1.cols();i++)
	{
	  B0.col(i) = B1.col(i);
	  B0.col(i+3*n) = B2.col(i);
	}
	//cout<<B0<<endl;
	Eigen::MatrixXd A2 = kronecker(Eigen::Matrix3d::Identity(),ln);//如果是李代数的话
	Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(3*n,3);
	Eigen::Matrix<double,6,1> parameter;
	Eigen::Matrix<double,6,1> parameter_new;
	if(s=="Euler")//使用欧拉角计算A1
	{
		  //A1=[vect(A11) vect(A12) vect(A13)],A11=对x轴角度的导数 n*3=(A+VA)*Rtx
		  //                                   A12=对y轴角度的导数 n*3=(A+VA)*Rty
		  //                                   A13=对z轴角度的导数 n*3=(A+VA)*Rtz
		  //Rtx=R转置对x轴角度求导  Rty=R转置对y轴角度求导  Rtz=R转置对z轴角度求导  
		  //先将R转换为欧拉角
		  Eigen::Vector3d eulerangle = R.eulerAngles(2,1,0);
		  
		  Eigen::Matrix3d R_euler_z = Eigen::AngleAxisd(eulerangle[0], Eigen::Vector3d::UnitZ()).toRotationMatrix();
		  Eigen::Matrix3d R_euler_y = Eigen::AngleAxisd(eulerangle[1], Eigen::Vector3d::UnitY()).toRotationMatrix();
		  Eigen::Matrix3d R_euler_x = Eigen::AngleAxisd(eulerangle[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
		  Eigen::Matrix3d R_euler_z_own,R_euler_y_own,R_euler_x_own;
		  R_euler_z_own<<cos(-eulerangle[0]),-sin(-eulerangle[0]),0,
				 sin(-eulerangle[0]),cos(-eulerangle[0]),0,
				 0,0,1;
		  R_euler_y_own<<cos(-eulerangle[1]),0,sin(-eulerangle[1]),
				0,1,0,
				-sin(-eulerangle[1]),0,cos(-eulerangle[1]);
		 R_euler_x_own<<1,0,0,
				0,cos(-eulerangle[2]),-sin(-eulerangle[2]),
				0,sin(-eulerangle[2]),cos(-eulerangle[2]);		  
					   
                  
		  /*cout<<"R_euler_z"<<endl<<R_euler_z<<endl;
		  cout<<"R_euler_z_own"<<endl<<R_euler_z_own<<endl;
		  cout<<"R_euler_y"<<endl<<R_euler_y<<endl;
		  cout<<"R_euler_y_own"<<endl<<R_euler_y_own<<endl;
		  cout<<"R_euler_x"<<endl<<R_euler_x<<endl;
		  cout<<"R_euler_x_own"<<endl<<R_euler_x_own<<endl;
		  cout<<"RT ="<<endl<<R.transpose()<<endl;
		  cout<<"R_euler="<<endl<<R_euler_x_own*R_euler_y_own*R_euler_z_own<<endl;*/
		  //cout<<eulerangle<<endl;
		  double zangle = eulerangle(0);//对应的yaw  z轴旋转角度
		  double yangle = eulerangle(1);//对应的pitch y轴旋转角度
		  double xangle = eulerangle(2);//对应的roll x轴旋转角度
		  //cout<<"z轴角度(度)="<<(zangle/3.1415926)*180.0<<endl;
		  //cout<<"y轴角度"<<(yangle/3.1415926)*180.0<<endl;
		  //cout<<"x轴角度"<<(xangle/3.1415926)*180.0<<endl;
		  Eigen::Matrix<double,3,3> Rtx,dRtx;
		  Eigen::Matrix<double,3,3> Rty,dRty;
		  Eigen::Matrix<double,3,3> Rtz,dRtz;
		  Rtx<< 1,0,0,
			0,cos(xangle),sin(xangle),
			0,-sin(xangle),cos(xangle);
		  //cout<<Rtx<<endl;
		  Rty<< cos(yangle),0,-sin(yangle),
			0,1,0,
			sin(yangle),0,cos(yangle);
		  Rtz<< cos(zangle),sin(zangle),0,
			-sin(zangle),cos(zangle),0,
			0,0,1;
		  dRtx << 0,0,0,
			0,-sin(xangle),cos(xangle),
			0,-cos(xangle),-sin(xangle);
		  dRty << -sin(yangle),0,-cos(yangle),
			  0,0,0,
			  cos(yangle),0,-sin(yangle);
		  dRtz << -sin(zangle),cos(zangle),0,
			  -cos(zangle),-sin(zangle),0,
			  0,0,0;
		  A1.col(0) = Vect((A+VA)*dRtx*Rty*Rtz);//默认返回的是VectorXd类型
		  A1.col(1) = Vect((A+VA)*Rtx*dRty*Rtz);
		  A1.col(2) = Vect((A+VA)*Rtx*Rty*dRtz);
		  //和A2联合起来构造出A
		  for(int i=0;i<A1.cols();i++)
		  {
		    A0.col(i) = A1.col(i);
		    A0.col(i+3)=A2.col(i);
		  }
		  
		  parameter(0) = xangle;
		  parameter(1) = yangle;
		  parameter(2) = zangle;
		  //下面是计算得到要估计的状态
		  for(int i=0;i<3;i++)
		  {
		    //parameter(i) = eulerangle(i);
		    parameter(i+3) = t(i);
		  }
		  //下面是计算w=B*e+限制方程的取值
		  //首先计算得到e
		  eA = Vect(VA);//维度是 3n*1
		  eY = Vect(VY);//维度是 3n*1
		  for(int i=0;i<eA.size();i++)
		  {
		    e(i) = eA(i);
		    e(i+3*n) = eY(i);
		  }
		  //然后计算得到限制方程的取值
		  //cout<<kronecker(t,ln)<<endl;
		  //cout<<Vect((A+VA)*R.transpose())<<endl;
		  //cout<<Vect(Y+VY)<<endl;
		  equ = Vect((A+VA)*R.transpose())+kronecker(t,ln)-Vect(Y+VY);
		  w = equ-B0*e;
		  
		  //然后计算N = BP.inverse*B.transpose
		  N = B0*P.inverse()*B0.transpose();
		  
		  //最后得到更新的结果-------------------------------------
		  //需要查验N、A0T*N-1*A0，B0*B0T的特征值
		  Eigen::EigenSolver<Eigen::MatrixXd> es1( N );
		  //cout<<"N的特征值 = "<<endl<<es1.eigenvalues()<<endl;
		  Eigen::EigenSolver<Eigen::MatrixXd> es2( A0.transpose()*N.inverse()*A0 );
		  //cout<<"A0(T)*N(-1)*A0的特征值 = "<<endl<<es2.eigenvalues()<<endl;
		  Eigen::EigenSolver<Eigen::MatrixXd> es3( B0*P.inverse()*B0.transpose() );
		  //cout<<"B0*P(-1)*B0(T)的特征值 = "<<endl<<es3.eigenvalues()<<endl;
		  
		  
		  parameter_new = parameter-(A0.transpose()*N.inverse()*A0).inverse()*A0.transpose()*N.inverse()*w ;
		  lam = (B0*P.inverse()*B0.transpose()).inverse()*(A0*(parameter_new-parameter)+w);
		  e = -(P.inverse()*B0.transpose()*lam);
		  //然后使用e去更新VA和VY
		  updateVAVY(e,VA,VY);
		  parameter = parameter_new;//使用新的状态更新旧的状态
		  R = Eigen::AngleAxisd(parameter(2),Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameter(1),Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameter(0),Eigen::Vector3d::UnitX());
		  t(0) = parameter(3);t(1) = parameter(4);t(2) = parameter(5);
		  error_new = e.transpose()*P*e;
		  cout<<"第"<<itestep+1<<"次迭代误差"<<error_new<<" "<<"等式的模="<<equ.norm()<<" 误差大小="<<error_new<<endl;
		  error_old = error_new;
		  
		  Result result;
		  result.step = itestep;
		  result.R = R;
		  result.t = t;
		  result.error = error_new;
		  result.equnorm = equ.norm();
		  results.push_back(result);
	}
	else if(s=="AngleAxis")//使用轴角来计算
	{
	    //下面是计算得到要估计的状态
	    Sophus::SO3 so3(R);
	    Eigen::Vector3d angle_axis = so3.log();
	    Eigen::MatrixXd A11 = Eigen::MatrixXd::Zero(n,3);
	    Eigen::MatrixXd A12 = Eigen::MatrixXd::Zero(n,3);
	    Eigen::MatrixXd A13 = Eigen::MatrixXd::Zero(n,3);
	    //使用e更新
	    for(int i=0;i<n;i++)
	    {
	      Eigen::Vector3d delta(e[i],e[i+n],e[i+2*n]);
	      Eigen::Vector3d point(delta[0]+pslam[i][0],delta[1]+pslam[i][1],delta[2]+pslam[i][2]);
	      Eigen::Matrix3d jacobian = Cere_Autodiff(angle_axis,point);
	      A11(i,0) =jacobian(0,0) ; A11(i,1) =jacobian(1,0) ;A11(i,2) =jacobian(2,0) ;
	      A12(i,0) =jacobian(0,1) ; A12(i,1) =jacobian(1,1) ;A12(i,2) =jacobian(2,1) ;
	      A13(i,0) =jacobian(0,2) ; A13(i,1) =jacobian(1,2) ;A13(i,2) =jacobian(2,2) ;
	   }
	   A1.col(0) = Vect(A11);
	   A1.col(1) = Vect(A12);
	   A1.col(2) = Vect(A13);
	   //和A2联合起来构造出A
	   for(int i=0;i<A1.cols();i++)
	   {
	      A0.col(i) = A1.col(i);
	      A0.col(i+3)=A2.col(i);
	   }
	   
	    //下面是计算得到要估计的状态
	    for(int i=0;i<3;i++)
	    {
	      parameter(i) = angle_axis(i);
	      parameter(i+3) = t(i);
	    }
	    
	     //下面是计算w=B*e+限制方程的取值
	    //首先计算得到e
	    eA = Vect(VA);//维度是 3n*1
	    eY = Vect(VY);//维度是 3n*1
	    for(int i=0;i<eA.size();i++)
	    {
	      e(i) = eA(i);
	      e(i+3*n) = eY(i);
	    }
	    //然后计算得到限制方程的取值
	    //cout<<kronecker(t,ln)<<endl;
	    //cout<<Vect((A+VA)*R.transpose())<<endl;
	    //cout<<Vect(Y+VY)<<endl;
	    equ = Vect((A+VA)*R.transpose())+kronecker(t,ln)-Vect(Y+VY);
	    w = equ-B0*e;
	    
	    //然后计算N = BP.inverse*B.transpose
	    N = B0*P.inverse()*B0.transpose();
	    
	    //最后得到更新的结果-------------------------------------
	    //我们发现其实是可逆的没有任何问题但是残差就是会变大，难道是程序的问题？
	    //需要查验N、A0T*N-1*A0，B0*B0T的特征值
	    
	    Eigen::EigenSolver<Eigen::MatrixXd> es1( N );
	    Eigen::EigenSolver<Eigen::MatrixXd> es2( A0.transpose()*N.inverse()*A0 );
	    Eigen::EigenSolver<Eigen::MatrixXd> es3( B0*P.inverse()*B0.transpose() );
	    /*cout<<"N的特征值 = "<<endl<<es1.eigenvalues()<<endl;
	    cout<<"A0(T)*N(-1)*A0的特征值 = "<<endl<<es2.eigenvalues()<<endl;
	    cout<<"B0*P(-1)*B0(T)的特征值 = "<<endl<<es3.eigenvalues()<<endl;*/
	    Eigen::Matrix<double,6,1> zerovector;//声明这个变量的意义是将动态矩阵转换成固定矩阵
	    zerovector(0) = 0;zerovector(1) = 0;zerovector(2) = 0;zerovector(3) = 0;zerovector(4) = 0;zerovector(5) = 0;
	    Eigen::Matrix<double,6,1> update_parmeter = zerovector -(A0.transpose()*N.inverse()*A0).inverse()*A0.transpose()*N.inverse()*w;
	    Eigen::Vector3d update_t(update_parmeter(3),update_parmeter(4),update_parmeter(5));
	    Eigen::Vector3d original_t(parameter(3),parameter(4),parameter(5));
	    Eigen::Vector3d new_t = update_t+original_t;
	    
	    Eigen::Vector3d update_angleaxis(update_parmeter(0),update_parmeter(1),update_parmeter(2));
	    Sophus::SO3 update_R = Sophus::SO3::exp(update_angleaxis);
	    Eigen::Vector3d original_angleaxis(parameter(0),parameter(1),parameter(2));
	    Sophus::SO3 original_R = Sophus::SO3::exp(original_angleaxis);
	    Sophus::SO3 new_R = update_R * original_R;//一定要注意这里使用的是左乘
	    Eigen::Vector3d new_R_angleaxis = new_R.log();
	    
	    //更新最新状态
	    parameter_new(0) = new_R_angleaxis(0); parameter_new(1) = new_R_angleaxis(1); parameter_new(2) = new_R_angleaxis(2);
	    parameter_new(3) = new_t(0); parameter_new(4) = new_t(1); parameter_new(5) = new_t(2);
	    //parameter_new = parameter-(A0.transpose()*N.inverse()*A0).inverse()*A0.transpose()*N.inverse()*w ;
	    //cout<<"更新状态成功"<<endl;
	    lam = (B0*P.inverse()*B0.transpose()).inverse()*(A0*update_parmeter+w);
	    e = -(P.inverse()*B0.transpose()*lam);
	    //然后使用e去更新VA和VY
	    updateVAVY(e,VA,VY);
	    parameter = parameter_new;//使用新的状态更新旧的状态
	    //这里计算得到的R和t在下一个循环中会用到
	    R = new_R.matrix();
	    t = new_t;
	    
	    error_new = e.transpose()*P*e;
	    cout<<"第"<<itestep+1<<"次迭代误差"<<error_new<<" "<<"等式的模="<<equ.norm()<<" 误差大小="<<error_new<<endl;
	    error_old = error_new;
	    
	    Result result;
	    result.step = itestep;
	    result.R = R;
	    result.t = t;
	    result.error = error_new;
	    result.equnorm = equ.norm();
	    results.push_back(result);
	}
	else if(s=="Lie")//使用李群和李代数求导得到
	{
	   Sophus::SE3 lie_se3(R,t);
	   Eigen::Matrix<double,6,1> se3=lie_se3.log();//前三位表示位移，后三位表示旋转，我们这里需要颠倒一下顺序
	   //
	   for(int i=0;i<n;i++)
	    {
	      Eigen::Vector3d delta(e[i],e[i+n],e[i+2*n]);
	      Eigen::Vector3d point(delta[0]+pslam[i][0],delta[1]+pslam[i][1],delta[2]+pslam[i][2]);
	      Eigen::Vector3d rotated_point = R*point+t;
	      A0(i,3) = 0;                    A0(i,4) = rotated_point[2];     A0(i,5) = -rotated_point[1];    A0(i,0) = 1;    A0(i,1) = 0;    A0(i,2) = 0;
	      A0(i+n,3) = -rotated_point[2];  A0(i+n,4) = 0;                  A0(i+n,5) = rotated_point[0];   A0(i+n,0) = 0;  A0(i+n,1) = 1;  A0(i+n,2) = 0;
	      A0(i+2*n,3) = rotated_point[1]; A0(i+2*n,4) = -rotated_point[0];A0(i+2*n,5) = 0;                A0(i+2*n,0) = 0;A0(i+2*n,1) = 0;A0(i+2*n,2) = 1;
	    }
	    //下面是计算得到要估计的状态
	    for(int i=0;i<6;i++)
	    {
	      parameter(i) = se3(i); 
	    }
	     //<<下面是相同的部分---------------------------------------------------
	    //下面是计算w=B*e+限制方程的取值
	    //首先计算得到e
	    eA = Vect(VA);//维度是 3n*1
	    eY = Vect(VY);//维度是 3n*1
	    for(int i=0;i<eA.size();i++)
	    {
	      e(i) = eA(i);
	      e(i+3*n) = eY(i);
	    }
	    //然后计算得到限制方程的取值
	    //cout<<kronecker(t,ln)<<endl;
	    //cout<<Vect((A+VA)*R.transpose())<<endl;
	    //cout<<Vect(Y+VY)<<endl;
	    equ = Vect((A+VA)*R.transpose())+kronecker(t,ln)-Vect(Y+VY);
	    w = equ-B0*e;
	    
	    //然后计算N = BP.inverse*B.transpose
	    N = B0*P.inverse()*B0.transpose();
	    
	    //最后得到更新的结果-------------------------------------
	    //我们发现其实是可逆的没有任何问题但是残差就是会变大，难道是程序的问题？
	    //需要查验N、A0T*N-1*A0，B0*B0T的特征值
	    Eigen::EigenSolver<Eigen::MatrixXd> es1( N );
	    //cout<<"N的特征值 = "<<endl<<es1.eigenvalues()<<endl;
	    Eigen::EigenSolver<Eigen::MatrixXd> es2( A0.transpose()*N.inverse()*A0 );
	    //cout<<"A0(T)*N(-1)*A0的特征值 = "<<endl<<es2.eigenvalues()<<endl;
	    Eigen::EigenSolver<Eigen::MatrixXd> es3( B0*P.inverse()*B0.transpose() );
	    //cout<<"B0*P(-1)*B0(T)的特征值 = "<<endl<<es3.eigenvalues()<<endl;
	    
	    Eigen::Matrix<double,6,1> zerovector;//声明这个变量的意义是将动态矩阵转换成固定矩阵
	    zerovector(0) = 0;zerovector(1) = 0;zerovector(2) = 0;zerovector(3) = 0;zerovector(4) = 0;zerovector(5) = 0;
	    Eigen::Matrix<double,6,1> update_parmeter = zerovector-(A0.transpose()*N.inverse()*A0).inverse()*A0.transpose()*N.inverse()*w;
	    Sophus::SE3 se3_update = Sophus::SE3::exp(update_parmeter);
	    Sophus::SE3 se3_original = Sophus::SE3::exp(parameter);
	    Sophus::SE3 se3_new = se3_update*se3_original;
	    parameter_new = se3_new.log();
	    //parameter_new = parameter-(A0.transpose()*N.inverse()*A0).inverse()*A0.transpose()*N.inverse()*w ;
	    lam = (B0*P.inverse()*B0.transpose()).inverse()*(A0*update_parmeter+w);
	    e = -(P.inverse()*B0.transpose()*lam);
	    //然后使用e去更新VA和VY
	    updateVAVY(e,VA,VY);
	    parameter = parameter_new;//使用新的状态更新旧的状态
	    //----->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>以上是相同的部分
	    se3<<parameter[0],parameter[1],parameter[2],parameter[3],parameter[4],parameter[5];
	    t = (Sophus::SE3::exp(se3)).translation();
	    R = (Sophus::SE3::exp(se3)).rotation_matrix();
	    error_new = e.transpose()*P*e;
	    cout<<"第"<<itestep+1<<"次迭代误差"<<error_new<<" "<<"等式的模="<<equ.norm()<<" 误差大小="<<error_new<<endl;
	    //cout<<"e = "<<e<<endl;
	    error_old = error_new;  
	    
	    Result result;
	    result.step = itestep;
	    result.R = R;
	    result.t = t;
	    result.error = error_new;
	    result.equnorm = equ.norm();
	    results.push_back(result);
	}
	else
	{
	  cout<<"输入的模式错误！！！"<<endl;
	}
        
        cout<<"执行"<<s<<"一步所耗费的时间 = "<<usetime.toc()<<endl;
    
   }//大的for循环结束
   //优化结束显示优化之后的R和t
   cout<<"整体最小二乘得到的结果"<<endl;
   cout<<"R = "<<endl;
   cout<<R<<endl;
   cout<<"t = "<<endl;
   cout<<t<<endl;
   cout<<"VA = "<<endl<<VA<<endl;
   cout<<"VY = "<<endl<<VY<<endl;
   /*旋转矩阵转换至轴角表示
   Eigen::AngleAxisd axis(R);*/
   return results;
 }
}
#endif