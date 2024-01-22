#ifndef _CAMERACALIBRATE_
#define _CAMERACALIBRATE_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <iterator>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>//使用到了四元数数据结构

#include "Convert.h"
#include "Ceres.h"
using namespace cv;
using namespace std;
namespace CameraCalibration3D
{
///
    //根据输入的坐标我们使用pca算法得到新的坐标系统,返回的是已经经过变换的坐标
    vector<Eigen::Vector3d> choose_coordinate(vector<Eigen::Vector3d> orignal,Eigen::Matrix3d& R,Eigen::Vector3d& t)//得到的R和t是将输入的坐标变换到新的坐标系中的变换
    {
      Eigen::MatrixXd X(3,orignal.size());
      Eigen::Vector3d average;
      Eigen::Vector3d sum(0,0,0);
      for(int i=0;i<orignal.size();i++)
      {
	sum = sum + orignal[i];
      }
      average = sum/double(orignal.size());
      for(int i=0;i<orignal.size();i++)
      {
	Eigen::Vector3d a = orignal[i]-average;
	X(0,i) = a[0];
	X(1,i) = a[1];
	X(2,i) = a[2];
      }
      
      Eigen::MatrixXd XTX = X*X.transpose();
      Eigen::EigenSolver<Eigen::MatrixXd> es1( XTX );
      Eigen::MatrixXcd evecs = es1.eigenvectors();
      Eigen::MatrixXcd evals = es1.eigenvalues();
      //cout<<"特征值 = "<<endl;
      //cout<<evals<<endl;
      //cout<<"特征向量 = "<<endl;
      //cout<<evecs<<endl;
      
      for(int i=0;i<3;i++)//使用特征向量更新旋转矩阵R
      {
	R(0,i) = evecs.real()(0,i);
	R(1,i) = evecs.real()(1,i);
	R(2,i) = evecs.real()(2,i);
      }
      if(R.determinant()<0)
      {
	R = R*-1.0;
      }
      //cout<<"R = "<<endl;
      //cout<<"R 行列式 = "<<R.determinant()<<endl;
      //cout<<R<<endl;
      //R = Eigen::Matrix3d::Identity();
      t = -R*average;
      vector<Eigen::Vector3d> output;
      for(int i=0;i<orignal.size();i++)
      {
	Eigen::Vector3d coor_new = R*(orignal[i]-average);
	output.push_back(coor_new);
      }
      return output;
    }
  
  
    void cvInitIntrinsicParams2Dnew( const CvMat* objectPoints,        //need to change
			    const CvMat* imagePoints, const CvMat* npoints,
			    CvSize imageSize, CvMat* cameraMatrix,
			    bool use_normalization,double aspectRatio )
    {
	Ptr<CvMat> matA, _b;//_allH;
	int i, j,k, pos, nimages, ni = 0;                        //add k;
	double a[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };            //相机内参
	double H[12], f[2];                         //f表示参数fx和fy
	CvMat _a = cvMat( 3, 3, CV_64F, a );
	CvMat matH = cvMat( 3, 4, CV_64F,H);                
	CvMat _f = cvMat( 2, 1, CV_64F,f);
	
	
	
	assert( CV_MAT_TYPE(npoints->type) == CV_32SC1 &&
		CV_IS_MAT_CONT(npoints->type) );
	nimages = npoints->rows + npoints->cols - 1;//图像个数

	if( (CV_MAT_TYPE(objectPoints->type) != CV_32FC3 &&
	    CV_MAT_TYPE(objectPoints->type) != CV_64FC3) ||
	    (CV_MAT_TYPE(imagePoints->type) != CV_32FC2 &&
	    CV_MAT_TYPE(imagePoints->type) != CV_64FC2) )
	    CV_Error( CV_StsUnsupportedFormat, "Both object points and image points must be 2D" );

	if( objectPoints->rows != 1 || imagePoints->rows != 1 )
	    CV_Error( CV_StsBadSize, "object points and image points must be a single-row matrices" );

	matA.reset(cvCreateMat( 5*nimages, 2, CV_64F ));
	_b.reset(cvCreateMat( 5*nimages, 1, CV_64F ));
	a[2] = (!imageSize.width) ? 0.5 : (imageSize.width)*0.5;
	a[5] = (!imageSize.height) ? 0.5 : (imageSize.height)*0.5;
	//_allH.reset(cvCreateMat( nimages, 12, CV_64F )); //存储的是每张图像的单应性矩阵

	// extract vanishing points in order to obtain initial value for the focal length
	int count = MAX(objectPoints->cols, objectPoints->rows)/nimages;//每张图像的角点坐标 
	for( i = 0, pos = 0; i < nimages; i++, pos += ni )
	{
	    ni = npoints->data.i[i];
	    CvMat _m, matM;      
	    cvGetCols( objectPoints, &matM, pos, pos + ni );//得到这个图像的三维坐标
	    cvGetCols( imagePoints, &_m, pos, pos + ni );//得到这个图像的像素坐标
            
	    //首先经过pca分解得到新的坐标系统<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
	    vector<Eigen::Vector3d> input3Dpoints,out3Dpoints;
	    cv::Mat matM_mat = cv::cvarrToMat(&matM);
	    for(int row=0;row<matM_mat.rows;row++)
	    {
		    for(int col=0;col<matM_mat.cols;col++)
		    {
			double x = double(matM_mat.at<Vec3f>(row,col)[0]);
			double y = double(matM_mat.at<Vec3f>(row,col)[1]);
			double z = double(matM_mat.at<Vec3f>(row,col)[2]);
			input3Dpoints.push_back(Eigen::Vector3d(x,y,z));
		    }
	    }
	    
	    Eigen::Matrix3d R_3D;
	    Eigen::Vector3d t_3D;
	    Eigen::Matrix3d R_scale_3D= Eigen::Matrix3d::Identity();
	    out3Dpoints = choose_coordinate(input3Dpoints,R_3D,t_3D);
	    double x_scale=0,y_scale=0,z_scale=0;
	    for(int k=0;k<out3Dpoints.size();k++)
	    {
	      x_scale = x_scale + abs(out3Dpoints[k][0]);
	      y_scale = y_scale + abs(out3Dpoints[k][1]);
	      z_scale = z_scale + abs(out3Dpoints[k][2]);
	    }
	    x_scale = double(out3Dpoints.size())/x_scale;
	    y_scale = double(out3Dpoints.size())/y_scale;
	    z_scale = double(out3Dpoints.size())/z_scale;
	    R_scale_3D(0,0) = x_scale;
	    R_scale_3D(1,1) = y_scale;
	    R_scale_3D(2,2) = z_scale;
	    for(int k=0;k<out3Dpoints.size();k++)
	    {
	      out3Dpoints[k] = R_scale_3D*out3Dpoints[k];
	    }
	    Eigen::Matrix<double,4,4> T_3D_eigen;
	    Eigen::Matrix3d  Rtemp = R_scale_3D*R_3D;
	    Eigen::Vector3d ttemp = R_scale_3D*t_3D;
	    T_3D_eigen(0,0) = Rtemp(0,0);T_3D_eigen(0,1) = Rtemp(0,1);T_3D_eigen(0,2) = Rtemp(0,2);T_3D_eigen(0,3) = ttemp[0];
	    T_3D_eigen(1,0) = Rtemp(1,0);T_3D_eigen(1,1) = Rtemp(1,1);T_3D_eigen(1,2) = Rtemp(1,2);T_3D_eigen(1,3) = ttemp[1];
	    T_3D_eigen(2,0) = Rtemp(2,0);T_3D_eigen(2,1) = Rtemp(2,1);T_3D_eigen(2,2) = Rtemp(2,2);T_3D_eigen(2,3) = ttemp[2];
	    T_3D_eigen(3,0) = 0.0;       T_3D_eigen(3,1) = 0.0;       T_3D_eigen(3,2) = 0.0;       T_3D_eigen(3,3) = 1.0;
	    if(!use_normalization)
	    {
	      out3Dpoints = input3Dpoints;
	      T_3D_eigen(0,0) = 1.0; T_3D_eigen(0,1) = 0.0;T_3D_eigen(0,2) = 0.0;T_3D_eigen(0,3) = 0.0;
	      T_3D_eigen(1,0) = 0.0; T_3D_eigen(1,1) = 1.0;T_3D_eigen(1,2) = 0.0;T_3D_eigen(1,3) = 0.0;
	      T_3D_eigen(2,0) = 0.0; T_3D_eigen(2,1) = 0.0;T_3D_eigen(2,2) = 1.0;T_3D_eigen(2,3) = 0.0;
	      T_3D_eigen(3,0) = 0.0; T_3D_eigen(3,1) = 0.0;T_3D_eigen(3,2) = 0.0;T_3D_eigen(3,3) = 1.0;
	    }
	    //下面开始计算像素坐标的变换矩阵
	    cv::Mat _m_mat = cv::cvarrToMat(&_m);
	    //show_Mat(_m_mat,2);
	    vector<Eigen::Vector2d> input2Dpoints,out2Dpoints;
	    for(int row=0;row<_m_mat.rows;row++)
	    {
		    for(int col=0;col<_m_mat.cols;col++)
		    {
			double x = double(_m_mat.at<Vec2f>(row,col)[0]);
			double y = double(_m_mat.at<Vec2f>(row,col)[1]);
			input2Dpoints.push_back(Eigen::Vector2d(x,y));
			//cout<<x<<" "<<y<<endl;
		    }
	    }
	    Eigen::Matrix3d T_2D= Eigen::Matrix3d::Identity();
	    Eigen::Vector2d avg_uv(0,0);
	    for(int k=0;k<input2Dpoints.size();k++)
	    {
	      avg_uv = avg_uv + input2Dpoints[k];
	    }
	    avg_uv = avg_uv/double(input2Dpoints.size());
	    double scale_u=0,scale_v=0;
	    for(int k=0;k<input2Dpoints.size();k++)
	    {
	      double u = (input2Dpoints[k]-avg_uv)[0];
	      double v = (input2Dpoints[k]-avg_uv)[1];
	      scale_u = scale_u + abs(u);
	      scale_v = scale_v + abs(v);
	    }
	    scale_u = double(input2Dpoints.size())/scale_u;
	    scale_v = double(input2Dpoints.size())/scale_v;
	    T_2D(0,0) = scale_u;
	    T_2D(1,1) = scale_v;
	    T_2D(0,2) = -scale_u*avg_uv[0];
	    T_2D(1,2) = -scale_v*avg_uv[1];
	    for(int k=0;k<input2Dpoints.size();k++)
	    {
	      Eigen::Vector3d homo_pixel(input2Dpoints[k][0],input2Dpoints[k][1],1);
	      Eigen::Vector3d transformed_homo_pixel = T_2D*homo_pixel;
	      out2Dpoints.push_back(Eigen::Vector2d(transformed_homo_pixel[0],transformed_homo_pixel[1]));
	    }
	    if(!use_normalization)
	    {
	      out2Dpoints = input2Dpoints;
	      T_2D = Eigen::Matrix3d::Identity();
	    }
	    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>..
	    //下面是开始求单应性矩阵
	    double* L;
	    double LL[12*12], LW[12], LV[12*12];    //changed:double LL[12*12], LW[12], LV[12*12],sc;
	    CvMat _LL = cvMat( 12, 12, CV_64F, LL );
	    CvMat _LW = cvMat( 12, 1, CV_64F, LW );
	    CvMat _LV = cvMat( 12, 12, CV_64F, LV );
	    CvMat hommatrix;
	    
	    Ptr<CvMat>  matL;
	    matL.reset(cvCreateMat( 2*count, 12, CV_64F ));
	    L = matL->data.db;

	    for( k = 0; k < count; k++, L += 24 )        //changed:i to k
	    {
		double x = -out2Dpoints[k][0], y = -out2Dpoints[k][1];
		L[0] = L[16] = out3Dpoints[k][0];
		L[1] = L[17] = out3Dpoints[k][1];
		L[2] = L[18] = out3Dpoints[k][2];
		L[3] = L[19] = 1.;
		L[4] = L[5] = L[6] = L[7] = 0.;
		L[12] = L[13] = L[14] = L[15] = 0.;
		L[8] = x*out3Dpoints[k][0];
		L[9] = x*out3Dpoints[k][1];
		L[10] = x*out3Dpoints[k][2];
		L[11] = x;
		L[20] = y*out3Dpoints[k][0];
		L[21] = y*out3Dpoints[k][1];
		L[22] = y*out3Dpoints[k][2];
		L[23] = y;
	    }

	    cvMulTransposed( matL, &_LL, 1 );
	    cvSVD( &_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T );
	    matH = cvMat( 3, 4, CV_64F, LV + 11*12 );
	    //计算完毕变换之后的齐次坐标 乘以之前的变换矩阵得到最终的齐次变化矩阵
	    //将我们上面得到的矩阵转换成opencv的格式
	    Eigen::Matrix<double,3,4> T_temp = T_2D.inverse()*Convert::Convert_to_EigenMatrix(matH) *T_3D_eigen;
	    //再使用非现行优化 优化T_temp;
	    T_temp = T_temp/T_temp(2,3);
 	    T_temp = optimize_homogo(T_temp,input3Dpoints,input2Dpoints);//非常重要借鉴opencv的单应性矩阵的代码
	    Eigen::Vector3d r3_vector(T_temp(2,0),T_temp(2,1),T_temp(2,2));
	    matH = Convert::Convert_to_CvMat(T_temp/r3_vector.norm());//计算得到最终的单应性矩阵
	    
	    //更新H中的数据
	    for(int row=0;row<matH.rows;row++)
	    {
		double * matH_ptr = matH.data.db+row*(matH.step/8);
		for(int col=0;col<matH.cols;col++)
		{
		      H[row*matH.cols+col] = *(matH_ptr+col);
		}
	    }
	   
	    //cout<<"计算得到的单应性矩阵 ="<<endl;
	    //show_cvMat_instance(matH,1);
	    //cout<<"更新得到的H数组="<<endl;
	    //cout<<H[0]<<" "<<H[1]<<" "<<H[2]<<" "<<H[3]<<endl;
	    //cout<<H[4]<<" "<<H[5]<<" "<<H[6]<<" "<<H[7]<<endl;
	    //cout<<H[8]<<" "<<H[9]<<" "<<H[10]<<" "<<H[11]<<endl;
	    
	    //3*4的单应性矩阵计算完毕,开始个更新fx和fy的相关矩阵
	    double* Ap = matA->data.db + i*10;     //changed:double* Ap = matA->data.db + i*4;
	    double* bp = _b->data.db + i*5;       //changed:double* bp = _b->data.db + i*2;
	    double h[3], v[3],u[3], d1[3], d2[3], d3[3], d4[3];      //changed:double h[3], v[3], d1[3], d2[3];
	    double n[7] = {0,0,0,0,0,0,0};                         //changed:double n[4] = {0,0,0,0};
	    //H矩阵等价于matH矩阵
	    H[0] -= H[8]*a[2]; H[1] -= H[9]*a[2]; H[2] -= H[10]*a[2];H[3] -= H[11]*a[2];
	    H[4] -= H[8]*a[5]; H[5] -= H[9]*a[5]; H[6] -= H[10]*a[5];H[7] -= H[11]*a[5]; //added
	    
	    for( j = 0; j < 3; j++ )       
	    {
		double t0 = H[j*4], t1 = H[j*4+1], t2 = H[j*4+2];   //changed:double t0 = H[j*3], t1 = H[j*3+1];
		h[j] = t0; v[j] = t1; u[j]=t2;       //changed:h[j] = t0; v[j] = t1; 
		
		d1[j] = (t0 + t1)*0.5;
		d2[j] = (t0 - t1)*0.5;
		d3[j] = (t1 + t2)*0.5;     //added
		d4[j] = (t1 - t2)*0.5;     //added
		
		n[0] += t0*t0; 
		n[1] += t1*t1;
		n[2] += t2*t2;
		
		n[3] += d1[j]*d1[j]; 
		n[4] += d2[j]*d2[j];
		n[5] += d3[j]*d3[j]; 
		n[6] += d4[j]*d4[j];
	    }

	    for( j = 0; j < 7; j++ )
		n[j] = 1./std::sqrt(n[j]);

	    for( j = 0; j < 3; j++ )
	    {
		h[j] *= n[0]; 
		v[j] *= n[1];
		u[j] *= n[2];//added
		
		d1[j] *= n[3]; 
		d2[j] *= n[4];
		d3[j] *= n[5]; 
		d4[j] *= n[6];
	    }

	    Ap[0] = h[0]*v[0];    Ap[1] = h[1]*v[1];
	    Ap[2] = v[0]*u[0];    Ap[3] = v[1]*u[1];  
	    Ap[4] = h[0]*u[0];    Ap[5] = h[1]*u[1];  
	    Ap[6] = d1[0]*d2[0];  Ap[7] = d1[1]*d2[1];
	    Ap[8] = d3[0]*d4[0];  Ap[9] = d3[1]*d4[1];    
	  
	    bp[0] = -h[2]*v[2]; 
	    bp[1] = -v[2]*u[2]; 
	    bp[2] = -h[2]*u[2]; 
	    bp[3] = -d1[2]*d2[2];
	    bp[4] = -d3[2]*d4[2];
	}
        //先计算矩阵matA*matA转置的特征值
        CvMat matA_instance=*matA;
	//show_cvMat_instance(matA_instance,1);
	Mat matA_mat = cvarrToMat(&matA_instance);
	//cout<<"mat数据类型-------------------------------------------------"<<endl;
	//show_Mat(matA_mat,1);
	Mat eValuesMat,eVectorsMat;
	cv::eigen(matA_mat.t()*matA_mat, eValuesMat, eVectorsMat);
	std::cout << "特征值值 = " << std::endl;
	for(auto i=0; i<eValuesMat.rows; i++)
	{
		for(auto j=0; j<eValuesMat.cols; j++){
			std::cout << eValuesMat.at<double>(i,j) << " ";
		}
		std::cout << std::endl;
	}
	
	cvSolve( matA, _b, &_f, CV_NORMAL + CV_SVD );
	//cout<<"fx = "<<f[0]<<endl;
	//cout<<"fy = "<<f[1]<<endl;
	a[0] = std::sqrt(fabs(1./f[0]));
	a[4] = std::sqrt(fabs(1./f[1]));
	if( aspectRatio != 0 )
	{
	    double tf = (a[0] + a[4])/(aspectRatio + 1.);
	    a[0] = aspectRatio*tf;
	    a[4] = tf;
	}
        //show_cvMat_instance(_a,1);
	cvConvert( &_a, cameraMatrix );
    }
    
    //opencv的自带函数完全没有修改
    static void collectCalibrationData( InputArrayOfArrays objectPoints,
                                    InputArrayOfArrays imagePoints1,
                                    InputArrayOfArrays imagePoints2,
                                    Mat& objPtMat, Mat& imgPtMat1, Mat* imgPtMat2,
                                    Mat& npoints )
{
    int nimages = (int)objectPoints.total();//m*n,m=±ê¶š°åÉÏÄÚœÇµãžöÊý£¬n±íÊŸÒ»¹²¶àÉÙžöÏà»úÎ»×Ë
    int i, j = 0, ni = 0, total = 0;
    CV_Assert(nimages > 0 && nimages == (int)imagePoints1.total() &&
        (!imgPtMat2 || nimages == (int)imagePoints2.total()));

    for( i = 0; i < nimages; i++ )
    {
        ni = objectPoints.getMat(i).checkVector(3, CV_32F);//ni=3
        if( ni <= 0 )
            CV_Error(CV_StsUnsupportedFormat, "objectPoints should contain vector of vectors of points of type Point3f");
        int ni1 = imagePoints1.getMat(i).checkVector(2, CV_32F);
        if( ni1 <= 0 )
            CV_Error(CV_StsUnsupportedFormat, "imagePoints1 should contain vector of vectors of points of type Point2f");
        CV_Assert( ni == ni1 );

        total += ni;
    }

    npoints.create(1, (int)nimages, CV_32S);//CV_32S is a signed 32bit integer value for each pixel,1ÐÐ m*nÁÐ
    objPtMat.create(1, (int)total, CV_32FC3);//3ÍšµÀ
    imgPtMat1.create(1, (int)total, CV_32FC2); //2ÍšµÀ
    Point2f* imgPtData2 = 0;

    if( imgPtMat2 )
    {
        imgPtMat2->create(1, (int)total, CV_32FC2);
        imgPtData2 = imgPtMat2->ptr<Point2f>();
    }

    Point3f* objPtData = objPtMat.ptr<Point3f>();
    Point2f* imgPtData1 = imgPtMat1.ptr<Point2f>();

    for( i = 0; i < nimages; i++, j += ni )
    {
        Mat objpt = objectPoints.getMat(i);
        Mat imgpt1 = imagePoints1.getMat(i);
        ni = objpt.checkVector(3, CV_32F);//ÍšµÀÊý¡¢Éî¶È£¬Âú×ãÒ»¶šÌõŒþµÄ»°ŸÍ·µ»ØMatÔªËØµÄžöÊýºÍÆäÍšµÀÊýµÄ³Ë»ý£¬·ñÔò·µ»Ø-1
        npoints.at<int>(i) = ni;
        memcpy( objPtData + j, objpt.ptr(), ni*sizeof(objPtData[0]) );
        memcpy( imgPtData1 + j, imgpt1.ptr(), ni*sizeof(imgPtData1[0]) );

        if( imgPtData2 )
        {
            Mat imgpt2 = imagePoints2.getMat(i);
            int ni2 = imgpt2.checkVector(2, CV_32F);
            CV_Assert( ni == ni2 );
            memcpy( imgPtData2 + j, imgpt2.ptr(), ni*sizeof(imgPtData2[0]) );
        }
    }
}
    
    //opencv 自带的函数 完全没有修改
    cv::Mat initCameraMatrix2DNew( InputArrayOfArrays objectPoints,
				  InputArrayOfArrays imagePoints,
				  Size imageSize, bool use_normalization = false,double aspectRatio=0)
    {
    
	Mat objPt, imgPt, npoints, cameraMatrix(3, 3, CV_64F);
	collectCalibrationData( objectPoints, imagePoints, noArray(),
				objPt, imgPt, 0, npoints );
	CvMat _objPt = objPt, _imgPt = imgPt, _npoints = npoints, _cameraMatrix = cameraMatrix;
	cvInitIntrinsicParams2Dnew( &_objPt, &_imgPt, &_npoints,
				imageSize, &_cameraMatrix,use_normalization,aspectRatio);
	//show_cvMat_instance(_cameraMatrix,1);
	//show_cvMat_instance(cameraMatrix,1);
	return cameraMatrix;
    }
    
}
///
#endif
