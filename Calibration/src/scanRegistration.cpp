// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;
//扫描周期, velodyne频率10Hz，周期0.1s
const double scanPeriod = 0.1;
//弃用前systemDelay帧初始数据
const int systemDelay = 0; 
//systemInitCount用于计数过了多少帧
//超过systemDelay后，systemInited为true即初始化完成
int systemInitCount = 0;
bool systemInited = false;
//激光雷达线数初始化为0
int N_SCANS = 0;
//点云曲率, 400000为一帧点云中点的最大数量
float cloudCurvature[400000];
//曲率点对应的序号
int cloudSortInd[400000];
//用于标注点是否已处理：0-未处理，1-已处理
int cloudNeighborPicked[400000];
//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
//用于将点云特征分类
int cloudLabel[400000];
//两点曲率比较，如果点i的曲率小于点j，返回true
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

float total_time = 0;
int total_frame = 0;

//定义点云发布的话题
//设置发布内容，整体点云，角点，降采样角点，面点，降采样面点，剔除点
ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
//ros形式的一线扫描
std::vector<ros::Publisher> pubEachScan;
//是否发布每行Scan
bool PUB_EACH_LINE = false;
//
//不接收以激光雷达为中心，半径为0.1m的球内点云
double MINIMUM_RANGE = 0.1; 

//统一时间戳，滤除最近点去除 使用template进行兼容,作用是使函数适用于不同的输入类型
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    //统一header(时间戳)和size
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    //它是一种“整型”类型，里面保存的是一个整数，就像int, long那样。这种整数用来记录一个大小(size)。
    //size_t的全称应该是size type，就是说“一种用来记录大小的数据类型”
    size_t j = 0;

    //逐点距离比较
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {   
        //当前点x^2+y^2+z^2  >= thres^2, 保存到cloud_out.points[]，否则去除
        //即：去除以激光雷达坐标原点半径为thres内的球体里的点
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }

    //有点被剔除时，size改变
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    //数据行数，默认1为无序的数据
    cloud_out.height = 1;
    //点云的长度；static_cast强制将j转换为uint32_t类型，不可能为负数
    cloud_out.width = static_cast<uint32_t>(j);
    //点数是否有限
    cloud_out.is_dense = true;
}

//订阅点云句柄
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
//每接收到一帧点云就执行一次laserCloudHandler
{
    //是否剔除前systemDelay帧
    //初始化未完成，初始点计数，超过systemDelay点数，初始化完成
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    //registration计时，TicToc库用于计时
    TicToc t_whole;
    //计算曲率前的预处理计时
    TicToc t_prepare;
    //记录每个scan有曲率的点的开始和结束索引
    //长度为N_SCANS，值全为0
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    //命名一个pcl形式的输入点云
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    //将ROS系统接收到的laserCloudMsg转为laserCloudIn点云作为A-LOAM的输入点云
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    

    //首先对该点云进行滤波，去除NaN值的无效点，以及在Lidar坐标系为原点，MINIMUM_RANGE距离以内的点
    //这个函数目的是去除NaN，第一个参数是输入点云，第二个参数是输出点云，第三个参数是对应保留的索引
    //输出里的第i个点，是输入里的第indices[i]个点，就是
    //cloud_out.points[i] = cloud_in.points[indices[i]]
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    //调用上文的removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,indices)
    //对齐时间戳，滤除太近点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    //该次scan的点数
    int cloudSize = laserCloudIn.points.size();
    //每次扫描是一条线，看作者的数据集激光x向前，y向左，那么下面就是线一端到另一端
    //atan2的输出为-pi到pi(PS:atan输出为-pi/2到pi/2)
    //计算旋转角时取负号是因为velodyne是顺时针旋转
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);//起始的角度
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;                                      //一帧旋转后的最后角度,+2pi是表示旋转一周
    //理想情况下，雷达扫描点的起始点在-x轴附近，结束点也在-x附近。
    //但是不能保证结束点就是比π稍小，起始点比-−π稍大。二者相减正好≈2π。再加上结束点加了2π。
    //激光间距收束到1pi到3pi
    //-atan2计算出来的角度在(-pi,pi]，结束点角度应该在计算出来的在(pi,3pi]，
    if (endOri - startOri > 3 * M_PI)//当起始点接近-pi，结束点接近3pi
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)//当起始点接近pi左邻域，结束点接近pi右邻域
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    //过半记录标志
    bool halfPassed = false;
    //记录总的点数
    int count = cloudSize;
    PointType point;

    //按线数保存的点云集合
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    
    //循环对每个点进行以下操作
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        //求仰角atan输出为-pi/2到pi/2，实际看scanID应该每条线之间差距是2度
        //16线的使用的是velodyne的激光雷达,±15°的垂直视场，360°水平视场扫描,则有[15-(-15)]/(16-1)=2
        //当前的点，角度变弧度
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        //根据不同线数使用不同参数对每个点对应的第几根激光线进行判断
        if (N_SCANS == 16)
        {   
            //0.5的效果等于四舍五入，
            //[angle-(-15)]/2表示当前的angle与最下端扫描线的角度差(正值)，除以2得到该点对应的扫描线
            //scanID即为扫描的第几条线(从下往上数)
            scanID = int((angle + 15) / 2 + 0.5);
            //如果判定线在16以上或是负数则忽视该点回到上面for循环
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        //只有16,32,64线
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
        //一开始初始化为false,未过半
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        //过半以后，确保-3*pi/2 < ori - endOri < pi/2
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        //看看旋转多少了，记录比例relTime
        float relTime = (ori - startOri) / (endOri - startOri);
        //假设16线Lidar的水平角度分辨率为0.2°（实际是0.1~0.4），则每个scan理论上有360/0.2=1800个点
        //为方便叙述，称每个scan点的ID为fireID，即fireID[0~1799]，相应的scanID[0~16]
        //通过Lidar坐标系下的仰角和水平夹角计算点云的scanId和fireID
        //第几根线和本线进度到多少记录在point.intensity
        //intensity的整数部分存放scanID，小数部分存放归一化后的fireID
        point.intensity = scanID + scanPeriod * relTime;
        //push_back() 在laserCloudSacans[scanID]最后添加一个元素（参数为要插入的值）
        //按线分类保存,将点根据scanID放到对应的子点云laserCloudScans中
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);
    //加了ptr的是指针类型
    //PointType不是PCL点类型，它是一个模板参数，可以代表pcl::PointXYZ或任何其他PCL数据类型
    //定义一个laserCloud，将合并后的laserCloudScans存进来
    //也就是把所有线保存在laserCloud一个数据集合里，把每条线的第五个和倒数第五个位置反馈给scanStartInd和scanEndInd
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    //将子点云laserCloudScans合并成一帧点云laserCloud
    //注意这里的单帧点云laserCloud可以认为已经是有序点云了，按照scanID和fireID的顺序存放
    for (int i = 0; i < N_SCANS; i++)
    { 
        //记录每个scan的开始index，忽略前五个点。i=0时，laserCloud->size()为0，i=1时，laserCloud->size()为加入第一条扫描线的数据
        scanStartInd[i] = laserCloud->size() + 5;
        //把scanID[i]加入到laserCloud中，此时laserCloud已经是一个单帧的16线点云，存储顺序i=0,1...16线点云排列
        *laserCloud += laserCloudScans[i];
        //记录每个scan的结束index，忽略后五个点,开始和结束处的点云容易产生步闭合的"接缝",对提取edge_feature不利
        scanEndInd[i] = laserCloud->size() - 6;
    }
    //预处理部分终于完了
    printf("prepare time %f \n", t_prepare.toc());

    //point[i]和附近十点求曲率，自然是在一条线上的十个点
    for (int i = 5; i < cloudSize - 5; i++)
    { 
        //从第6个点开始，当前点的曲率由该点及其前后五个点的对应坐标之和的平方和表示
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        //曲率，序号，是否筛过标志位，曲率分类
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        //一轮循环后变为cloudSize - 6
        cloudSortInd[i] = i;
        //点没有被选择为feature点
        cloudNeighborPicked[i] = 0;
        // Label 2：corner_sharp
        // Label 1：corner_less_sharp, 包含Label 2
        // Label -1：surf_flat
        // Label 0：surf_less_flat，包含Label -1，因为点太多，最后会降采样
        // 先默认每个点都是surf_less_flat
        cloudLabel[i] = 0;
    }

    //记录提取角点和面特征，降采样总用时
    TicToc t_pts;
    //角点，降采样角点，面点，降采样面点
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    //记录sort用时，388行
    float t_q_sort = 0;
    //根据曲率计算四种特征点：边缘点特征：sharp、less_sharp；面特征：flat、less_flat
    for (int i = 0; i < N_SCANS; i++)
    {
        //如果该scan的点数小于6个点，跳过；第i线的扫描末端索引 — 扫描开始索引
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        //定义surfPointsLessFlatScan储存less_flat点云
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        //将第i个scan(扫描线)分成6小段subscan执行特征检测,确保周围都有点被选作特征点,或者说每两行都有
        for (int j = 0; j < 6; j++)
        {
            //六等份起点：sp = scanStartInd + (scanEndInd - scanStartInd)*j/6
	        //六等份终点：ep = scanStartInd - 1 + (scanEndInd - scanStartInd)*(j+1)/6
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            //subscan的结束index
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            //根据曲率由小到大对subscan的点进行sort;  对每一线都得到了6个曲率由小到大的点集
            //bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j])};
            //cloudSortInd为数组首元素地址，该函数表示在内存块 cloudSortInd+sp 到 cloudSortInd+ep+1内,
            //通过比较曲率之间的大小改变对应内存块储存的曲率值实现排序
            //sort是一个api直接调用即可
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            //挑选每个分段的曲率很大和比较大的点
            int largestPickedNum = 0;
            //提取corner_sharp的feature，选择该subscan曲率最大的前2个点为corner_sharp，
            //曲率最大的前20个点认为是corner_less_sharp
            //从曲率大的点开始提取corner feature，此时ep到sp内对应的曲率由大到小
            for (int k = ep; k >= sp; k--)
            {
                //初始k=ep，ind为曲率最大的点的索引，由于进行排序后每条线的6个分段曲率点集由小到大排列，k=ep时分段最大的
                int ind = cloudSortInd[k];  
                
                //如果该点没有被选择过，并且曲率大于0.1
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)       
                {

                    largestPickedNum++;
                    //该subscan中曲率最大的前2个点认为是Corner_sharp特征点
                    if (largestPickedNum <= 2)  
                    {
                        //Label 2：corner_sharp                        
                        cloudLabel[ind] = 2;
                        //也将这两个corner_sharp点认为是corner_less_sharp    
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    //该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                    else if (largestPickedNum <= 20)
                    {
                        // Label 1：corner_less_sharp                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    //多了就不要了
                    else
                    {
                        break;
                    }

                    //标记该点被选择过
                    cloudNeighborPicked[ind] = 1; 

                    //与当前点距离的平方小于等于0.05的点标记为选择过，避免特征点密集分布
                    //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        //没有直接过的就算是筛过的
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            
            //提取surf_flat的feature，选择该subscan曲率最小的前4个点为surf_flat
            int smallestPickedNum = 0;
            //从曲率小的点开始提取surf_flat特征，此时sp到ep内对应的曲率由小到大
            
            for (int k = sp; k <= ep; k++)
            {
                //ind对应曲率最小的点的索引
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    //将该点标记为已选
                    cloudNeighborPicked[ind] = 1;
                    //与当前点距离的平方小于等于0.05的点标记为选择过，避免特征点密集分布
                    //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        //应该是决定简单计算不稳定，直接过
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        
                        //没有直接过的就算是筛过的
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        //没有直接过的就算是筛过的
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            //将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别中
            // Label -1：surf_flat
            // Label 0：surf_less_flat，包含Label -1，因为点太多，最后会降采样
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }
        //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        //体素网格滤波
        pcl::VoxelGrid<PointType> downSizeFilter;
        //surfPointsLessFlatScan中存着surf_less_flat特征点
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        //设置体素大小
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        //执行滤波
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        //less flat点汇总
        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    //（求出曲率后）降采样和分类的时间
    total_frame++;
    total_time += t_pts.toc();
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());
    printf("seperate points time %f \n", total_time / total_frame);

    //将四种特征点云转为ROS格式的点云
    //将当前帧点云提取出的4种特征点与滤波后的当前帧点云进行publish
    sensor_msgs::PointCloud2 laserCloudOutMsg;//ROS点云格式
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);//当前的单帧点云
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;//对齐时间戳
    laserCloudOutMsg.header.frame_id = "/camera_init";//用来告诉你，发布的数据是来自哪一个坐标系的。
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);//当前的角点
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);//当前的降采样角点
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);//当前的平面点
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);//当前的降采样平面点
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // pub each scan
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);//将每条线都发布出去
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }
    //点云预处理，提取特征用时
    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    //参数,线数
    nh.param<int>("scan_line", N_SCANS, 16);
    //参数，过近点去除
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    //接收激光雷达信号
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);
    
    //发布laserCloud，laserCloud是按线堆栈的全部点云
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
    
    //发布角点，降采样角点，面点，降采样面点
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    //发布去除点
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    //发布每行scan
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
