    %本程序是根据cop-slam的代码修改的 完成对回环边的绘制
    clear all;
    clc;
    close all;
    
    %绘制图像6
     %filename = 'KITTI_00';
     %filename = 'KITTI_02';
     filename ='Pittsburgh_A';
      %filename= 'Pittsburgh_B';
     %filename= 'Pittsburgh_C';
    %首先读取整体的优化结果 并绘制图像
    path_true='';
     if (strcmp(filename,'Pittsburgh_A') || strcmp(filename,'Pittsburgh_B') || strcmp(filename,'Pittsburgh_C'))
          path_true = ['/home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/' filename '_gt_align.g2o'];
     else
         path_true = ['/home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/' filename '_gt.g2o'];
     end
    path_vo = ['/home/fyy/Documents/slam_source_code/COP-SLAM_v0.1/data/' filename '_vo.g2o'];
    %% parse the file
    [track_vo loops_vo]  = parseG2OFile( path_vo  );%track的大小是3×l l为顶点的个数 loop大小是2×m m为回环边的个数 而且第一个id一定比第二个大
    [track_true loops_true]  = parseG2OFile( path_true );
    %计算这个数据集的总长度,单位是米
    [M N]=size(track_true);
     totallength = 0;
     pre_pos = [0;0;0];
     cur_pos = [0;0;0];
     for i=1:N
         if i==1
             pre_pos = track_true(:,1);
         else
             cur_pose = track_true(:,i);
             totallength = totallength + norm(cur_pose-pre_pos);
             pre_pos = cur_pose;
         end
     end
     totallength
      
     plottwoTracks(track_true,track_vo,filename);
     plotLops(loops_vo,filename);%绘制回环分布图
    %绘制图像7
    %然后绘制单独的闭环轨迹
%     names = {'iSAM_full','FYY','ORB','G2O','SLAM14','CERES','Partition'};
%     method = 2;%表示使用哪种算法进行的pose graph优化
%     for n = 1:size(loops_vo,2)%遍历不同的闭环段
%         path_subset = ['/home/fyy/Documents/my_code/posegraph/build/subset_result/' char(names(method)) '/'];
%         id1 = loops_vo(2,n);
%         id2 = loops_vo(1,n);
%         subset_id = [num2str(id1) '_' num2str(id2)]
%         path_subset_pose = [path_subset 'Pose/opti_' subset_id '.g2o'];%我们得到的闭环段的轨迹
%         path_subset_truepose=['/home/fyy/Documents/my_code/posegraph/build/subset_result/TRUE/true_subset_' subset_id '.g2o'];%闭环段的真值轨迹
%         path_subset_vopose = ['/home/fyy/Documents/my_code/posegraph/build/subset_result/VO/vo_subset_' subset_id '.g2o'];%没有优化之前的结果
%        
%         track_subset_gt   = parseG2OFile( path_subset_truepose  );%读取真值文件
%         track_subset_opt   = parseG2OFile( path_subset_pose  );%读取优化后的结果
%         track_subset_vo  = parseG2OFile( path_subset_vopose  );%读取初始的值
%         loops(1,1)=id2-id1;
%         loops(2,1)=0;
%         plot_subsetTrack(track_subset_opt,track_subset_gt,track_subset_vo,loops,id1,filename);
%     end


    %绘制图像8
    path_GPGO = '/home/fyy/Documents/my_code/posegraph/build/whole_result/opti_FYY_group.g2o';
    [track_GPGO loop_GPO] = parseG2OFile(path_GPGO);
    plottwoTracks(track_true,track_GPGO,filename);






