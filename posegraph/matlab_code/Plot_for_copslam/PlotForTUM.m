 clear all;clc;close all;
 
 filename = 'freiburg1_room';
 %filename = 'freiburg2_desk';
 %filename = 'freiburg3_long_office';
 
 path_true = ['/home/fyy/Documents/my_code/posegraph/data/' filename '_gt.txt'];
 path_vo = ['/home/fyy/Documents/my_code/posegraph/data/' filename '_vo.txt']
 [track_vo loops_vo]  = parseG2OFile( path_vo  );%track的大小是3×l l为顶点的个数 loop大小是2×m m为回环边的个数 而且第一个id一定比第二个大
 track_true = parseTUMFile(path_true);
 
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
 plotLops(loops_vo,filename);%绘制回环分布图
 plotoneTrack(track_true,filename);
 
