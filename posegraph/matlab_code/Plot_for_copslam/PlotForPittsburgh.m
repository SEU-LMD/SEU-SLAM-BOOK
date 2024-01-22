clear all;clc;close all;
 
%filename ='Pittsburgh_A';
%filename= 'Pittsburgh_B';
filename= 'Pittsburgh_C';
 
 path_true = ['/home/fyy/Documents/my_code/posegraph/data/' filename '_GPGO_GT.g2o'];
 path_vo = ['/home/fyy/Documents/my_code/posegraph/data/' filename '_GPGO_OPT.g2o']
 [track_vo loops_vo]  = parseG2OFile( path_vo  );%track的大小是3×l l为顶点的个数 loop大小是2×m m为回环边的个数 而且第一个id一定比第二个大
 [track_gt loops_gt]  = parseG2OFile( path_true  );%track的大小是3×l l为顶点的个数 loop大小是2×m m为回环边的个数 而且第一个id一定比第二个大
  plottwoTracks(track_gt,track_vo,filename);

 