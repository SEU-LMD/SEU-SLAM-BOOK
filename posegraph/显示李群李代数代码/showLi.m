   clear all;
   clc;
   close all;
%    path_true = '/home/fyy/Documents/my_code/LAGO_3D/build/output_Li.g2o';
   path_true = 'D:\my_code\LAGO_3D\build\output_Li.g2o';
   pose = parseLIFile( path_true );%
   
 %绘制图像

 plotLi(pose);
%     figure(2)
%     clf%用来清除图形的命�?%     hold on
%     title('VO');
%     plotLi();