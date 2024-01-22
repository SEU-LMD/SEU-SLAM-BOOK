%%这个函数主要是完成对某一条轨迹进行绘制的工作
function plottwoTracks( track_true,track_vo,filename)
         % plot in 3D   
         %绘制所有的轨迹
         figure('name',filename)
         title(filename,'Interpreter','none');%'Interpreter','none'为了显示下划线正常
         hold on;
         plot3( track_true(1,:), track_true(3,:), track_true(2,:),'r','LineWidth',2);%真值的轨迹绘制成红色
         hold on;
         plot3( track_vo(1,:), track_vo(3,:), track_vo(2,:),'b','LineWidth',2);%vo或者我们优化得到的轨迹绘制成蓝色
         %plot3(  track_vo(2,:),track_vo(1,:),track_vo(3,:) ,'b','LineWidth',2);%vo或者我们优化得到的轨迹绘制成蓝色
         view(-50,30);
         grid on
         axis equal
         legend('True Trajectory','G-PGO Trajectory')
return