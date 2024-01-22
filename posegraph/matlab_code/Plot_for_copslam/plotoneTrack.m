%%
%% Plot trajectory in 3D
%%这个函数主要是完成对某一条轨迹进行绘制的工作
function plotoneTrack( track,filename)
         % plot in 3D   
         %绘制所有的轨迹
         figure('name',filename)
         title(filename,'Interpreter','none');%'Interpreter','none'为了显示下划线正常
         hold on;
         %plot3( track(1,:), track(3,:), track(2,:),'b','LineWidth',2);%我们算法得到的轨迹绘制成蓝色
         plot3( track(2,:),track(1,:),track(3,:),'r','LineWidth',2);%我们算法得到的轨迹绘制成红色
         view(-50,30);
         grid on
         axis equal
         legend('True')
return