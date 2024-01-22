%%
%% Plot trajectory in 3D
%%track是我们自己算法得到的回环 loops是我们读取到的边信息为了绘制回环点 track_gt是真实姿态
%%这个函数主要是完成整体优化的结果绘制
function plot3Tracks( track, track_gt ,track_vo)
         % plot in 3D   
         %绘制所有的轨迹
         figure('name','Whole Result')
         %plot3( track(1,:), track(3,:), track(2,:),'b' );%我们算法得到的轨迹绘制成蓝色
         hold on;
         plot3( track_gt(1,:), track_gt(3,:), track_gt(2,:),'r' );%真实轨迹绘制为红色 
         hold on;
         plot3( track_vo(1,:), track_vo(3,:), track_vo(2,:),'g' );%vo轨迹绘制为绿色
         view(-50,30);
         grid on
         axis equal
         legend('optimized track','true track','VO track')
         
        
        % plot loop closures只绘制某个回环的轨迹
%         for n = 1:size(loops,2)
%             figure(n+1);
%             %scatter3(track(1,:), track(3,:), track(2,:),'g');
%             plot3( track(1,:), track(3,:), track(2,:),'g' );%普通的轨迹绘制成绿色
%             hold on;
%             x = track(1,loops(2,n)+1:loops(1,n)+1);
%             y = track(2,loops(2,n)+1:loops(1,n)+1);
%             z = track(3,loops(2,n)+1:loops(1,n)+1);
%             title(['回环' num2str(loops(2,n)+1) '->' num2str(loops(1,n)+1)]);
%             plot3( x, z, y, 'r' );%某段回环的轨迹绘制为红色
%             hold on;
%             scatter3(track(1,loops(1,n)+1),track(3,loops(1,n)+1),track(2,loops(1,n)+1),'MarkerFaceColor','r');%红色终点
%             hold on;
%             scatter3(track(1,loops(2,n)+1),track(3,loops(2,n)+1),track(2,loops(2,n)+1),'MarkerFaceColor','g');%绿色起点
%             view(-50,30);
%             grid on
%             axis equal%横轴纵轴的定标系数设成相同值 ,即单位长度相同，
%             legend('trajectory','loop closures','Location','NorthWest')
%         end
return