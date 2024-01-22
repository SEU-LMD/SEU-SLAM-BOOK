%这个函数主要是绘制某段回环的轨迹
%第一个参数是优化后的轨迹 第二个参数是真实的轨迹 第三个参数是初始没有进行优化的轨迹 第四个参数是闭环段的起始id
function plot_subsetTrack( track, track_gt, track_vo,loops,id1,name)
if size(loops,2)==1%保证只有一个回环
    figure('name',['回环' num2str(loops(2,1)+id1) '->' num2str(loops(1,1)+id1)]);
    title([name '.' num2str(loops(2,1)+id1) '->' num2str(loops(1,1)+id1)],'Interpreter','none');%'Interpreter','none'为了显示下划线正常
    hold on;
    plot3( track(1,:), track(3,:), track(2,:),'g','LineWidth',2 );%优化后的轨迹绘制成绿色
    hold on;
    plot3( track_gt(1,:), track_gt(3,:), track_gt(2,:),'r','LineWidth',2 );%真值绘制为红色的轨迹
    hold on;
    plot3( track_vo(1,:), track_vo(3,:), track_vo(2,:),'b','LineWidth',2 );%初始的轨迹绘制为红色的轨迹
    hold on;
    scatter3(track(1,loops(1,1)+1),track(3,loops(1,1)+1),track(2,loops(1,1)+1),'MarkerFaceColor','r');%红色终点
    hold on;
    scatter3(track(1,loops(2,1)+1),track(3,loops(2,1)+1),track(2,loops(2,1)+1),'MarkerFaceColor','g');%绿色起点
    view(-50,30);
    grid on
    axis equal
    legend('Optimized Trajectory','True Trajectory','VO Trajectory')
else
    disp('输入的回环个数不等于1');
end
return