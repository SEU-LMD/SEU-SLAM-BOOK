%绘制闭环帧的重叠性
function plotLops( loops,filename )
         figure('name',filename)
         title(filename,'Interpreter','none');%'Interpreter','none'为了显示下划线正常
         hold on;
         xlabel={};
         for n = 1:size(loops,2)
            hold on;
            plot([loops(2,n)+1 loops(1,n)+1],[n n],'b','LineWidth',5);   
            label = [num2str(loops(2,n)+1) '->' num2str(loops(1,n)+1)];
            xlabel=[xlabel,label];
         end
         set(gca,'ytick',1:1:n);
         set(gca,'yticklabel',xlabel);
return