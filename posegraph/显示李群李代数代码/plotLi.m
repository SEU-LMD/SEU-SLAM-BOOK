function plotLi(pose)
    figure(1)
    clf%用来清除图形的命�?    hold on
    title('Ground True');
    view(-50,30);
    % plot loop closures
    x_bef=0;
    y_bef=0;
    z_bef=0;
%     for n = 1:size(pose,2)
%         n
%         hold on;
%         x=pose(2,n);
%         y=pose(3,n);
%         z=pose(4,n);
% %         angle = norm(pose(i,:));
% %         if angle>pi
% %              angle = angle-2*pi;
% %              x = x
% %         end   
%          %scatter3(x,y,z,'b');%绘制�?%          text(x,y,z,num2str(pose(1,n)));%在这个定点旁边加上标�?%          a=[x x_bef];
%          b=[y y_bef];
%          c=[z z_bef];
%          if n~=1
%              plot3( a, b, c, 'r' );%绘制�?%          end
%          x_bef = x;
%          y_bef = y;
%          z_bef = z;
%     end
    
    count=0;
    figure(2)
    hold on
    title('角度');
    subplot(4,1,1);%绘制角度
    id_horizon=0;
    theata_vertical=0;
    for n = 1:size(pose,2)
         hold on;
         id_horizon=[id_horizon pose(1,n)];
         theata_vertical = [theata_vertical pose(5,n)];
%          fi_former=0;
%          whichfi_former=0;
%          sign_change=0;
%          if (sign_change==0)&&(theta_vertical>(170/180)*pi)
%              maxvalue=max([abs(pose(2,n)) abs(pose(3,n)) abs(pose(4,n))];
%              if abs(pose(2,n))==maxvalue
%                  fi_former = pose(2,n);
%                  whichfi_former=2;
%              end
%              if abs(pose(3,n))==maxvalue
%                  fi_former = pose(3,n);
%                   whichfi_former=3;
%              end
%              if abs(pose(4,n))==maxvalue
%                   fi_former = pose(4,n);
%                   whichfi_former=4;
%              end
%          end
%          
%          if pose(which_former,n)*fi_former<0
%                sign_change=1;  
%          else
%                sign_change=0;
%          end
         scatter(pose(1,n),pose(5,n),10,'b');
    end
    plot(id_horizon,theata_vertical);
    hold on
    %绘制直线
    plot([0,size(pose,2)],[pi pi],'r');
    hold on
    %绘制直线
    plot([0,size(pose,2)],[-pi -pi],'r');
    hold on
    plot([0,size(pose,2)],[pi/2 pi/2],'r');
    hold on
    %绘制直线
    plot([0,size(pose,2)],[-pi/2 -pi/2],'r'); 
    
    
    
    subplot(4,1,2);%绘制李代数中的x
    horizon=0;
    vertical=0;
    for n = 1:size(pose,2)
         hold on;
         horizon=[horizon pose(1,n)];
         if pose(2,n)>0
            valuefix=1;
         else
            valuefix=-1;
         end
         vertical = [vertical valuefix];
         scatter(pose(1,n),pose(2,n),10,'r');
    end
    hold on;
    plot(horizon,vertical);
%     hold on
%     %绘制直线
%     plot([0,size(pose,2)],[pi pi],'r');
%     hold on
%     %绘制直线
%     plot([0,size(pose,2)],[-pi -pi],'r');
    
    
    
    subplot(4,1,3);%绘制李代数中的y
    horizon=0;
    vertical=0;
    for n = 1:size(pose,2)
         hold on;
         horizon=[horizon pose(1,n)];
         if pose(3,n)>1
             valuefiy=1;
         else
             valuefiy=-1;
         end
         vertical = [vertical valuefiy];
         scatter(pose(1,n),pose(3,n),10,'r');
    end
    hold on;
    plot(horizon,vertical);
%    hold on
%     %绘制直线
%     plot([0,size(pose,2)],[pi pi],'r');
%     hold on
%     %绘制直线
%     plot([0,size(pose,2)],[-pi -pi],'r');
    
    
    
    
    subplot(4,1,4);%绘制李代数中的z
    horizon=0;
    vertical=0;
    for n = 1:size(pose,2)
         hold on;
         horizon=[horizon pose(1,n)];
         if pose(4,n)>1
             valuefiz=1;
         else
             valuefiz=-1;
         end
         vertical = [vertical valuefiz];
         scatter(pose(1,n),pose(4,n),10,'r');
    end
    hold on;
    plot(horizon,vertical);
%    hold on
%     %绘制直线
%     plot([0,size(pose,2)],[pi pi],'r');
%     hold on
%     %绘制直线
%     plot([0,size(pose,2)],[-pi -pi],'r');
return