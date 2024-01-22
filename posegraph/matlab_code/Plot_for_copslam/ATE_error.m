%就是那两个姿态的绝对误差
function ate_error = ATE_error( track_true,track )
    [true_row true_col]=size(track_true);
    [own_row own_col]=size(track);
    sum_error=0;
   if true_col==own_col
        for i=1:1:true_col
          error = track(:,i)-track_true(:,i);
          sum_error =  norm(error)*norm(error)+sum_error;
        end   
        ate_error = sqrt(sum_error)/own_col;
   else
       disp('输入的数据大小不同');
       ate_error = 0;
   end  
return