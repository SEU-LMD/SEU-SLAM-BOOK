    clear all;
    clc;
    close all;
    filenames = {'/home/fyy/Documents/my_code/posegraph/outlier_data/KITTI00.txt',...
                 '/home/fyy/Documents/my_code/posegraph/outlier_data/KITTI02.txt',...
                 '/home/fyy/Documents/my_code/posegraph/outlier_data/P_A.txt',...
                 '/home/fyy/Documents/my_code/posegraph/outlier_data/P_B.txt',...
                 '/home/fyy/Documents/my_code/posegraph/outlier_data/P_C.txt'};
    datasets={'KITTI00','KITTI02','Pittsburgh-A','Pittsburgh-B','Pittsburgh-C'}
    figure(1);
    title('Outlier Metric');
    for i=1:5
        filenames{i}
        [id1,id2,metrix]=textread( filenames{i},'%f%f%f','delimiter',' ');
        names = {};
        for j=1:size(id1,1)
            name = [num2str(id1(j)) '->' num2str(id2(j))];
            names = [names,name];
        end
        subplot(5,1,i);
        hold on;
        title(datasets{i});
        bar(abs(metrix));
        set(gca,'xtick',1:1:size(id1,1));
        set(gca,'xticklabel',names);
        if i==5
            set(gca,'XTickLabel',names,'XTickLabelRotation', 45);
        end
    end
    
    
    
    
    

    
    
    


