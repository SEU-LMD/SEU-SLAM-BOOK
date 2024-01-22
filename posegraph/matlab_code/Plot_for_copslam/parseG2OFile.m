%%
%% Parse absolute positions from a g2o file
%%
function [track, loops] = parseG2OFile( filename )
    
    % open the file
    fid   = fopen(filename);
    verts = fscanf(fid,'VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n' ,[inf]);
    loops = fscanf(fid,'EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' ,[inf]);
    if isempty( verts )
        fseek(fid, 0, 'bof');
        verts  = fscanf(fid,'VERTEX_RST3:QUAT %i %f %f %f %f %f %f %f %f\n' ,[inf]);
        loops  = fscanf(fid,'EDGE_RST3:QUAT %i %i %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' ,[inf]);
        verts  = reshape(verts,9,size(verts,1)/9);
        verts  = verts(1:8,:);
        loops  = reshape(loops,31,size(loops,1)/31);
        loops  = loops(1:10,:);        
    else
        verts  = reshape(verts,8,size(verts,1)/8);%verts 列是读取的定点信息
        loops  = reshape(loops,30,size(loops,1)/30);%loops 列存储的是边对应的两个定点
        loops  = loops(1:2,:);
    end    
    track = verts(2:4,:);%得到每个顶点的绝对位置 大小3×n n为顶点的个数
    idx   = find( 1 < abs(loops(1,:) - loops(2,:)) );
    loops = loops(:,idx);%现在loops中列存储的是回环边相关的两个定点序号 2*m m为回环变得个数
    fclose( fid );
    
return