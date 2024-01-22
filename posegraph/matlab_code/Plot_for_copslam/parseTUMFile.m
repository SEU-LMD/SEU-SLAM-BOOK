function track = parseTUMFile( filename )
fid   = fopen(filename);
    poses = fscanf(fid,'%f %f %f %f %f %f %f %f\n' ,[inf]);
    poses  = reshape(poses,8,size(poses,1)/8);%verts 列是读取的定点信息
    track = poses(2:4,:);%得到每个顶点的绝对位置 大小3×n n为顶点的个数
    fclose( fid );
end

