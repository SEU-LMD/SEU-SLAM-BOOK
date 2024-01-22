%%
%% Parse absolute positions from a g2o file
%%
function [pose] = parseLIFile( filename )
    % open the file
    fid   = fopen(filename);
    pose = fscanf(fid,'%i %lf %lf %lf %lf\n' ,[inf]);
    pose=reshape(pose,5,size(pose,1)/5);
    fclose( fid );
return