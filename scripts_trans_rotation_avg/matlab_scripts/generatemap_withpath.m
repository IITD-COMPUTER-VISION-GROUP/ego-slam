function generatemap_withpath(motherlistname,focalfile_cluster,mapfile)
    %motherlistname = './list_focal.txt';
    [list dum1]= textread(motherlistname,'%s %f');
    
    %listname = sprintf('./cluster12540/cluster12540_focal.txt');
    [imagename dummy focal]= textread(focalfile_cluster,'%s %d %f');
    
  
    %mapname = sprintf(mapfile);
    mapname=mapfile;
    fid = fopen(mapname,'w');
    
    map = zeros(size(imagename,1));
    for j = 1:size(imagename,1)
        imagequery = imagename{j};
        clear indices;
        clear index;
        
        indices = strfind(list, imagequery);
        index  = find(not(cellfun('isempty', indices)));
        if(size(index,1) >0)
            fprintf(fid,'%d\n',index-1);
            
        end
    end
    fclose(fid);