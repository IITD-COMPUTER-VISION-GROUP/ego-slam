function [tracks,track_size] = track_read(file,num_original_cams,mapfile,map_for_trans_avg);

img_map = load(mapfile);
map_original_to_cluster =zeros(num_original_cams,1);

for i =1:size(img_map,1)
    map_original_to_cluster(img_map(i)+1) = i;
end

fid = fopen(file,'r');

tline = fgetl(fid);
tracks = [];
ind = 1;
while ischar(tline)
    
    str=strread(tline,'%s');
    num = str2num(str{7});
    
    A = [str2num(str{7})];
    track_imgs = str2num(str{7});
    %val = num*3;
    flag = 1;
    for i = 1:num
        cloc = 7 + (i-1)*4;
        A = [A map_for_trans_avg(map_original_to_cluster(str2num(str{cloc+1})+1))];
        if(map_for_trans_avg(map_original_to_cluster(str2num(str{cloc+1})+1)) ==0)
            flag = 0;
            break;
        end
        for j =1: 3
            A = [A str2num(str{cloc+1+j})];
        end
    end
    
    if(flag ==0)
        tline = fgetl(fid);
        continue;
    end
    tracks{ind}= A;
    track_size(ind,1) =ind;
    track_size(ind,2) =uint32(track_imgs);
    ind = ind + 1;
    tline = fgetl(fid);
end
track_size = sortrows(track_size,-2);
tracks = tracks';
fclose(fid);
end
