 
function generate_map_to_triangulate_script(oripath)
 %oripath = sprintf('H:\\suvam\\expts_depth\\test\\');
 clusternames = textread(strcat(oripath,'clusternames.ini'),'%s');
addpath('./matlab_scripts');

fid = fopen('errorFittingPerCluster.txt','w');
fclose(fid);
%callrotationAveraging(original_path);
listfocalfile = strcat(oripath,'list_focal.txt');
[list focal]= textread(listfocalfile,'%s %f');
num_cams = size(list,1);
for i = 1:size(clusternames,1)
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% if .txt is present %%%%%%%%%%%%%%%%
%     curr_clustername_with_ext  = clusternames{i};
%     index = findstr(curr_clustername_with_ext,'.');
%     index = index - 1;
%     curr_clustername = curr_clustername_with_ext(1:index);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% else %%%%%%%%%%%%%%%%
    
    curr_clustername  = clusternames{i};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
%     cluster_listname = sprintf('./%s/listsize_focal1.txt',curr_clustername);
%     clustermapname = sprintf('./%s/cluster_list_map.txt',curr_clustername);
%     generatemap(cluster_listname,clustermapname);
    
    
    
    path = strcat(oripath,curr_clustername,'\');    %sprintf('H:\\suvam\\expts_depth\\batchdata2\\%s\\',curr_clustername);
    cluster_listname = strcat(path,'listsize_focal1.txt'); %sprintf('./%s/listsize_focal1.txt',curr_clustername);
    clustermapname =  strcat(path,'cluster_list_map.txt');             %sprintf('./%s/cluster_list_map.txt',curr_clustername);
    generatemap_withpath(listfocalfile,cluster_listname,clustermapname);
    locallist_focal = strcat(path, 'list_focal.txt');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     map_path = strcat(path,'cluster_list_map.txt');
    matchesforRT = strcat(path,'matches_forRtinlier5point.txt');
    oursfile = strcat(path,'world1.txt');
    %command = sprintf('corres2.exe %d %s %s %s 0 128 255 %s',num_cams, map_path,matchesforRT,oursfile,listfocalfile);
    command = sprintf('corres_to_track.exe %s %s %s %s %s', oripath,matchesforRT, map_path, locallist_focal, oursfile);
    system(command);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    
    
    
    
    [comp,rotE, transE] = trans_avg_func_with_frob_with_oripath(path,oripath);
    save('test_comp.txt', 'comp', '-ASCII');
    save('test_rotE.txt', 'rotE', '-ASCII');
    save('test_transE.txt', 'transE', '-ASCII');    
    
   if(comp==0)
       fid = fopen('errorFittingPerCluster.txt','a');
       fprintf(fid,'%s -1.0 -1.0 -1.0 -1.0 -1.0 -1.0\n',curr_clustername);
       fclose(fid);
       continue;
   end
   
   fid = fopen('errorFittingPerCluster.txt','a');
   fprintf(fid,'%s %f %f %f %f %f %f\n',curr_clustername,rotE(1,1), rotE(1,2), rotE(1,3),transE(1,1), transE(1,2), transE(1,3));
   fclose(fid);
    
   
   mapping_sequential_to_global_index(path,  num_cams);
    
    exe = 'corres.exe';
    
    loadfilepath =strcat(path,'\','list.txt');% sprintf('./%s/list.txt',curr_clustername);
   % local_list = load(loadfilepath);
    
  %  num_images = size(local_list,1);
    
    map_path = strcat(path,'cluster_list_map.txt');
    matchesforRT = strcat(path,'matches_forRT_filtered.txt');
    oursfile = strcat(path,'world1.txt');
    %command = sprintf('corres2.exe %d %s %s %s 0 128 255 %s',num_cams, map_path,matchesforRT,oursfile,listfocalfile);
    command = sprintf('corres_to_track.exe %s %s %s %s %s', oripath, matchesforRT, map_path, locallist_focal, oursfile);
    if(comp > 1)
        system(command);
    end
    
    numcors_path = strcat(path,'num_cors.txt');
    finp = fopen(numcors_path,'r');
    cors=fscanf(finp, '%d',[1 1]);
    cors = cors(1,1);
    cors=3;
    command = sprintf('triangulate2.exe %s %s %d 0.1',path,listfocalfile, cors);
    tic;
    system(command);
    toc;
end
fclose('all');
end