clearvars;



path ='D:\slam\slam_src\data\vid5'; %create this folder inside data
videoname = 'vid5.mp4'
calib_filename = 'calib_results_1440p.txt';
cd ..\slam_src\build\Release\



%slam.exe -dirname=D:\slam\slam_src\data\vid5 -video=D:\slam\slam_src\data\vid5\vid5.MP4 -chunks=60 -undistort=true -calib=D:\slam\slam_src\data\calib_results_1440p.txt

filename_video = strcat(path,'\',videoname);
chunk_size=60;
calib_file = strcat(path,'\',calib_filename);

command_slam =strcat('slam.exe -dirname=',path,' -video=',filename_video,' -chunks=',string(chunk_size),' -undistort=true -calib=',calib_file);
system(char(command_slam));
cd ..\..\..\scripts_trans_rotation_avg\


path1 = strcat(path,'\');
generate_map_to_triangulate_script(path1);
RunBundler(path1);


%Merging 
clusternames = textread(strcat(path1,'clusternames.ini'),'%s');
if(size(clusternames,1)>=2)
    %copy 1st cluster to a temp cluster
    tmp_clustername = 'tmp_cluster'
    merging_clustername = 'combined'
    tmp_clustername_path = strcat(path1,tmp_clustername,'\');
    merging_clustername_path = strcat(path1,merging_clustername,'\');
    mkdir(tmp_clustername_path);
    mkdir(merging_clustername_path);
    
    curr_clustername1  = clusternames{1};
    curr_clustername1_path = strcat(path1,curr_clustername1,'\');
    nvm_src = strcat(curr_clustername1_path,'outputVSFM_GB.nvm');
    nvm_dest = strcat(tmp_clustername_path,'outputVSFM_GB.nvm');
    ply_src = strcat(curr_clustername1_path,'bundler_output.ply');
    ply_dest = strcat(tmp_clustername_path,'bundler_output.ply');
    copyfile(nvm_src, nvm_dest);
    copyfile(ply_src, ply_dest);
    
    
    for i = 2:size(clusternames,1)
        curr_clustername  = clusternames{i};
        curr_clustername_path = strcat(path1,curr_clustername,'\');
        cd ..\slam_src\build\Release\

   %     merge.exe -path1=D:\slam\slam_src\data\vid5\batch_0\ -path2=D:\slam\slam_src\data\vid5\batch_1\ -path_data=D:\slam\slam_src\data\vid5\ -output_dir=D:\slam\slam_src\data\vid5\

        command_merge =strcat('merge.exe -path1=',tmp_clustername_path,' -path2=',curr_clustername_path,' -path_data=',path1,' -output_dir=',merging_clustername_path);
        system(char(command_merge));
        cd ..\..\..\scripts_trans_rotation_avg\
        
        nvm_src1 = strcat(merging_clustername_path,'combined.nvm');
        nvm_dest1 = strcat(tmp_clustername_path,'outputVSFM_GB.nvm');
        ply_src1 = strcat(merging_clustername_path,'combined.ply');
        ply_dest1 = strcat(tmp_clustername_path,'bundler_output.ply');
        copyfile(nvm_src1, nvm_dest1);
        copyfile(ply_src1, ply_dest1);
    end
end
