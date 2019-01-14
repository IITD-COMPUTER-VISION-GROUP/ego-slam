function mapping_sequential_to_global_index(path,  num_cams)
load(strcat(path,'inv_map.mat'));
load(strcat(path,'map.mat'));
load(strcat(path,'Rnew.mat'));
%num_cams = 6514; %% specify 

filename = strcat(path,'cluster_list_map.txt');
map = load(filename);
Transf = load(strcat(path,'output.txt'));

    Rglobal = zeros(3,3,num_cams);
Transglobal = zeros(num_cams,9);
for i =1: size(Rnew,3)
    oriIndex = map(inv_map(i))+1;
    Rglobal(:,:,oriIndex) = Rnew(:,:,i);
    Transglobal(oriIndex,1:3) = Transf(i,1:3)-Transf(1,1:3);
end

fid = fopen(strcat(path,'RTglobal.txt'),'w');


for i = 1:num_cams
    
    fprintf(fid,'%f %f %f %f %f %f %f %f %f\n',Rglobal(1,1,i),Rglobal(1,2,i),Rglobal(1,3,i),Rglobal(2,1,i),Rglobal(2,2,i),Rglobal(2,3,i),Rglobal(3,1,i),Rglobal(3,2,i),Rglobal(3,3,i));
    fprintf(fid,'%f %f %f\n',Transglobal(i,1),Transglobal(i,2),Transglobal(i,3));
end
fclose(fid);

end
%%%%%% For  Noah output%%%%%%


% numcluster = 1;
% foldername = 'noah_soln';
% 
% for clus = 1:numcluster
%     rotfilename = sprintf('./%s/cluster%d/rot_solution.txt',foldername,clus);
%     tranfilename = sprintf('./%s/cluster%d/trans_problem_solution.txt',foldername,clus);
%     listfilename = sprintf('./%s/cluster%d/list.txt',foldername,clus);
%     %rot = load('./noah_soln/cluster1/rot_solution.txt');
%     %tran = load('./noah_soln/cluster1/trans_problem_solution.txt');
%     
%     rot = load(rotfilename);
%     tran = load(tranfilename);
%     
%     %[S,d,focal] = textread('./noah_soln/cluster1/list.txt','%s %f %f');
%     
%     [S,d,focal] = textread(listfilename,'%s %f %f');
%     
%     num_cams = size(focal,1);
%         Rglobal = zeros(3,3,num_cams);
%     Transglobal = zeros(num_cams,9);
%     for i =1: size(rot,1)
%         oriIndex = rot(i,1)+1;;
%         Rglobal(:,:,oriIndex) = reshape(rot(i,2:10),3,3)';
%         Transglobal(oriIndex,1:3) = tran(i,2:4)- tran(1,2:4);
%     end
% 
%     outfilename = sprintf('./%s/cluster%d/RTglobal.txt',foldername,clus);
%     
%     %fid = fopen('RTglobal.txt','w');
% 
%     fid = fopen(outfilename,'w');
% 
%     for i = 1:num_cams
% 
%         fprintf(fid,'%f %f %f %f %f %f %f %f %f\n',Rglobal(1,1,i),Rglobal(1,2,i),Rglobal(1,3,i),Rglobal(2,1,i),Rglobal(2,2,i),Rglobal(2,3,i),Rglobal(3,1,i),Rglobal(3,2,i),Rglobal(3,3,i));
%         fprintf(fid,'%f %f %f\n',Transglobal(i,1),Transglobal(i,2),Transglobal(i,3));
%     end
%     fclose(fid);
% end