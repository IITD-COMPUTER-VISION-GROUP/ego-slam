% clearvars;
function RunBundler(oripath)
%clusternames = textread('clusternames.ini','%s');
%oripath = sprintf('H:\\suvam\\expts_depth\\test\\');
clusternames = textread(strcat(oripath,'clusternames.ini'),'%s');

command = sprintf('rm -f bundlog.txt');
system(command);

inner_iter = 4;
outer_iter = 200;
for i = 1:size(clusternames,1)
    curr_clustername_with_ext  = clusternames{i};
    curr_clustername = curr_clustername_with_ext;
    command = sprintf('TestBundler_new_5params.exe %s%s\\ %d %d 16 32 2',oripath,curr_clustername,inner_iter,outer_iter);    
    system(command);
end

command = sprintf('cp bundlog.txt %sbundlog.txt', oripath);
tic;
system(command)
toc;
end