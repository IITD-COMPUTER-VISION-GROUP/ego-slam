function [comp,rotationfiterror,translationfiterror]= trans_avg_func_with_frob_with_oripath(path,oripath)

%num_cams =655;
% E5point = load('E5point_out.txt');
% %Error = load('Error.txt');
% pair = load('original_pairs_bundler.txt');
% T = load('T_bundler.txt');
% RR = load('R_bundler.txt');
% %focal = load('listsize_focalB.txt');
% [S,du,focal] = textread('listsize_focal1.txt','%s %d %f');




E5point = load(strcat(path,'E5point.txt'));
pair = load(strcat(path,'original_pairs5point.txt'));
T = load(strcat(path,'T5point.txt'));
RR = load(strcat(path,'R5point.txt'));
[S,du,focal] = textread(strcat(path,'listsize_focal1.txt'),'%s %d %f');




num_cams = size(S,1);

if(size(pair,1) < 4)
    comp = 0;
    rotationfiterror= [-1.0 -1.0 -1.0];
    translationfiterror= [-1.0 -1.0 -1.0];
    return;
end
maxval = -1;
for i = 1:size(pair,1)
    pair(i,4) = pair(i,4)/pair(i,3);
    pair(i,3) = 1.0;

E = reshape(E5point(i,:),3,3)';

expr = E*E'*E - 0.5*trace(E*E')*E;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fval = norm(expr,'fro');
if(Fval >=maxval)
   maxval= Fval;
end
pair(i,4) = Fval;%norm(expr,'fro');
%pair(i,4)= Error(i,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
pair(:,4) = ones(size(pair,1),1) - pair(:,4)./maxval;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ind =1;
% hard_thresh = 0.5;
% for i = 1:size(pair,1)
%     if(pair(i,4) < hard_thresh)
%         npair(ind,:) = pair(i,:);
%         nT(ind,:) = T(i,:);
%         nRR(ind,:) = RR(i,:);
%         ind =ind+1;
%     end
% end
% pair = npair;
% T = nT;
% RR = nRR;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%[S,s1] = textread('listsizeB.txt','%s %s');
 thresh = inf;% for our case it is 1.1
% 
 [pair, RR, T] = refine_with_triangle_constraint( pair, RR,T, num_cams, thresh );
[originalpairsnew1,pairsnew1,Rpairwisemapped,Tnew1,map1,inv_map1,num_cams_found]=find_connected_pairwise(pair,RR,T,num_cams);
% [originalpairsnew1,pairsnew1,Rpairwisemapped,Tnew1,map1,inv_map1,num_cams_found]=find_connected_pairwise(pairsnew1,Rpairwisemapped,Tnew1);


%%%%%%%%%%%%%%%%%%%%%%%  For Noah trans Avg
%%%%%%%%%%%%%%%%%%%%%%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fid = fopen('conn_comp.txt','w');
for i = 1:size(map1,1)
    fprintf(fid,'%d\n',map1(i));
end
fclose(fid);
fid = fopen('original_pairs_comp.txt','w');
for i = 1:size(originalpairsnew1,1)
    fprintf(fid,'%d %d %f %f\n',originalpairsnew1(i,1),originalpairsnew1(i,2),originalpairsnew1(i,3),originalpairsnew1(i,4));
end
fclose(fid);

fid = fopen('R_comp.txt','w');
fid2 = fopen('T_comp.txt','w');
for i = 1:size(Rpairwisemapped,1)
    fprintf(fid,'%f %f %f %f %f %f %f %f %f\n',Rpairwisemapped(i,1),Rpairwisemapped(i,2),Rpairwisemapped(i,3),Rpairwisemapped(i,4),Rpairwisemapped(i,5),Rpairwisemapped(i,6),Rpairwisemapped(i,7),Rpairwisemapped(i,8),Rpairwisemapped(i,9));
    fprintf(fid2,'%f %f %f\n',Tnew1(1,i),Tnew1(2,i),Tnew1(3,i));
end
fclose(fid);
fclose(fid2);


%return; % This return should be eliminated if noah's original trans_avg is not needed and our algo need to run

% generate new matchRT

% ii =1;
% fp = fopen('matches_forRT.txt','r');
% num = fscanf(fp,'%d\n',1);
% fp1 = fopen('matches_forRTA_filtered.txt','w');
% fprintf(fp1,'                     \n');
% for i = 1: num
%     line1 = fscanf(fp,'%d %d %d\n',3);
%     %size(line1)
%    % line1
%    
%    
%     if(mod(i,1000) ==0)
%         i
%     endexit

%     id1 = line1(1,1);
%     id2 = line1(2,1);
%     num_matches = line1(3,1);
%     found =0;
%     
%     if(map(id1+1)~=0 && map(id2+1)~=0)
%         fprintf(fp1,'%d %d %d\n',id1,id2,num_matches);
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%             fprintf(fp1,'%d %f %f %d %f %f\n',line2(1dddddd,1),line2(2,1),line2(3,1),line2(4,1),line2(5,1),line2(6,1));
%         end
%         ii = ii+1;
%     else
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%         end  
%     end
% 
% end
% 'ii ='
% ii
% fseek(fp1,0,'bof');
% fprintf(fp1,'%d',ii);
% fclose(fp);
% fclose(fp1);








sx = size(pairsnew1,1);
s = ones(sx,1);
%adj = sparse(pair(:,1),pair(:,2),s,sx,sx);
pair2 = pairsnew1';
pair1 = pair2(1:2,:);
R1 = Rpairwisemapped';
T = Tnew1';
[sx,sy] = size(R1);
R2 = reshape(R1,3,3,sy);
for i = 1: sy
    R2(:,:,i) = R2(:,:,i)'; %R2(:,:,i) = R2(:,:,i)' is for our convention. For noah's convention I have change it
end
size(pair1);
size(R2);

for i=1:size(R2,3)
    if(det(R2(:,:,i))<=0)
        error('det(RR(:,:,%d))=%f\n',i,det(R2(:,:,i)))
    end
    [U,S1,V]=svd(R2(:,:,i));
    if(any(abs(diag(S1)-1)>=.1))
        error('svd(RR(:,:,%d))=[%f %f %f]\n',i,S1(1,1),S1(2,2),S1(3,3));
    elseif(any(abs(diag(S1)-1)>=.01))
        warning('svd(RR(:,:,%d))=[%f %f %f]\nProjecting to 1 s\n',i,S1(1,1),S1(2,2),S1(3,3));
    end
    R2(:,:,i)=U*round(S1)*V';
end




[Rout1] =  BoxMedianSO3Graph(R2,pair1);
[Rout2] =  RobustMeanSO3Graph(R2,pair1,5,Rout1);
[E e]=ValidateSO3Graph(Rout2,R2,pair1);
rotationfiterror = E;

for i = 1: size(T,1)
    k= pairsnew1(i,2);
    Cij(i,:) = -1*Rout2(:,:,k)'*T(i,:)';
end
%E = load('error.txt');
thres = 180;
% ii=1;
% for i1 =1: size(originalpairsnew1,1)
% %     if(originalpairsnew1(i1,1) == 100 || originalpairsnew1(i1,1) == 101 || originalpairsnew1(i1,1) == 102 || originalpairsnew1(i1,2) == 100 || originalpairsnew1(i1,2) == 101 || originalpairsnew1(i1,2) == 102)
% %         continue;
% %     end
%     if (e(i1) <= thres)
%         
%         %fprintf(f1,'%d %d\n',Pair(i,1),Pair(i,2));
%         %fprintf(f2,'%f %f %f %f %f %f %f %f %f\n',R(i,1),R(i,2),R(i,3),R(i,4),R(i,5),R(i,6),R(i,7),R(i,8),R(i,9));
%         newpairs(ii,:) =  originalpairsnew1(i1,:);
%         newtrans(ii,:) = Cij(i1,:);
%         newrot(:,:,ii) = R2(:,:,i1);
%         ii = ii+1;
%     end
% end
[newpairs, newrot, newtrans] = refine_with_triangle_constraintafterRA( originalpairsnew1, R2,Cij, num_cams,e, thres );


% map global rotations
Rglobal = zeros(3,3,num_cams_found);

for i =1: size(Rout2,3)
    Rglobal(:,:,inv_map1(i)) = Rout2(:,:,i);
end
Rout2 = Rglobal;


% generate new matchRT

% ii =1;
% fp = fopen('matches_forRT.txt','r');
% num = fscanf(fp,'%d\n',1);
% fp1 = fopen('matches_forRTA_filtered.txt','w');
% fprintf(fp1,'                     \n');
% for i = 1: num
%     line1 = fscanf(fp,'%d %d %d\n',3);
%     %size(line1)
%    % line1
%    
%    
%     if(mod(i,1000) ==0)
%         i
%     end
%     id1 = line1(1,1);
%     id2 = line1(2,1);
%     num_matches = line1(3,1);
%     found =0;
%     for i1 = 1: size(newpairs,1)
%         if((newpairs(ii,1) == (id1+1) && newpairs(ii,2) == (id2+1))||(newpairs(ii,2) == (id1+1) && newpairs(ii,1) == (id2+1)))
%             found =1;
%             break;
%         end
%     end
%     if(found ==1)
%         fprintf(fp1,'%d %d %d\n',id1,id2,num_matches);
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%             fprintf(fp1,'%d %f %f %d %f %f\n',line2(1,1),line2(2,1),line2(3,1),line2(4,1),line2(5,1),line2(6,1));
%         end
%         ii = ii+1;
%     else
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%         end  
%     end
% 
% end
% 'ii ='
% ii
% fseek(fp1,0,'bof');
% fprintf(fp1,'%d',ii);
% fclose(fp);
% fclose(fp1);
% 
% 



% pair2 = newpairs';
% pair1 = pair2(1:2,:);
% [Rout1] =  BoxMedianSO3Graph(newrot,pair1);
% [Rout2] =  RobustMeanSO3Graph(newrot,pair1,5,Rout1);
% [E e]=ValidateSO3Graph(Rout2,newrot,pair1);
[originalpairsnew,pairsnew,Rnew,Tnew,map,inv_map,comp]=find_connected_returned_comp(newpairs,Rout2,newtrans,num_cams);


% generate new matchRT

if(comp > 1)
    ii =1;
    fp = fopen(strcat(path,'matches_forRtinlier5point.txt'),'r');
    num = fscanf(fp,'%d\n',1);
    fp1 = fopen(strcat(path,'matches_forRT_filtered.txt'),'w');
    fprintf(fp1,'                     \n');
    for i = 1: num
        line1 = fscanf(fp,'%d %d %d\n',3);
        %size(line1)
       % line1


        if(mod(i,1000) ==0)
            i
        end
        id1 = line1(1,1);
        id2 = line1(2,1);
        
        num_matches = line1(3,1);
        found =0;

        if(map(id1+1)~=0 && map(id2+1)~=0)
            fprintf(fp1,'%d %d %d\n',id1,id2,num_matches);
            for j =1:num_matches
                line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
                fprintf(fp1,'%d %f %f %d %f %f\n',line2(1,1),line2(2,1),line2(3,1),line2(4,1),line2(5,1),line2(6,1));
            end
            ii = ii+1;
        else
            for j =1:num_matches
                line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
            end  
        end

    end
    'ii ='
    ii
    fseek(fp1,0,'bof');
    fprintf(fp1,'%d',ii-1);
    fclose(fp);
    fclose(fp1);

else
    file1 = strcat(path,'matches_forRtinlier5point.txt');
    filedst = strcat(path,'matches_forRT_filtered.txt');
    copyfile(file1,filedst );
end




newtrans =Tnew';
 newpairs = pairsnew';

 hcam = max(max(newpairs(1:2,:)));
adjj = zeros(hcam,hcam);

for i = 1:size(newpairs,2)
    
        adjj(newpairs(1,i),newpairs(2,i)) = 1;
end

histt = zeros(size(adjj,1),1);
histtT = zeros(size(adjj,1),1);
for i = 1:size(adjj,1)
    histt(i)=sum(adjj(i,:));
     histtT(i)=sum(adjj(:,i));
end
 
 
 %%%%%%%%%%%%%%%%%%%%%%%% Dumping data for noah code%%%%%%%%%%%%%%%%%%%%%%%%
 
 save(strcat(path,'map.mat'),'map');
 save(strcat(path,'inv_map.mat'),'inv_map');
 save(strcat(path,'Rnew.mat'),'Rnew');
 fid = fopen('TTA.txt','w');
 fid1 = fopen('IA.txt','w');
 %fidnewpairs = fopen('original_pairs_filtered.txt','w');
 fprintf(fid1,'%d\n',size(newpairs,2));
 
 for i = 1:size(newtrans,1)
    fprintf(fid,'%f %f %f\n',newtrans(i,1),newtrans(i,2),newtrans(i,3));
    fprintf(fid1,'%f %f\n',newpairs(1,i)-1,newpairs(2,i)-1);
   % fprintf(fidnewpairs,'%f %f %f %f\n',newpairs(1,i)-1,newpairs(2,i)-1,newpairs(3,i),newpairs(4,i));
    
 end
 fclose(fid);
 fclose(fid1);
 %fclose(fidnewpairs);
 
 
 
 
 %%%%%%% read tracks %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%[orilistnames,orifocal] = textread('./list_focal.txt','%s %f');
[orilistnames,orifocal] = textread(strcat(oripath,'list_focal.txt'),'%s %f');
num_original_cams = size(orilistnames,1);
[tracksall,tracksallsize] = track_read(strcat(path,'world1.txt'),num_original_cams,strcat(path,'cluster_list_map.txt'),map);

num_cams_comp = size(inv_map,1);

%%%%%%%%%%%%%%Find coherent 3d points%%%%%%%%%%%%%%%%%%%%
 
%  [extraTrans,extrapairs,num_points_added] = find_coherent_3d_points_forTranslationavg(Rnew,num_cams_comp,focal,tracksall,tracksallsize,inv_map);
    nummin3dpoints=20;
  [extraTrans,extrapairs,num_points_added] = find_coherent_3d_points_forTranslationavgNpoints(Rnew,num_cams_comp,focal,tracksall,tracksallsize,inv_map,nummin3dpoints);

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 newtrans1 = newtrans';
 newtrans1 = [newtrans1 extraTrans];
 
 
 
 avipairs = newpairs(1:2,:);
 
 avipairs1 = [avipairs extrapairs];
 
 Transf3 = RobustMeanDirectionGraph(newtrans1,avipairs1);
%  Transf3 = RobustMeanDirectionGraph(newtrans',avipairs);
 Transf2 = Transf3';
 
%  system('ceres_test2.exe IA.txt TTA.txt');
% Transf2 = load('output.txt'); 

 
 for i = 1:size(Transf2,1)
     Transf2(i,:) = Transf2(i,:) - Transf2(1,:);
 end

 
 Transf2 = Transf2(1: num_cams_comp,:);
 
 save(strcat(path,'output.txt'),'Transf2','-ascii');

 
%  ii =1;
%     max1 = 0;
%     fpp = fopen(strcat(path,'error.txt'),'w');
%     fp = fopen(strcat(path,'matches_forRT.txt'),'r');
%     num = fscanf(fp,'%d\n',1);
%     fp1 = fopen(strcat(path,'matches_forRT_filtered.txt'),'w');
%     fprintf(fp1,'                     \n');
%     for i = 1: num
%         line1 = fscanf(fp,'%d %d %d\n',3);
%         %size(line1)
%        % line1
% 
% 
%         if(mod(i,1000) ==0)
%             i
%         end
%         id1 = line1(1,1);
%         id2 = line1(2,1);
%         
%         Rj = Rout2(:,:,id2+1);
%         tij = Rj*(Transf2(id1 + 1,:) - Transf2(id2 + 1,:))';
%         Rij = Rj * Rout2(:,:,id1+1)';
%         
%        Tijx = [0 -tij(3) tij(2); tij(3) 0 -tij(1); -tij(2) tij(1) 0];
%        
%        E = Tijx * Rij;
%         f1 = focal(id1 + 1);
%         f2 = focal(id2+1);
%         
%        k1 = [f1 0 0; 0 f1 0; 0 0 1];
%        k2 = [f2 0 0; 0 f2 0; 0 0 1];
%        F = inv(k2')*E*inv(k1);
%        
%         num_matches = line1(3,1);
%         found =0;
% 
%         if(map(id1+1)~=0 && map(id2+1)~=0)
%             
%             match_inlier_ind=0;
%             inliers_F = [];
%             for j =1:num_matches
%                 
%                
%                 line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%                 
%                  x2 = [line2(5,1);line2(6,1);1];
%                  x1 = [line2(2,1);line2(3,1);1];
%                  
%                  val = x2'*F*x1;
%                  val = abs(val);
%                  
%                  if(max1 < val)
%                      max1 = val;
%                  end
%                  
%                  fprintf(fpp,'%f\n',val);
%                  if(val < 0.0005)
%                     inliers_F(match_inlier_ind+1,:) = line2;
%                     match_inlier_ind=match_inlier_ind+1;
%                     %fprintf(fp1,'%d %f %f %d %f %f\n',line2(1,1),line2(2,1),line2(3,1),line2(4,1),line2(5,1),line2(6,1));
%                  end
%             end
%             
%             
%             if(size(inliers_F,1) > 1)
%                 fprintf(fp1,'%d %d %d\n',id1,id2,size(inliers_F,1));
%                 fprintf(fpp,'%d %d\n',id1,id2);
%                 for j =1:size(inliers_F,1)
%                     
%                     fprintf(fp1,'%d %f %f %d %f %f\n',inliers_F(j,1),inliers_F(j,2),inliers_F(j,3),inliers_F(j,4),inliers_F(j,5),inliers_F(j,6));
%                 end
%                 ii = ii+1;    
%             end
%             clear inliers_F;
%         else
%             for j =1:num_matches
%                 line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%             end  
%         end
% 
%     end
%     'ii ='
%     ii
%     fseek(fp1,0,'bof');
%     fprintf(fp1,'%d',ii-1);
%     fclose(fp);
%     fclose(fp1);
%     fclose(fpp);
% max1
% else
%     file1 = strcat(path,'matches_forRT.txt');
%     filedst = strcat(path,'matches_forRT_filtered.txt');
%     copyfile(file1,filedst );
% end
%  
 
 
 
 
 
 
 
 %Writing camera_track format for running the data dump code "Avishek".
 fid = fopen(strcat(path,'cameratrack.txt'),'w');
 
 for i =1: size(Rnew,3)
   % Rglobal(:,:,inv_map(i)) = Rnew(:,:,i);
    %Transglobal(inv_map(i),1:3) = Transf2(i,1:3);
    quar = R2q(Rnew(:,:,i));
    
    oriIndex = inv_map(i);
    %names_avg = [names_avg;S{oriIndex}];
    fprintf(fid,'%s %f %f %f %f %f %f %f %f %f %f\n', S{oriIndex},1000,quar(1), quar(2), quar(3), quar(4),Transf2(i,1), Transf2(i,2), Transf2(i,3), 0,0);
    %dataformat_avg = [dataformat_avg;[ S{oriIndex} 1000 quar(1) quar(2) quar(3) quar(4) Transf2(i,1) Transf2(i,2) Transf2(i,3) 0] ];
    
end
 fclose(fid);
 
 
 
 
 
 
 
 
 
 
% return;
 
 
 
 %[translation] = iterativeReweightedLeastSquareFast1( newtrans,newpairs );
 
 
 
% Transf = zeros(size(Rnew,3),9);
%Transf(2:end,1:3) = translation;





for i = 1:size(newpairs,2)
     l= newpairs(1,i);
     k= newpairs(2,i);
   %  i
  
%      vec = Transf(k,1:3) - Transf(l,1:3);
%      vec  = vec./norm(vec);
%      
     vec2(i,:) = Transf2(k,1:3) - Transf2(l,1:3);
     if(norm(vec2(i,:)) == 0)
        vec22(i,:)  = vec2(i,:);
     else
        vec22(i,:)  = vec2(i,:)./norm(vec2(i,:));
     end
     
     
%      d = dot(vec,newtrans(i,:));
%      dot1 = d/(norm(vec)*norm(newtrans(i,:)));
%      degreetrans(i) = acos(dot1)*180/3.14;
%      degreetrans(i) = real(degreetrans(i));
     
     d = dot(vec22(i,:),newtrans(i,:));
     if(norm(vec22(i,:)) < 0.0001)
         dot2(i) = 1;
     else
         dot2(i) = d/(norm(vec22(i,:))*norm(newtrans(i,:)));
     end
     
     
     degreetrans2(i) = acos(dot2(i))*180/3.14;
     degreetrans2(i) = real(degreetrans2(i));
end
 names_avg = {};
 dataformat_avg = [];
 fid = fopen(strcat(path,'cameratrack_trans_avg.txt'),'w');
for i =1: size(Rnew,3)
    Rglobal(:,:,inv_map(i)) = Rnew(:,:,i);
    Transglobal(inv_map(i),1:3) = Transf2(i,1:3);
    
    oriIndex = inv_map(i);
    names_avg = [names_avg;S{oriIndex}];
    fprintf(fid,'%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',S{oriIndex},1000,Rnew(1,1,i), Rnew(1,2,i), Rnew(1,3,i), Rnew(2,1,i), Rnew(2,2,i), Rnew(2,3,i), Rnew(3,1,i), Rnew(3,2,i), Rnew(3,3,i), Transf2(i,1), Transf2(i,2), Transf2(i,3),0,0);
    dataformat_avg = [dataformat_avg;[ 1000 Rnew(1,1,i), Rnew(1,2,i) Rnew(1,3,i) Rnew(2,1,i) Rnew(2,2,i) Rnew(2,3,i) Rnew(3,1,i) Rnew(3,2,i) Rnew(3,3,i) Transf2(i,1) Transf2(i,2) Transf2(i,3)] ];
    
end
fclose(fid);
translationfiterror=[0 0 0];
return;
R= [    1.0000    0.00    0.00;
   0.00    1.0000   0.00;
   0.00   0.00    1.00];
T=[  0.00    0.00  0.00];
scalea_b =     1.00;
cameratrack_gt= 'cameratrackGT.txt';
[namesgt, Rcgt] = cameraDumporiginal(cameratrack_gt,R,T,scalea_b);
extract_same_images_to_compare(names_avg, namesgt,  dataformat_avg,Rcgt); % 1 ours 2 Ground Truth
filematchedours = 'ours_matched.txt';
filematchedGT = 'ground_truthmatched.txt';
find_error_RC_with_path(path,filematchedGT, filematchedours);
translationfiterror = [mean(degreetrans2) median(degreetrans2) sqrt((degreetrans2*degreetrans2')/size(degreetrans2,2))];


sqrt(degreetrans*degreetrans'/size(degreetrans,2))
ind=1;
 for i1 = 1: size(newpairs,2)
        if(degreetrans(i1) < 25)
             newpairs11(:,ind) = newpairs(:,i1);
             newpairs1(ind,:) = originalpairsnew(i1,:);
             newtrans1(ind,:) = newtrans(i1,:);
             ind = ind+1;
        end
 end
 
 
 fid = fopen('TTA.txt','w');
 fid1 = fopen('IA.txt','w');
 
 fprintf(fid1,'%d\n',size(newpairs11,2));
 
 for i = 1:size(newtrans1,1)
    fprintf(fid,'%f %f %f\n',newtrans1(i,1),newtrans1(i,2),newtrans1(i,3));
    fprintf(fid1,'%f %f\n',newpairs11(1,i)-1,newpairs11(2,i)-1);
    
 end
 fclose(fid);
 fclose(fid1);
 
 system('ceres_test2.exe IA.txt TTA.txt');
 
 Transf2 = load('output.txt'); 
 
  names_avg = {};
 dataformat_avg = [];
for i =1: size(Rnew,3)
    Rglobal(:,:,inv_map(i)) = Rnew(:,:,i);
    Transglobal(inv_map(i),1:3) = Transf2(i,1:3);
    
    oriIndex = inv_map(i);
    names_avg = [names_avg;S{oriIndex}];
    dataformat_avg = [dataformat_avg;[ 1000 Rnew(1,1,i), Rnew(1,2,i) Rnew(1,3,i) Rnew(2,1,i) Rnew(2,2,i) Rnew(2,3,i) Rnew(3,1,i) Rnew(3,2,i) Rnew(3,3,i) Transf2(i,1) Transf2(i,2) Transf2(i,3)] ];
    
end

extract_same_images_to_compare(names_avg, namesgt,  dataformat_avg,Rcgt); % 1 ours 2 Ground Truth
filematchedours = 'ours_matched.txt';
filematchedGT = 'ground_truthmatched.txt';
find_error_RC(filematchedGT, filematchedours);
 
 
 
 
 
 
  [translation] = iterativeReweightedLeastSquareFast1( newtrans1,newpairs11 );
  Transf(2:end,1:3) = translation;
  
  
  clear degreetrans;
  for i = 1:size(newpairs11,2)
     l= newpairs11(1,i);
     k= newpairs11(2,i);
   %  i
     vec = Transf(k,1:3) - Transf(l,1:3);
     vec  = vec./norm(vec);
     d = vec*newtrans1(i,:)';
     dot1 = d/(norm(vec)*norm(newtrans1(i,:)));
     degreetrans(i) = acos(dot1)*180/3.14;
end
 
sqrt(degreetrans*degreetrans'/size(degreetrans,2))
% [originalpairsnew,pairsnew,Rnew,Tnew,map,inv_map]=find_connected(newpairs1,Rout2,newtrans1);

% generate new matchRT

% ii =1;
% fp = fopen('matches_forRTB.txt','r');
% num = fscanf(fp,'%d\n',1);
% fp1 = fopen('matches_forRTB_filtered.txt','w');
% for i = 1: num
%     line1 = fscanf(fp,'%d %d %d\n',3);
%    % size(line1)
%    %line1
%    
%    
%     if(mod(i,1000) ==0)
%         i
%     end
%     id1 = line1(1,1);
%     id2 = line1(2,1);
%     num_matches = line1(3,1);
%     found =0;
%     loc =-1;
%     for i1 = 1: size(newpairs1,1)
%         if((newpairs1(i1,1) == (id1+1) && newpairs1(i1,2) == (id2+1))||(newpairs1(i1,2) == (id1+1) && newpairs1(i1,1) == (id2+1)))
%             found =1;
%             loc =i1;
%             break;
%         end
%     end
%     if(found ==1 )
%         fprintf(fp1,'%d %d %d\n',map(id1+1)-1,map(id2+1)-1,num_matches);
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%             fprintf(fp1,'%d %f %f %d %f %f\n',line2(1,1),line2(2,1),line2(3,1),line2(4,1),line2(5,1),line2(6,1));
%         end
%         ii = ii+1;
%     else
%         for j =1:num_matches
%             line2 = fscanf(fp,'%d %f %f %d %f %f\n',6);
%         end  
%     end
% 
% end
% 'ii ='
% ii
% fclose(fp);
% fclose(fp1);


%now lets map back to original coordinates

Rglobal = zeros(3,3,num_cams);
Transglobal = zeros(num_cams,9);
for i =1: size(Rnew,3)
    Rglobal(:,:,inv_map(i)) = Rnew(:,:,i);
    Transglobal(inv_map(i),1:3) = Transf(i,1:3);
end
Transf1 = Transf;
translation = Transglobal(2:end,1:3);
Transf = Transglobal;
Rout2 = Rglobal;

 %[translation1] = MeanDirectionGraph( Tnew,newpairs(1:2,:) );
 %cameratrack = load('listsize_focal_camtrackB.txt');
 
 [cameratrack] = cameraTrack2listReorganisation('cameratrack1.txt',S)

[errorRot,degree, dist] =error_deg_plot(cameratrack,translation,Rglobal);
mean(degree)
median(degree)
mean(dist)
median(dist)
[sx,sy,sz] = size(Rout2);
newameraTrack = zeros(sz,10);
fcamtrack = fopen('newCameraTrackB.txt','w');
for i = 1:sz
    Rout22(:,:,i) = Rout2(:,:,i)';
    qR = R2q(Rout2(:,:,i));
    if(i == 1)
%        newameraTrack(i,:) = [focal(i) qR(i,:) 0 0 0 0 0 ];
         fprintf(fcamtrack,'%s %f %f %f %f %f %f %f %f %f %f\n',S{i},focal(i),qR(1),qR(2),qR(3),qR(4),0.0, 0.0, 0.0, 0.0, 0.0);
    else
%        newameraTrack(i,:) = [focal(i) qR(i,:) translation(i-1,:) 0 0];
         fprintf(fcamtrack,'%s %f %f %f %f %f %f %f %f %f %f\n',S{i},focal(i),qR(1),qR(2),qR(3),qR(4), translation(i-1,1),  translation(i-1,2), translation(i-1,3), 0.0, 0.0);
    end
end
fclose(fcamtrack);
Rout22 = reshape(Rout22,9,sz);
Rout22 = Rout22';

Rt = zeros(2*size(Rout22,1),size(Rout22,2));
Rt(1:2:end,:) = Rout22;
Rt(2:2:end,:) = Transf;
save('RTBvsfm.txt','Rt','-ascii');


for i = 1:size(Rnew,3)
    Rout222(:,:,i) = Rnew(:,:,i)';
end
Rout222 = reshape(Rout222,9,size(Rout222,3))';
%Rout22 = Rout22';

Rt1 = zeros(2*size(Rout222,1),size(Rout222,2));
Rt1(1:2:end,:) = Rout222;
Rt1(2:2:end,:) = Transf1;
save('RTBvsfmmapped.txt','Rt1','-ascii');

fp = fopen('listfocalB_mapped.txt','w');
for i =1:size(inv_map,1)
    fprintf(fp,'%s %f\n',S{inv_map(i)},focal(inv_map(i)));
end
fclose(fp);
end

% write Rt and new list.txt


%save('newCameraTrackB.txt','newameraTrack','-ascii');


