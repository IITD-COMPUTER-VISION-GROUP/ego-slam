function  [extraTrans,extrapairs,num_points_added] = find_coherent_3d_points_forTranslationavgNpoints(R,num_cams_comp,focal,track,tracksizewisesorted,inv_map,num3dpts)

cams_used_flag = zeros(num_cams_comp,1);
cams_used_count = zeros(num_cams_comp,1);
index3d=num_cams_comp+1;
indexcur = 1;
for i =1: size(tracksizewisesorted,1)
%     if(size(find(cams_used_flag==1),1) == num_cams_comp)
%         break;
%     end
	
    if(size(find(cams_used_count>=num3dpts),1) == num_cams_comp)
        break;
    end
    numcams =  track{tracksizewisesorted(i),1}(1);
    cams_used_from_current=0;
    if(i==1644)
        'hi';
    end
    % for j =1: numcams
        % %j
        % cloc = 1+ (j-1)*4;
        % if(cams_used_flag(track{tracksizewisesorted(i),1}(cloc + 1))==1)
            % cams_used_from_current =cams_used_from_current+1;
        % end
    % end
	
    for j =1: numcams
        %j
        cloc = 1+ (j-1)*4;
        if(cams_used_count(track{tracksizewisesorted(i),1}(cloc + 1))>=num3dpts)
            cams_used_from_current =cams_used_from_current+1;
        end
    end
    if(numcams == cams_used_from_current)
        continue;
    end
    
    for j =1: numcams
        cloc = 1+ (j-1)*4;
        current_cam = track{tracksizewisesorted(i),1}(cloc + 1);
        ix = track{tracksizewisesorted(i),1}(cloc + 3);
        iy = track{tracksizewisesorted(i),1}(cloc + 4);
        cams_used_flag(current_cam)=1;
		cams_used_count(current_cam)=cams_used_count(current_cam)+1;
        extrapairs(1,indexcur)= current_cam;
        extrapairs(2,indexcur)= index3d;
        px = ix/focal(inv_map(current_cam));
        py = iy/focal(inv_map(current_cam));
        ray_calc = [px, py,1]';
        Rot_cur = R(:,:,current_cam);
        trans_calc = Rot_cur'*ray_calc;
        dir_calc = trans_calc./norm(trans_calc);
        
        extraTrans(1,indexcur) = dir_calc(1);
        extraTrans(2,indexcur) = dir_calc(2);
        extraTrans(3,indexcur) = dir_calc(3);
        indexcur = indexcur+1;
    end
    index3d = index3d+1;
end
num_points_added = index3d-1


end