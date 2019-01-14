function [pairout, rout, tout] = refine_with_triangle_constraintafterRA( pair, R,t, num_cams,E, thresh )

%thresh = 1.0 / thresh;
filterout = find(E(:) > thresh);
refinepairs = find(E(:) <= thresh);
pairin = pair(refinepairs,:);
pairout = pair(refinepairs,:);
rout = R(:,:,refinepairs);
tout = t(refinepairs,:);
removed = pair(filterout,:);
removedR = R(:,:,filterout);
removedT = t(filterout,:);
lastindadded = size(pairin,1);
A = sparse(pairin(:,1), pairin(:,2),1,num_cams,num_cams);
A = full(A);
for i =1: num_cams
    for j =1:num_cams
        if(A(i,j)==1)
            A(j,i)=1;
        else
            if(A(j,i)==1)
                A(i,j)=1;
            end
        end
        
        if (i==j)
            A(i,j)=0;
        end
    end
end
cnt=1;

%create a flag matrix for each node denoting if its a aprt of a triangle or
%not

tripletflag = zeros(num_cams,1);

for i =1:num_cams
    adjnodes = find(A(i,:)==1);
    adjnodes = adjnodes';
    for j =1:size(adjnodes,1)
        f1 = find(A(adjnodes(j),adjnodes)==1);
        if(size(f1,2) > 0)
            tripletflag(i)=1;
            break;
        end
    end
end





clear f1;



for i = 1: size(removed,1)
    ind1 = removed(i,1);
    ind2 = removed(i,2);
    
    f1 = find(A(ind1,:)==1);
    f2 = find(A(ind2,f1)==1);
    if(size(f2,2) > 0)
        broken=0;
        for k = 1: size(f2,2)
            if(tripletflag(f2(1,k))==0)
                broken =1;
                break;
            end
        end
        if(broken==1)
            pairout(lastindadded+1,:) = removed(i,:);
            rout(:,:,lastindadded+1) = removedR(:,:,i);
            tout(lastindadded+1,:) = removedT(i,:);
            lastindadded = lastindadded+1;
            cnt= cnt+1;
        end
    end
end
cnt
