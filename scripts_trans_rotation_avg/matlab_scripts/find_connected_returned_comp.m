function [originalpairsnew,pairsnew,Rglobalmapped,Tnew,map,inv_map,s]=find_connected_returned_comp(p,R,T,num_cams)



pnew = zeros(size(p));
for i =1:size(p,1)
    if(p(i,1) < p(i,2))
        pnew(i,1) = p(i,2);
        pnew(i,2) = p(i,1);
         pnew(i,3) = p(i,3);
         pnew(i,4) = p(i,4);
    else
        pnew(i,:) = p(i,:);
    end
end
m1 = num_cams;
g = sparse(pnew(:,1),pnew(:,2),true,m1,m1);
[s,c] = graphconncomp(g,'DIRECTED','false');
s
if(s~=1)
    max_comp=1;
    max_size=0;
   for i =1: s
       h = size(find(c==i),2);
       if(h>max_size)
           max_comp =i;
           max_size=h;
       end
   end     
   max_comp
   max_comp_cams = find(c==max_comp);
   map = zeros(m1,1);
   inv_map = zeros(max_size,1);
  % map(:)=-1
   ind =1;
   for i =1:size(max_comp_cams,2)
        inv_map(ind,1)= max_comp_cams(1,i);
        map(max_comp_cams(1,i),1) =ind;
        ind =ind+1;
   end
   ind=1;
   for i=1: size(p,1)
        if(map(p(i,1),1)~=0 && map(p(i,2),1)~=0)
            originalpairsnew(ind,1) = p(i,1);
            originalpairsnew(ind,2) = p(i,2);
            pairsnew(ind,1) = map(p(i,1),1);
            pairsnew(ind,2) = map(p(i,2),1);
            pairsnew(ind,3) = p(i,3);
            pairsnew(ind,4) = p(i,4);
        %    Rnew(:,:,ind) = R(:,:,i);
            Tnew(:,ind) = T(i,:);
            
            ind =ind+1;
        end
   end
   size(max_comp_cams,2)
   max_size
   Rglobalmapped  = zeros(3,3,size(max_comp_cams,2));
   for i =1: size(max_comp_cams,2)
       Rglobalmapped(:,:,i) = R(:,:,inv_map(i,1));
      % Rglbalmap(i,1) =inv_map(i,1);
   end
   %[Rout1 E e]=rot_avg(Rnew,pairsnew);
    
else
    Rglobalmapped = R;
    originalpairsnew =p;
    pairsnew = p; 
    Tnew = T';
    m1 = max(p(:));
    map= (1:1:m1)';
    inv_map = (1:1:m1)';
   %[Rout E e]=rot_avg(R,pair); 
end 