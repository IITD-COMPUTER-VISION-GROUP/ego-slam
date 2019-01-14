function [names, Rc] = cameraDumporiginal(cameratrack,R,T,scalea_b)
fid = fopen(cameratrack);
a1 = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f');
fclose(fid);
names = a1{1,1};
for i = 2: size(a1,2)
  %  for j = 1:size(a1,2)-1
     a(:,i-1) = a1{1,i};
  %  end
end
cameras = a(:,6:8);
% R= [    1.0000    0.00    0.00;
%    0.00    1.0000   0.00;
%    0.00   0.00    1.00];
% T=[  0.00    0.00  0.00];
% scalea_b =     1.00;

%collosseo
%1-1 to 2
% R= [  0.8002   -0.0114    0.5997;
%    -0.0059    0.9996    0.0268;
%    -0.5998   -0.0250    0.7998];
% %T=[  1.1159   -0.2983    3.4552];
% %T=[1.2105   -0.2949    3.7407];
% T = [1.2805   -0.2949    3.4407];
% scalea_b =       0.6605;
% 
% %1-4 to 2
% R= [ -0.2544    0.0402   -0.9663;
%     0.0316    0.9990    0.0332;
%     0.9666   -0.0220   -0.2554];
% 
% T = [6.1536   -0.1362    5.1933];
% scalea_b =       0.3636;
% 
% %1-3 to 2
% R= [ -1.0000   -0.1093   -0.0293;
%    -0.1126    0.9837    0.1799;
%     0.0092    0.1819   -0.9896];
% 
% T = [1.7712   -0.7776    1.7037];
% scalea_b =       0.3375;




Ap = cameras;
Ap = Ap*scalea_b;
Ap = [Ap ones(size(Ap,1),1)];
R1 = [R T']
Ap_dash = R1*Ap';
Ap_dash = Ap_dash';
Ap_dash1 = Ap_dash(:,1:3);

Apnew = zeros(size(cameras,1),6);
Apnew(:,1:3) = Ap_dash1(:,1:3);
Apnew(:,4:6) = repmat([255 0 0],size(Apnew,1),1);
% save('camerasall.txt','Apnew','-ascii');
a2 = a(:,2:5);
for i = 1:size(a2,1)
    Rout = q2R(a2(i,:));
    Rtemp = Rout*R';
    r(i,:) = reshape(Rtemp',1,9);
end
Rc = [a(:,1) r(:,1:9) Ap_dash1(:,1:3) ];
%fp = fopen('peters_cameratracktrans.txt','w');
% for i =1: size(Rc,1)
%     fprintf(fp, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f\n', names{i}, Rc(i,1),Rc(i,2),Rc(i,3),Rc(i,4),Rc(i,5),Rc(i,6),Rc(i,7),Rc(i,8),Rc(i,9),Rc(i,10),Rc(i,11),Rc(i,12),Rc(i,13));
% end
% fclose(fp);
'hi'
%save('cameras2RC.txt','Rc','-ascii');