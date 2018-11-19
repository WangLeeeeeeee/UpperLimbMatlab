function [Jac] = Jacobi_dif(T)
% ����TΪ��[T01,T12,T23,T34,T45]
% ����΢�ֱ任�������ſɱ�
% ���裺1����������˵ı任����
%       2������ĩ�����˵������˵ı任����
%       3�����ݹؽ���ת���ؽڻ����ƶ��ؽ�д�ſɱȸ���
%       4������ת���ؽڣ�JTi = [-nx*py+ny*px; -ox*py+oy*px; -ax*py+ay*px; nz; oz; az]
%       5���������Ϊ�ڹ�������ϵ�£���ת��Ϊ������ϵ�£�J = [R50,0;0,R50]*JT
% syms nx ny nz ox oy oz ax ay az px piy pz;
% Ti = [nx ox ax px;
%        ny oy ay piy;
%        nz oz az pz;
%        0 0 0 1];
% Ji = [-nx*piy+ny*px;-ox*piy+oy*px;-ax*piy+ay*px;nz;oz;az];
% T45 = T(:,:,5);
% J4 = subs(Ji,Ti(1:3,:),T45(1:3,:));
% T35 = T(:,:,4)*T45;
% J3 = subs(Ji,Ti(1:3,:),T35(1:3,:));
% T25 = T(:,:,3)*T35;
% J2 = subs(Ji,Ti(1:3,:),T25(1:3,:));
% T15 = T(:,:,2)*T25;
% J1 = subs(Ji,Ti(1:3,:),T15(1:3,:));
% JT = [J1,J2,J3,J4];
% T05 = T(:,:,1)*T15;
% R50 = T05(1:3,1:3);
% Jac = ([R50,zeros(3);zeros(3),R50])*JT;

T45 = T(:,:,5);
nx = T45(1,1);
ny = T45(2,1);
nz = T45(3,1);
ox = T45(1,2);
oy = T45(2,2);
oz = T45(3,2);
ax = T45(1,3);
ay = T45(2,3);
az = T45(3,3);
px = T45(1,4);
piy = T45(2,4);
pz = T45(3,4);
J4 = [-nx*piy+ny*px;-ox*piy+oy*px;-ax*piy+ay*px;nz;oz;az];
T35 = T(:,:,4)*T45;
nx = T35(1,1);
ny = T35(2,1);
nz = T35(3,1);
ox = T35(1,2);
oy = T35(2,2);
oz = T35(3,2);
ax = T35(1,3);
ay = T35(2,3);
az = T35(3,3);
px = T35(1,4);
piy = T35(2,4);
pz = T35(3,4);
J3 = [-nx*piy+ny*px;-ox*piy+oy*px;-ax*piy+ay*px;nz;oz;az];
T25 = T(:,:,3)*T35;
nx = T25(1,1);
ny = T25(2,1);
nz = T25(3,1);
ox = T25(1,2);
oy = T25(2,2);
oz = T25(3,2);
ax = T25(1,3);
ay = T25(2,3);
az = T25(3,3);
px = T25(1,4);
piy = T25(2,4);
pz = T25(3,4);
J2 = [-nx*piy+ny*px;-ox*piy+oy*px;-ax*piy+ay*px;nz;oz;az];
T15 = T(:,:,2)*T25;
nx = T15(1,1);
ny = T15(2,1);
nz = T15(3,1);
ox = T15(1,2);
oy = T15(2,2);
oz = T15(3,2);
ax = T15(1,3);
ay = T15(2,3);
az = T15(3,3);
px = T15(1,4);
piy = T15(2,4);
pz = T15(3,4);
J1 = [-nx*piy+ny*px;-ox*piy+oy*px;-ax*piy+ay*px;nz;oz;az];
JT = [J1,J2,J3,J4];
T05 = T(:,:,1)*T15;
R50 = T05(1:3,1:3);
Jac = ([R50,zeros(3);zeros(3),R50])*JT;
end