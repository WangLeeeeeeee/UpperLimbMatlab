% �ɽǶȣ����ٶȣ��Ǽ��ٶ���ؽ�����
function torque = InverseDynamic(qLimb,dqLimb,ddqLimb)
% ��������
global mShoulder
global mElbow
% ��֫�˶�ѧ����
global LenUpperarm
global LenForeArm
global g
% �任����
T01 = DH(0,0,0,qLimb(1));
T12 = DH(0,-pi/2,0,qLimb(2)-pi/2);
T23 = DH(0,pi/2,0,qLimb(3));
T34 = DH(LenUpperarm,-pi/2,0,qLimb(4));
T45 = DH(LenForeArm,0,0,0);
% ��ʼ��
R = cell(1,6);
w = cell(1,6);
dw = cell(1,6);
f = cell(1,6);
n = cell(1,6);
dv = cell(1,6);
dvc = cell(1,6);
P = cell(1,6);
Pc = cell(1,6);
F = cell(1,6);
I = cell(1,6);
dq = cell(1,6);
ddq = cell(1,6);
N = cell(1,6);
m = cell(1,6);
% ��ת����
R{1} = T01(1:3,1:3);
R{2} = T12(1:3,1:3);
R{3} = T23(1:3,1:3);
R{4} = T34(1:3,1:3);
R{5} = T45(1:3,1:3);
dv{1} = [0;0;g]; % ��������
% ����ϵԭ��λ�ƣ���P{1}��ʾ����ϵ1������ϵ0ԭ��λ�ù�ϵ����P{2}��ʾ����ϵ2������ϵ1ԭ��λ�ù�ϵ
P{1} = [0;0;0];
P{2} = [0;0;0];
P{3} = [0;0;0];
P{4} = [LenUpperarm;0;0];
P{5} = [LenForeArm;0;0];
% ÿ���������ĵ�λ��ʸ��(Q:��ؽ��������ɶ����ģ�)
Pc{2} = [0;0;0];
Pc{3} = [0;0;0];
Pc{4} = [LenUpperarm/2;0;0];
Pc{5} = [LenForeArm/2;0;0];
% ��������
m{2} = 0;
m{3} = 0;
m{4} = mShoulder;
m{5} = mElbow;
% ������������
% �ο�http://bbs.21ic.com/upfiles/img/20079/2007916104241639.pdf����ȸ���Ĺ���
rShoulder = 0.035*1000;
rElbow = 0.030*1000;
Ish1 = mShoulder*rShoulder^2/2;
Ish2 = mShoulder*rShoulder^2/4 + mShoulder*LenUpperarm^2/3;
Iel1 = mElbow*rElbow^2/2;
Iel2 = mElbow*rElbow^2/4 + mElbow*LenForeArm^2/3;
I{2} = [0, 0, 0; 0, 0, 0; 0, 0, 0];
I{3} = [0, 0, 0; 0, 0, 0; 0, 0, 0];
I{4} = [Ish1, 0, 0; 0, Ish2, 0; 0, 0, Ish2];
I{5} = [Iel1, 0, 0; 0, Iel2, 0; 0, 0, Iel2];
% ��������ת
w{1} = [0;0;0];
dw{1} = [0;0;0];
% ��ת�ؽ�
dq{2} = [0;0;dqLimb(1)];
dq{3} = [0;0;dqLimb(2)];
dq{4} = [0;0;dqLimb(3)];
dq{5} = [0;0;dqLimb(4)];
ddq{2} = [0;0;ddqLimb(1)];
ddq{3} = [0;0;ddqLimb(2)];
ddq{4} = [0;0;ddqLimb(3)];
ddq{5} = [0;0;ddqLimb(4)];
% ĩ��ִ����û����
f{6} = [0;0;0];
n{6} = [0;0;0];
% ���Ƽ���������ٶȺͼ��ٶ�i:0:1:n
for i=1:4
    w{i+1} = R{i}.'*w{i} + dq{i+1};
    dw{i+1} = R{i}.'*dw{i} + cross(R{i}.'*w{i},dq{i+1}) + ddq{i+1};
    dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
    dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};
    F{i+1}=m{i+1}*dvc{i+1};
    N{i+1} = I{i+1}*dw{i+1} + cross(w{i+1},I{i+1}*w{i+1});
end
% ���ڵ����������� i=n:-1:1
for i=5:-1:2
    f{i} = R{i}*f{i+1}+F{i};
    n{i} = N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end
torqueShoulder1 = n{2};
torqueShoulder2 = n{3};
torqueShoulder3 = n{4};
torqueElbow = n{5};
torque = [torqueShoulder1(3);torqueShoulder2(3);torqueShoulder3(3);torqueElbow(3)];
end