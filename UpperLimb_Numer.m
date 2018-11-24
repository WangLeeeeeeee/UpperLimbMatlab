clc;
clear;
%% 上肢运动学参数
LenUpperarm = 0.3035*1000;
LenForeArm = 0.135*1000;

%% 绳索节点坐标
% 基座上六个绳索节点（相对于0坐标系）
B10 = [-0.142,-0.030,0.033]*1000;
B20 = [-0.142,0,0.033]*1000;
B30 = [-0.006,0.142,0.033]*1000;
B40 = [0.006,0.142,0.033]*1000;
B50 = [0.142,0,0.033]*1000;
B60 = [0.142,-0.030,0.033]*1000;
BArc0 = [0,0.142,0.033]*1000; % 为了绘制圆弧
% 上臂四个绳索节点（相对于3坐标系）
U13 = [0.1715,-0.030,0.125]*1000;
U23 = [0.1715,0,0.125]*1000;
U33 = [0.1715,0,-0.125]*1000;
U43 = [0.1715,-0.030,-0.125]*1000;
U53 = [0.1715,0,0.125]*1000;
UArc3 = [0.1715,0.125,0]*1000; % 为了绘制圆弧
% 前臂两个绳索节点（相对于4坐标系）
F14 = [0.118,-0.118,0]*1000;
F24 = [0.118,0.118,0]*1000;
FArc4 = [0.118,0,0.118]*1000;

% %% 遍历工作空间
% the1_start = -50*pi/180;
% the1_end = 50*pi/180;
% the2_start = -45*pi/180;
% the2_end = 45*pi/180;
% the3_start = 0*pi/180;
% the3_end = 45*pi/180;
% the4_start = 0*pi/180;
% the4_end = 70*pi/180;
% space = zeros(3,10000000);
% i = 0;
% for the1_sp = the1_start:0.01:the1_end
%     for the2_sp = the2_start:0.01:the2_end
%         for the3_sp = the3_start:0.5:the3_end
%             for the4_sp = the4_start:0.5:the4_end
%                 i = i + 1;
%                 T01 = DH(0,0,0,the1_sp);
%                 T12 = DH(0,-pi/2,0,the2_sp-pi/2);
%                 T23 = DH(0,pi/2,0,the3_sp);
%                 T34 = DH(LenUpperarm,-pi/2,0,the4_sp);
%                 T45 = DH(LenForeArm,0,0,0);
%                 T05 = T01*T12*T23*T34*T45;
%                 space(1,i) = T05(1,4);
%                 space(2,i) = T05(2,4);
%                 space(3,i) = T05(3,4);
%             end
%         end
%     end
% end
% figure();
% scatter3(space(1,:), space(2,:), space(3,:),'b','filled');
% title('末端工作空间');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% grid on;
% hold on;

%% 关节空间轨迹规划
% 设定初始角度，终止角度，运行时间
% qStart = [0,0,0,0]*pi/180;
% qEnd = [10,20,20,60]*pi/180;
% qTf = 5;
% Theta = zeros(4,qTf/0.05+1);
% Omega = zeros(4,qTf/0.05+1);
% Beta = zeros(4,qTf/0.05+1);
% Pos = zeros(3,qTf/0.05+1);
% i = 0;
% for t=0:0.05:qTf
%     i = i + 1;
%     [Theta(1,i),Omega(1,i),Beta(1,i)]=CubicPolynomial(qStart(1),qEnd(1),qTf,t);
%     [Theta(2,i),Omega(2,i),Beta(2,i)]=CubicPolynomial(qStart(2),qEnd(2),qTf,t);
%     [Theta(3,i),Omega(3,i),Beta(3,i)]=CubicPolynomial(qStart(3),qEnd(3),qTf,t);
%     [Theta(4,i),Omega(4,i),Beta(4,i)]=CubicPolynomial(qStart(4),qEnd(4),qTf,t);
%     PlotUpperLimb([Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i)]);
%     drawnow();
%     T01 = DH(0,0,0,Theta(1,i));
%     T12 = DH(0,-pi/2,0,Theta(2,i)-pi/2);
%     T23 = DH(0,pi/2,0,Theta(3,i));
%     T34 = DH(LenUpperarm,-pi/2,0,Theta(4,i));
%     T45 = DH(LenForeArm,0,0,0);
%     T05 = T01*T12*T23*T34*T45;
%     Pos(1,i) = T05(1,4);
%     Pos(2,i) = T05(2,4);
%     Pos(3,i) = T05(3,4);
% end
% t=0:0.05:qTf;
% figure();
% scatter3(Pos(1,:), Pos(2,:), Pos(3,:),'b','filled');
% title('末端轨迹');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% grid on;
% hold on;
% 
% figure();
% xlabel('s');
% ylabel('rad');
% title('关节角度变化');
% grid on;
% hold on;
% plot(t,Theta(1,:),'c',t,Theta(2,:),'m',t,Theta(3,:),'y',t,Theta(4,:),'r','LineWidth',2);
% legend('theta1','theta2','theta3','theta4');
% 
% figure();
% xlabel('s');
% ylabel('rad/s');
% title('关节角速度变化');
% grid on;
% hold on;
% plot(t,Omega(1,:),'c',t,Omega(2,:),'m',t,Omega(3,:),'y',t,Omega(4,:),'r','LineWidth',2);
% legend('omega1','omega2','omega3','omega4');
% 
% figure();
% xlabel('s');
% ylabel('rad/s^2');
% title('关节角加速度变化');
% grid on;
% hold on;
% plot(t,Beta(1,:),'c',t,Beta(2,:),'m',t,Beta(3,:),'y',t,Beta(4,:),'r','LineWidth',2);
% legend('Beta1','Beta2','Beta3','Beta4');


%% 直线插补
% 选定初始点，终止点，速度，加速度，插补点数（初始点选取的是theta1=theta2=theta3=theta4=0,终止点选取的是theta1=0,theta2=pi/6,theta3=pi/6,theta4=pi/3）
% q0 = [0,0,0,pi/12];
% T010 = DH(0,0,0,q0(1));
% T120 = DH(0,-pi/2,0,q0(2)-pi/2);
% T230 = DH(0,pi/2,0,q0(3));
% T340 = DH(LenUpperarm,-pi/2,0,q0(4));
% T450 = DH(LenForeArm,0,0,0);
% T050 = T010*T120*T230*T340*T450;
% pos_start = T050;
% ps = pos_start(1:3,4); % 初始点位置
% os = pos_start(1:3,1:3); % 初始点姿态
% q1 = [0,0,pi/6,pi/4];
% T011 = DH(0,0,0,q1(1));
% T121 = DH(0,-pi/2,0,q1(2)-pi/2);
% T231 = DH(0,pi/2,0,q1(3));
% T341 = DH(LenUpperarm,-pi/2,0,q1(4));
% T451 = DH(LenForeArm,0,0,0);
% T051 = T011*T121*T231*T341*T451;
% pos_end = T051;
% pe = pos_end(1:3,4); % 末端点位置
% oe = pos_end(1:3,1:3); % 末端点姿态
% vel=100; % 速度
% acc=100; % 加速度
% k=100; % 插补点数
% [R,p,t] = interpolation(pe,ps,oe,os,vel,acc,k);
% 
% q = zeros(4,k+1);
% q(:,1) = [0;0;0;pi/12];
% for i=2:(k+1)
%     dq = [5;5;5;5]*pi/180;
%     Td = [R(:,:,i),p(:,i);zeros(1,3),1];
%     q(:,i) = q(:,i-1);
%     count = 0;
%     while (norm(dq)>0.0001 && count<200)
%         count = count + 1;
%         T01 = DH(0,0,0,q(1,i));
%         T12 = DH(0,-pi/2,0,q(2,i)-pi/2);
%         T23 = DH(0,pi/2,0,q(3,i));
%         T34 = DH(LenUpperarm,-pi/2,0,q(4,i));
%         T45 = DH(LenForeArm,0,0,0);
%         T05 = T01*T12*T23*T34*T45;
%         T(:,:,1) = T01;
%         T(:,:,2) = T12;
%         T(:,:,3) = T23;
%         T(:,:,4) = T34;
%         T(:,:,5) = T45;
%         dt = Td - T05;
%         [m,n]=size(dt);
%         if (m==4 && n==4)
%             v=[dt(3,2),-dt(3,1),dt(2,1)];
%             w=[dt(1,4),dt(2,4),dt(3,4)];
%             kesi=[w,v];
%         else
%             kesi=zeros(6,1);
%         end
%         DT = kesi;
%         J = Jacobi_dif(T);
%         Jp = pinv(J);
%         dq = Jp*DT.';
%         q(:,i) = dq + q(:,i);
%     end
% end
% 
% % 输出关节角度曲线
% figure();
% xlabel('time/s');
% ylabel('angle of joint/rad');
% title('关节角度变化');
% grid on;
% hold on;
% plot(t,real(q(1,:)),'c',t,real(q(2,:)),'m',t,real(q(3,:)),'y',t,real(q(4,:)),'r','LineWidth',2);
% legend('theta1','theta2','theta3','theta4');
% 
% % 正向运动学验证
% x = zeros(1,k+1);
% y = zeros(1,k+1);
% z = zeros(1,k+1);
% for i=2:k+1
%     T01 = DH(0,0,0,q(1,i));
%     T12 = DH(0,-pi/2,0,q(2,i)-pi/2);
%     T23 = DH(0,pi/2,0,q(3,i));
%     T34 = DH(LenUpperarm,-pi/2,0,q(4,i));
%     T45 = DH(LenForeArm,0,0,0);
%     AnsT05 = T01*T12*T23*T34*T45;
%     x(i) = AnsT05(1,4);
%     y(i) = AnsT05(2,4);
%     z(i) = AnsT05(3,4);
% end
% figure();
% scatter3(x(2:101), y(2:101), z(2:101),'k');
% hold on;
% scatter3(p(1,:),p(2,:),p(3,:),'r');
% title('带入关节角度值验证直线');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% view(-30,10);
% grid on;
% hold on;

%% 静力学即绳索张力转换到关节力矩
q = [0,0,0,0];
figure();
PlotUpperLimb(q);

T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

%肩关节力雅可比(3坐标系)
JF_SH = zeros(3,4);
B13 = T03(1:3,1:3)*B10';
B33 = T03(1:3,1:3)*B30';
B43 = T03(1:3,1:3)*B40';
B63 = T03(1:3,1:3)*B60';
L1 = U13' - B13; L2 = U23' - B33; % cable vector
L3 = U33' - B43; L4 = U43' - B63;
l1 = norm(L1); l2 = norm(L2); % cable length
l3 = norm(L3); l4 = norm(L4);
u1 = L1/l1; u2 = L2/l2; % cable unit vector
u3 = L3/l3; u4 = L4/l4;
JF_SH(:,1) = cross(U13',u1);
JF_SH(:,2) = cross(U23',u2);
JF_SH(:,3) = cross(U33',u3);
JF_SH(:,4) = cross(U43',u4);

% 肘关节力雅可比（4坐标系）
JF_EL = zeros(3,2);
T40 = inv(T04);
B24 = T40*[B20,1]';
B54 = T40*[B50,1]';
L5 = F14' - B24(1:3);
L6 = F24' - B54(1:3);
l5 = norm(L5); l6 = norm(L6);
u5 = L5/l5; u6 = L6/l6;
JF_EL(:,1) = cross(F14',u5);
JF_EL(:,2) = cross(F24',u6);
JF_EL(1,:) = zeros(1,2);
JF_EL(2,:) = zeros(1,2);

%% 动力学方程
% 连杆质量
mShoulder = 2;
mElbow = 2;
% 假设已经知道角度，角速度，角加速度
qLimb = [0,0,0,30]*pi/180;
dqLimb = [0,0,0,0];
ddqLimb = [0,0,0,0];
T01 = DH(0,0,0,qLimb(1));
T12 = DH(0,-pi/2,0,qLimb(2)-pi/2);
T23 = DH(0,pi/2,0,qLimb(3));
T34 = DH(LenUpperarm,-pi/2,0,qLimb(4));
T45 = DH(LenForeArm,0,0,0);
% 初始化
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
% 旋转矩阵
R{1} = T01(1:3,1:3);
R{2} = T12(1:3,1:3);
R{3} = T23(1:3,1:3);
R{4} = T34(1:3,1:3);
R{5} = T45(1:3,1:3);
g = -9.81; % 重力方向及大小
dv{1} = [0;0;g]; % 考虑重力
% 坐标系原点位移，用P{1}表示坐标系1与坐标系0原点位置关系，用P{2}表示坐标系2与坐标系1原点位置关系
P{1} = [0;0;0];
P{2} = [0;0;0];
P{3} = [0;0;0];
P{4} = [LenUpperarm;0;0];
P{5} = [LenForeArm;0;0];
% 每个连杆质心的位置矢量(Q:球关节三个自由度质心？)
Pc{2} = [0;0;0];
Pc{3} = [0;0;0];
Pc{4} = [LenUpperarm/2;0;0];
Pc{5} = [LenForeArm/2;0;0];
% 连杆质量
m{2} = 0;
m{3} = 0;
m{4} = mShoulder;
m{5} = mElbow;
% 惯性张量矩阵
% 参考http://bbs.21ic.com/upfiles/img/20079/2007916104241639.pdf求均匀刚体的惯量
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
% 底座不旋转
w{1} = [0;0;0];
dw{1} = [0;0;0];
% 旋转关节
dq{2} = [0;0;dqLimb(1)];
dq{3} = [0;0;dqLimb(2)];
dq{4} = [0;0;dqLimb(3)];
dq{5} = [0;0;dqLimb(4)];
ddq{2} = [0;0;ddqLimb(1)];
ddq{3} = [0;0;ddqLimb(2)];
ddq{4} = [0;0;ddqLimb(3)];
ddq{5} = [0;0;ddqLimb(4)];
% 末端执行器没有力
f{6} = [0;0;0];
n{6} = [0;0;0];
% 外推计算各连杆速度和加速度i:0:1:n
for i=1:4
    w{i+1} = R{i}.'*w{i} + dq{i+1};
    dw{i+1} = R{i}.'*dw{i} + cross(R{i}.'*w{i},dq{i+1}) + ddq{i+1};
    dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
    dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};
    F{i+1}=m{i+1}*dvc{i+1};
    N{i+1} = I{i+1}*dw{i+1} + cross(w{i+1},I{i+1}*w{i+1});
end

% 向内地推力，力矩 i=n:-1:1
for i=5:-1:2
    f{i} = R{i}*f{i+1}+F{i};
    n{i} = N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end






