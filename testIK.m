clc;
clear;
%% 上肢运动学参数
LenUpperarm = 0.3035*1000;
LenForeArm = 0.135*1000;

%% 直线插补
% 选定初始点，终止点，速度，加速度，插补点数（初始点选取的是theta1=theta2=theta3=theta4=0,终止点选取的是theta1=0,theta2=pi/6,theta3=pi/6,theta4=pi/3）
q0 = [0,0,0,pi/12];
T010 = DH(0,0,0,q0(1));
T120 = DH(0,-pi/2,0,q0(2)-pi/2);
T230 = DH(0,pi/2,0,q0(3));
T340 = DH(LenUpperarm,-pi/2,0,q0(4));
T450 = DH(LenForeArm,0,0,0);
T050 = T010*T120*T230*T340*T450;
pos_start = T050;
ps = pos_start(1:3,4); % 初始点位置
os = pos_start(1:3,1:3); % 初始点姿态
q1 = [0,0,pi/6,pi/4];
T011 = DH(0,0,0,q1(1));
T121 = DH(0,-pi/2,0,q1(2)-pi/2);
T231 = DH(0,pi/2,0,q1(3));
T341 = DH(LenUpperarm,-pi/2,0,q1(4));
T451 = DH(LenForeArm,0,0,0);
T051 = T011*T121*T231*T341*T451;
pos_end = T051;
pe = pos_end(1:3,4); % 末端点位置
oe = pos_end(1:3,1:3); % 末端点姿态
%oe = [0,1,0;0,0,1;1,0,0];
vel=100; % 速度
acc=100; % 加速度
k=100; % 插补点数
[R,p,t] = interpolation(pe,ps,oe,os,vel,acc,k);

% figure();
% scatter3(p(1,:),p(2,:),p(3,:),'k');
% title('插补直线');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% grid on;
% hold on;

q = zeros(4,k+1);
q(:,1) = [0;0;0;pi/12];
for i=2:(k+1)
    dq = [5;5;5;5]*pi/180;
    Td = [R(:,:,i),p(:,i);zeros(1,3),1];
    q(:,i) = q(:,i-1);
    count = 0;
    while (norm(dq)>0.0001 && count<200)
        count = count + 1;
        T01 = DH(0,0,0,q(1,i));
        T12 = DH(0,-pi/2,0,q(2,i)-pi/2);
        T23 = DH(0,pi/2,0,q(3,i));
        T34 = DH(LenUpperarm,-pi/2,0,q(4,i));
        T45 = DH(LenForeArm,0,0,0);
        T05 = T01*T12*T23*T34*T45;
        T(:,:,1) = T01;
        T(:,:,2) = T12;
        T(:,:,3) = T23;
        T(:,:,4) = T34;
        T(:,:,5) = T45;
        dt = Td - T05;
        [m,n]=size(dt);
        if (m==4 && n==4)
            v=[dt(3,2),-dt(3,1),dt(2,1)];
            w=[dt(1,4),dt(2,4),dt(3,4)];
            kesi=[w,v];
        else
            kesi=zeros(6,1);
        end
        DT = kesi;
        J = Jacobi_dif(T);
        Jp = pinv(J);
        dq = Jp*DT.';
        q(:,i) = dq + q(:,i);
    end
end

% 输出关节角度曲线
figure();
xlabel('time/s');
ylabel('angle of joint/rad');
title('关节角度变化');
grid on;
hold on;
plot(t,real(q(1,:)),'c',t,real(q(2,:)),'m',t,real(q(3,:)),'y',t,real(q(4,:)),'r','LineWidth',2);
legend('theta1','theta2','theta3','theta4');

% 正向运动学验证
x = zeros(1,k+1);
y = zeros(1,k+1);
z = zeros(1,k+1);
for i=2:k+1
    T01 = DH(0,0,0,q(1,i));
    T12 = DH(0,-pi/2,0,q(2,i)-pi/2);
    T23 = DH(0,pi/2,0,q(3,i));
    T34 = DH(LenUpperarm,-pi/2,0,q(4,i));
    T45 = DH(LenForeArm,0,0,0);
    AnsT05 = T01*T12*T23*T34*T45;
    x(i) = AnsT05(1,4);
    y(i) = AnsT05(2,4);
    z(i) = AnsT05(3,4);
end
figure();
scatter3(x(2:101), y(2:101), z(2:101),'k');
hold on;
scatter3(p(1,:),p(2,:),p(3,:),'r');
title('带入关节角度值验证直线');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
view(-30,10)
grid on;
hold on;