clc;
clear;
%% 上肢运动学参数
LenUpperarm = 0.3035*1000;
LenForeArm = 0.135*1000;

%% DH法建立运动学模型 subs(T01,theta,pi/4)
syms theta1;
syms theta2;
syms theta3;
syms theta4;
syms pi % 定义后cos(pi/2)=0, 否则不为零
T01 = DH(0,0,0,theta1);
T12 = DH(0,-pi/2,0,theta2-pi/2);
T23 = DH(0,pi/2,0,theta3);
T34 = DH(LenUpperarm,-pi/2,0,theta4);
T45 = DH(LenForeArm,0,0,0);

%% 绳索节点坐标
% 基座上六个绳索节点（相对于0坐标系）
B10 = [-0.142,-0.030,0.033]*1000;
B20 = [-0.142,0,0.033]*1000;
B30 = [-0.006,0.142,0.033]*1000;
B40 = [0.006,0.142,0.033]*1000;
B50 = [0.142,0,0.033]*1000;
B60 = [0.142,-0.030,0.033]*1000;
% 上臂四个绳索节点（相对于3坐标系）
U13 = [0.1715,-0.030,0.125]*1000;
U23 = [0.1715,0,0.125]*1000;
U33 = [0.1715,0,-0.125]*1000;
U43 = [0.1715,-0.030,-0.125]*1000;
U53 = [0.1715,0,0.125]*1000;
% 前臂两个绳索节点（相对于4坐标系）
F14 = [0.118,-0.118,0]*1000;
F24 = [0.118,0.118,0]*1000;

%% 正向运动学
T03 = T01*T12*T23;
T04 = T03*T34;
T05 = T01*T12*T23*T34*T45;
U10 = T03*[U13,1]';
U20 = T03*[U23,1]';
U30 = T03*[U33,1]';
U40 = T03*[U43,1]';
U50 = T03*[U53,1]';
F10 = T04*[F14,1]';
F20 = T04*[F24,1]';
len1 = sqrt((B10(1)-U10(1))^2 + (B10(2)-U10(2))^2 + (B10(3)-U10(3))^2);
len2 = sqrt((B30(1)-U20(1))^2 + (B30(2)-U20(2))^2 + (B30(3)-U20(3))^2);
len3 = sqrt((B40(1)-U30(1))^2 + (B40(2)-U30(2))^2 + (B40(3)-U30(3))^2);
len4 = sqrt((B50(1)-U40(1))^2 + (B50(2)-U40(2))^2 + (B50(3)-U40(3))^2);
len5 = sqrt((B20(1)-U50(1))^2 + (B20(2)-U50(2))^2 + (B20(3)-U50(3))^2) + sqrt((U50(1)-F10(1))^2 + (U50(2)-F10(2))^2 + (U50(3)-F10(3))^2);
len6 = sqrt((B60(1)-F20(1))^2 + (B60(2)-F20(2))^2 + (B60(3)-F20(3))^2);
%上臂连杆上点
syms posLenUpp;
syms posLenFor;
U03 = [posLenUpp, 0, 0];
F04 = [posLenFor, 0, 0];
U00 = T03*[U03,1].';%“'”求的是共轭复数，非共轭用的是“.'”
F00 = T04*[F04,1].';

% 初始关节角和终止关节角
Theta0 = [0,0,0,0];
Theta1 = [0,0,40*pi/180,0];


% 遍历工作空间
the1_start = -50*pi/180;
the1_end = 50*pi/180;
the2_start = -45*pi/180;
the2_end = 45*pi/180;
the3_start = 0*pi/180;
the3_end = 45*pi/180;
the4_start = 0*pi/180;
the4_end = 70*pi/180;
space = zeros(3,10000000);
i = 0;
for the1_sp = the1_start:0.05:the1_end
    for the2_sp = the2_start:0.05:the2_end
        for the3_sp = the3_start:0.5:the3_end
            for the4_sp = the4_start:0.5:the4_end
                i = i + 1;
                t05 = subs(T05,[theta1,theta2,theta3,theta4],[the1_sp,the2_sp,the3_sp,the4_sp]);
                space(1,i) = t05(1,4);
                space(2,i) = t05(2,4);
                space(3,i) = t05(3,4);
            end
        end
    end
end
figure();
scatter3(space(1,:), space(2,:), space(3,:),'r');
title('末端工作空间');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
grid on;
hold on;

%遍历肘关节
% i = 0;
% for the1_sp = the1_start:0.05:the1_end
%     for the2_sp = the2_start:0.05:the2_end
%         for the3_sp = the3_start:0.5:the3_end
%             i = i + 1;
%             t04 = subs(T04,[theta1,theta2,theta3],[the1_sp,the2_sp,the3_sp]);
%             space(1,i) = t04(1,4);
%             space(2,i) = t04(2,4);
%             space(3,i) = t04(3,4);
%         end
%     end
% end
% figure();
% scatter3(space(1,:), space(2,:), space(3,:),'r');
% title('肘部中心运动空间');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% grid on;
% hold on;

% % 正向运动学验证
% x = zeros(1,k+1);
% y = zeros(1,k+1);
% z = zeros(1,k+1);
% AnsT04 = zeros(4,4,k+1);
% for i=2:k+1
%     AnsT04(:,:,i) = subs(T01*T12*T23*T34,[theta1,theta2,theta3,theta4],[q(1,i),q(2,i),q(3,i),q(4,i)]);
%     x(i) = AnsT04(1,4,i);
%     y(i) = AnsT04(2,4,i);
%     z(i) = AnsT04(3,4,i);
% end
% figure();
% scatter3(x, y, z,'k');
% title('带入关节角度值验证直线');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% grid on;
% hold on;

len10 = vpa(subs(len1, [theta1,theta2,theta3,theta4], Theta0));
len20 = vpa(subs(len2, [theta1,theta2,theta3,theta4], Theta0));
len30 = vpa(subs(len3, [theta1,theta2,theta3,theta4], Theta0));
len40 = vpa(subs(len4, [theta1,theta2,theta3,theta4], Theta0));
len50 = vpa(subs(len5, [theta1,theta2,theta3,theta4], Theta0));
len60 = vpa(subs(len6, [theta1,theta2,theta3,theta4], Theta0));
len11 = vpa(subs(len1, [theta1,theta2,theta3,theta4], Theta1));
len21 = vpa(subs(len2, [theta1,theta2,theta3,theta4], Theta1));
len31 = vpa(subs(len3, [theta1,theta2,theta3,theta4], Theta1));
len41 = vpa(subs(len4, [theta1,theta2,theta3,theta4], Theta1));
len51 = vpa(subs(len5, [theta1,theta2,theta3,theta4], Theta1));
len61 = vpa(subs(len6, [theta1,theta2,theta3,theta4], Theta1));
detaLen1 = len11 - len10;
detaLen2 = len21 - len20;
detaLen3 = len31 - len30;
detaLen4 = len41 - len40;
detaLen5 = len51 - len50;
detaLen6 = len61 - len60;

%% 对于每个自由度单独考虑
% 对肘关节肘曲theta1=theta2=theta3=0
% 由于肘关节由两根绳索单独驱动，不影响肩关节，故只需要考虑len5和len6以及相应的绳索节点坐标B20 B60 F10 F20
len5_el = vpa(subs(len5, [theta1,theta2,theta3], [0,0,0]));
len6_el = vpa(subs(len6, [theta1,theta2,theta3], [0,0,0]));
%由于肩关节由四根绳索驱动，不影响肘关节，故只需要考虑len1,len2,len3,len4
% 对肩关节内外旋theta2=theta3=theta4=0
len1_sh1 = vpa(subs(len1, [theta2,theta3,theta4], [0,0,0]));
len2_sh1 = vpa(subs(len2, [theta2,theta3,theta4], [0,0,0]));
len3_sh1 = vpa(subs(len3, [theta2,theta3,theta4], [0,0,0]));
len4_sh1 = vpa(subs(len4, [theta2,theta3,theta4], [0,0,0]));
len5_sh1 = vpa(subs(len5, [theta2,theta3,theta4], [0,0,0]));
len6_sh1 = vpa(subs(len6, [theta2,theta3,theta4], [0,0,0]));
% 对于肩关节前屈后伸
len1_sh2 = vpa(subs(len1, [theta1,theta3,theta4], [0,0,0]));
len2_sh2 = vpa(subs(len2, [theta1,theta3,theta4], [0,0,0]));
len3_sh2 = vpa(subs(len3, [theta1,theta3,theta4], [0,0,0]));
len4_sh2 = vpa(subs(len4, [theta1,theta3,theta4], [0,0,0]));
len5_sh2 = vpa(subs(len5, [theta1,theta3,theta4], [0,0,0]));
len6_sh2 = vpa(subs(len6, [theta1,theta3,theta4], [0,0,0]));
% 对于肩关节内收外展
len1_sh3 = vpa(subs(len1, [theta1,theta2,theta4], [0,0,0]));
len2_sh3 = vpa(subs(len2, [theta1,theta2,theta4], [0,0,0]));
len3_sh3 = vpa(subs(len3, [theta1,theta2,theta4], [0,0,0]));
len4_sh3 = vpa(subs(len4, [theta1,theta2,theta4], [0,0,0]));
len5_sh3 = vpa(subs(len5, [theta1,theta2,theta4], [0,0,0]));
len6_sh3 = vpa(subs(len6, [theta1,theta2,theta4], [0,0,0]));

%% 运动学逆解
% 先用正向运动学给出末端位置
L04_pos = T01*T12*T23*T34;
L = subs(L04_pos,[theta1,theta2,theta3,theta4],[0,0,pi/6,0]);
theta_inv = Inverse_kinetic(L(1:3,4),L(1:3,1:3));
T04_ans = vpa(subs(T01*T12*T23*T34,[theta1,theta2,theta3,theta4],[theta_inv(1),theta_inv(2),theta_inv(3),theta_inv(4)]));

%% 直线插补
% 选定初始点，终止点，速度，加速度，插补点数（初始点选取的是theta1=theta2=theta3=theta4=0,终止点选取的是theta1=0,theta2=pi/6,theta3=pi/6,theta4=pi/3）
pos_start = vpa(subs(T05,[theta1,theta2,theta3,theta4],[pi/100,pi/100,pi/100,pi/100]));
ps = pos_start(1:3,4); % 初始点位置
os = pos_start(1:3,1:3); % 初始点姿态
pos_end = vpa(subs(T05,[theta1,theta2,theta3,theta4],[0,pi/6,pi/6,pi/6]));
pe = pos_end(1:3,4); % 末端点位置
oe = pos_end(1:3,1:3); % 末端点姿态
%oe = [0,1,0;0,0,1;1,0,0];
vel=100; % 速度
acc=100; % 加速度
k=100; % 插补点数
[R,p,t] = interpolation(pe,ps,oe,os,vel,acc,k);

figure();
scatter3(p(1,:),p(2,:),p(3,:),'k');
title('插补直线');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
grid on;
hold on;

q = zeros(4,k+1);
q(:,1) = [0;0;0;0]*pi/180;

% % 逆向运动学解算
% for i=2:k+1
%     q(:,i) = Inverse_kinetic(p(:,i),R(:,:,i));
%     p(:,i)
%     R(:,:,i)
%     vpa(subs(T01*T12*T23*T34,[theta1,theta2,theta3,theta4],[q(1,i),q(2,i),q(3,i),q(4,i)]))
% end
% % 输出关节角度曲线
% figure();
% xlabel('time/s');
% ylabel('angle of joint/rad');
% title('关节角度变化');
% grid on;
% hold on;
% plot(t,q(1,:),'c',t,q(2,:),'m',t,q(3,:),'y',t,q(4,:),'r','LineWidth',2);
% legend('theta1','theta2','theta3','theta4');

%% 逆运动学数值解
% maybe the 
dq = 0;
T(:,:,1) = T01;
T(:,:,2) = T12;
T(:,:,3) = T23;
T(:,:,4) = T34;
T(:,:,5) = T45;
T = subs(T,[theta1;theta2;theta3;theta4],q(:,1));
for i=2:(k+1)
    q(:,i) = q(:,i-1);
    count = 0;
    while true
        %calculate the error
        dq = [5;5;5;5]*pi/180;
        Td = [R(:,:,i),p(:,i);zeros(1,3),1];
        count = count + 1;
        T = subs(T,[theta1;theta2;theta3;theta4],q(:,i));
        dt = Td - subs(T05,[theta1;theta2;theta3;theta4],q(:,i));
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
        nm = norm(DT);
        if nm<=0.01 || count>200
            vpa(dt)
            break
        end
    end
end
% for i=2:(k+1)
%     dq = [5;5;5;5]*pi/180;
%     Td = [R(:,:,i),p(:,i);zeros(1,3),1];
%     q(:,i) = q(:,i-1);
%     count = 0;
%     while (norm(dq)>0.01 && count<200)
%         count = count + 1;
%         T = subs(T,[theta1;theta2;theta3;theta4],q(:,i));
%         dt = Td - subs(T05,[theta1;theta2;theta3;theta4],q(:,i));
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
%     vpa(dt)
% end

% 输出关节角度曲线
figure();
xlabel('time/s');
ylabel('angle of joint/rad');
title('关节角度变化');
grid on;
hold on;
plot(t,real(q(1,:)),'c',t,real(q(2,:)),'m',t,real(q(3,:)),'y',t,real(q(4,:)),'r','LineWidth',2);
legend('theta1','theta2','theta3','theta4');

% 解算绳索节点位置及绳长变化
len_pos = zeros(6,k+1);
for i=2:k+1
    len_pos(1,i) = subs(len1,[theta1;theta2;theta3;theta4],real(q(:,i)));
    len_pos(2,i) = subs(len2,[theta1;theta2;theta3;theta4],real(q(:,i)));
    len_pos(3,i) = subs(len3,[theta1;theta2;theta3;theta4],real(q(:,i)));
    len_pos(4,i) = subs(len4,[theta1;theta2;theta3;theta4],real(q(:,i)));
    len_pos(5,i) = subs(len5,[theta1;theta2;theta3;theta4],real(q(:,i)));
    len_pos(6,i) = subs(len6,[theta1;theta2;theta3;theta4],real(q(:,i)));
end
% 输出绳长变化曲线
figure();
xlabel('time/s');
ylabel('length/mm');
title('绳长变化');
grid on;
hold on;
plot(t,len_pos(1,:),'c',t,len_pos(2,:),'m',t,len_pos(3,:),'y',t,len_pos(4,:),'r',t,len_pos(5,:),'k',t,len_pos(6,:),'g','LineWidth',2);
legend('len1','len2','len3','len4','len5','len6');

% 正向运动学验证
x = zeros(1,k+1);
y = zeros(1,k+1);
z = zeros(1,k+1);
AnsT05 = zeros(4,4,k+1);
for i=2:k+1
    AnsT05(:,:,i) = subs(T05,[theta1,theta2,theta3,theta4],[real(q(1,i)),real(q(2,i)),real(q(3,i)),real(q(4,i))]);
    x(i) = AnsT05(1,4,i);
    y(i) = AnsT05(2,4,i);
    z(i) = AnsT05(3,4,i);
end
figure();
scatter3(x(2:101), y, z,'k');
title('带入关节角度值验证直线');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
grid on;
hold on;

ANSq = zeros(4,k+1);
for i=2:k+1
    ANSq(:,i) = vpa(real(q(:,i)),3);
end








 



















