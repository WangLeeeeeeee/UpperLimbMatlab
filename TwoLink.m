clc;
clear;
%% 二连杆长度
l1 = 3;
l2 = 3;
syms theta1;
syms theta2;
syms pi;
step = 100;

%% 正向运动学
x = l1*cos(theta1) + l2*cos(theta1+theta2);
y = l1*sin(theta1) + l2*sin(theta1+theta2);

%% 逆向运动学，位置解
x0 = input('请输入x起始坐标：');
y0 = input('请输入y起始坐标：');
c20 = (x0^2+y0^2-l1^2-l2^2)/(2*l1*l2);
s20 = sqrt(1-c20^2);
if (c20>1) || (c20<-1)
    warndlg('Position out of maxium');
else
    theta20 = atan2(s20,c20);
    theta10 = atan2(y0,x0)-atan2(l2*s20,l1+l2*c20);
end
x1 = input('请输入x终点坐标：');
y1 = input('请输入y终点坐标：');
c21 = (x1^2+y1^2-l1^2-l2^2)/(2*l1*l2);
s21 = sqrt(1-c21^2);
if (c21>1) || (c21<-1)
    warndlg('Position out of maxium');
else
    theta21 = atan2(s21,c21);
    theta11 = atan2(y1,x1)-atan2(l2*s21,l1+l2*c21);
end

%% 点到点
detaTheta1 = (theta11 - theta10)/step;
detaTheta2 = (theta21 - theta20)/step;
x_pos = zeros(1,step);
y_pos = zeros(1,step);
theta1_ptp = zeros(1,100);
theta2_ptp = zeros(1,100);
t = zeros(1,100);
for i=1:1:step
    t(i) = i;
    theta1_ptp(i) = theta10 + i*detaTheta1;
    theta2_ptp(i) = theta20 + i*detaTheta2;
    x_pos(i) = vpa(subs(x,[theta1,theta2],[theta1_ptp(i),theta2_ptp(i)]));
    y_pos(i) = vpa(subs(y,[theta1,theta2],[theta1_ptp(i),theta2_ptp(i)]));
end
figure(1);
scatter(x_pos,y_pos,'k');
axis([-6,6,-6,6]);
xlabel('x坐标(m)');
ylabel('y坐标(m)');
title('点到点');

%% 直线插补
% 设定速度，加速度，插补点数
vel = 100; % 速度
acc = 200; % 加速度
k = 100; % 插补点数
tA = vel/acc;
dist = sqrt((x1-x0)^2+(y1-y0)^2);
if (acc*tA*tA >= dist) 
    tA = sqrt(dist/acc);
    tS = 0; % 匀速时间
else
    tS = (dist-acc*tA*tA)/vel;
end
t_all = 2*tA + tS; % 总时间
t_interval = t_all/k; % 每段插补的时间
t=0:t_interval:t_all;
ps = [x0;y0];
pe = [x1;y1];
p = zeros(2,k+1);
p(:,1) = ps;
S = 0;
step = k;
if(tS == 0)
% no constant velocity
    for k=2:k+1
        if(t(k)<tA)
            dertaS = 0.5*acc*t_interval*(t(k)+t(k-1));
        elseif(t(k)>tA && t(k-1)<tA)
            dertaS = 0.5*acc*(tA-t(k-1))*(tA+t(k-1))+0.5*acc*(t(k)-tA)*(tA+tA-(t(k)-tA));
        else
            dertaS = 0.5*acc*((t_all-t(k))+(t_all-t(k-1)))*t_interval;
        end
        p(:,k) = p(:,k-1)+(pe-ps)*dertaS/dist;
        S = S + dertaS;
    end
else
% with constant velocity
    for k=2:k+1
        if(t(k)<tA)
            dertaS = 0.5*acc*t_interval*(t(k)+t(k-1));
        elseif(t(k)<=tA+tS && t(k-1)<=tA)
            dertaS=vel*(t(k)-tA)+0.5*vel*tA-0.5*acc*t(k-1)*t(k-1);
        elseif(t(k)<=tA+S && t(k-1)<=tA+tS)
            detraS = vel*t_interval;
        elseif(t(k)<=t_all && t(k-1)<=tA+tS)
            dertaS = -0.5*acc*(t(k)-tA-tS)*(t(k)-tA-tS)+vel*(t(k)-t(k-1));
        else
            dertaS=0.5*acc*((t_all-t(k))+(t_all-t(k-1)))*t_interval;
        end
        p(:,k) = p(:,k-1) + (pe-ps)*dertaS/dist;
        S = S + dertaS;
    end                     
end
q = zeros(2,step+1);
q(:,1) = [0;0]*pi/180;
x_test = zeros(1,step+1);
y_test = zeros(1,step+1);
for i=2:step+1
    c20 = ((p(1,i))^2+(p(2,i))^2-l1^2-l2^2)/(2*l1*l2);
    s20 = sqrt(1-c20^2);
    if (c20>1) || (c20<-1)
        warndlg('Position out of maxium');
        break;
    else
        q(2,i) = atan2(s20,c20);
        q(1,i) = atan2(p(2,i),p(1,i))-atan2(l2*s20,l1+l2*c20);
        x_test(i) = vpa(subs(x,[theta1,theta2],[q(1,i),q(2,i)]));
        y_test(i) = vpa(subs(y,[theta1,theta2],[q(1,i),q(2,i)]));
    end
end
figure();
xlabel('time/s');
ylabel('angle of joint/rad');
title('直线插补关节角度变化');
grid on;
hold on;
plot(t,q(1,:),'c',t,q(2,:),'m','LineWidth',2);
legend('theta1','theta2');
figure();
scatter(x_test,y_test,'k');
xlabel('x坐标(m)');
ylabel('y坐标(m)');


% detaX = (x1-x0)/step;
% detaY = (y1-y0)/step;
% x_line = zeros(1,step);
% y_line = zeros(1,step);
% theta1_line = zeros(1,step);
% theta2_line = zeros(1,step);
% for i=1:1:step
%     x_line(i) = x0 + i*detaX;
%     y_line(i) = y0 + i*detaY;
%     c20 = ((x_line(i))^2+y_line(i)^2-l1^2-l2^2)/(2*l1*l2);
%     s20 = sqrt(1-c20^2);
%     if (c20>1) || (c20<-1)
%         warndlg('Position out of maxium');
%         break;
%     else
%         theta2_line(i) = atan2(s20,c20);
%         theta1_line(i) = atan2(y_line(i),x_line(i))-atan2(l2*s20,l1+l2*c20);
%     end
% end
% figure(2);
% scatter(x_line,y_line,'k');
% axis([-6,6,-6,6]);
% xlabel('x坐标(m)');
% ylabel('y坐标(m)');
% title('直线插补');
% figure(3);
% plot(t,theta1_line);
% hold on;
% plot(t,theta2_line);
% legend('thrta1','theta2');
% title('直线插补');

