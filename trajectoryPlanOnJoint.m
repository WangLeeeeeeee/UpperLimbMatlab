% 关节空间轨迹规划
function [torque,JF_SH,JF_EL] = trajectoryPlanOnJoint(qstart, qend, tf, qStep)
% 上肢运动学参数
global LenUpperarm
global LenForeArm
%设定初始角度，终止角度，运行时间
qStart = qstart;
qEnd = qend;
qTf = tf;
step = qStep;
Theta = zeros(4,qTf/step+1);
Omega = zeros(4,qTf/step+1);
Beta = zeros(4,qTf/step+1);
Pos = zeros(3,qTf/step+1);
% 计算角度变化过程中的关节力矩及绳所张力雅可比
torque = zeros(4,qTf/step+1);
JF_SH = zeros(3,4,qTf/step+1);
JF_EL = zeros(3,2,qTf/step+1);
i = 0;
figure();
for t=0:step:qTf
    i = i + 1;
    [Theta(1,i),Omega(1,i),Beta(1,i)]=CubicPolynomial(qStart(1),qEnd(1),qTf,t);% 三次规划函数
    [Theta(2,i),Omega(2,i),Beta(2,i)]=CubicPolynomial(qStart(2),qEnd(2),qTf,t);
    [Theta(3,i),Omega(3,i),Beta(3,i)]=CubicPolynomial(qStart(3),qEnd(3),qTf,t);
    [Theta(4,i),Omega(4,i),Beta(4,i)]=CubicPolynomial(qStart(4),qEnd(4),qTf,t);
    PlotUpperLimb([Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i)]);
    drawnow();
    T01 = DH(0,0,0,Theta(1,i));
    T12 = DH(0,-pi/2,0,Theta(2,i)-pi/2);
    T23 = DH(0,pi/2,0,Theta(3,i));
    T34 = DH(LenUpperarm,-pi/2,0,Theta(4,i));
    T45 = DH(LenForeArm,0,0,0);
    T05 = T01*T12*T23*T34*T45;
    Pos(1,i) = T05(1,4);
    Pos(2,i) = T05(2,4);
    Pos(3,i) = T05(3,4);
    [JF_SH(:,:,i),JF_EL(:,:,i)] = TensiontoTorque(Theta(:,i));
    torque(:,i) = InverseDynamic(Theta(:,i),Omega(:,i),Beta(:,i));
end
t=0:0.05:qTf;
figure();
scatter3(Pos(1,:), Pos(2,:), Pos(3,:),'b','filled');
title('末端轨迹');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
grid on;

figure();
subplot(3,1,1);
xlabel('s');
ylabel('rad');
title('关节角度变化');
grid on;
plot(t,Theta(1,:),'c',t,Theta(2,:),'m',t,Theta(3,:),'y',t,Theta(4,:),'r','LineWidth',2);
legend('theta1','theta2','theta3','theta4');

subplot(3,1,2);
xlabel('s');
ylabel('rad/s');
title('关节角速度变化');
grid on;
plot(t,Omega(1,:),'c',t,Omega(2,:),'m',t,Omega(3,:),'y',t,Omega(4,:),'r','LineWidth',2);
legend('omega1','omega2','omega3','omega4');

subplot(3,1,3);
xlabel('s');
ylabel('rad/s^2');
title('关节角加速度变化');
grid on;
plot(t,Beta(1,:),'c',t,Beta(2,:),'m',t,Beta(3,:),'y',t,Beta(4,:),'r','LineWidth',2);
legend('Beta1','Beta2','Beta3','Beta4');

end