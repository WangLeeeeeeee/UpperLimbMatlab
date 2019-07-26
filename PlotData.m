%-------将采集到的数据绘制曲线--------------------
clc;
clear;
% num = xlsread("test20181205.xlsx");
% tension = num(1:7531,1:6);
% angleElbow = num(:,7:9);
% angleShoulder = num(:,10:12);
% a = 0:7530;
% plot(0.01*a,tension*0.01,'linewidth',1.5);
% legend('cable1','cable2','cable3','cable4','cable5','cable6');
% ylabel('tension value(N)');
% xlabel('time(s)');
% 
% %% shoulder angle
% % flexion/extension
% subplot(2,2,1);
% shoulderYStart = 0;
% shoulderYEnd = -27;
% qTf = 190;
% shoulderYangle = zeros(1800,1);
% for t=1:qTf
%     shoulderYangle(t) = CubicPolynomial(shoulderYStart,shoulderYEnd,qTf,t);% 三次规划函数
% end
% for t=qTf+1:2*qTf
%     shoulderYangle(t) = CubicPolynomial(shoulderYEnd,shoulderYStart,qTf,t-qTf);% 三次规划函数
% end
% shoulderYangle(2*qTf+1:4*qTf) = shoulderYangle(1:2*qTf);
% shoulderYangle(4*qTf+1:6*qTf) = shoulderYangle(1:2*qTf);
% shoulderYangle(6*qTf+1:8*qTf) = shoulderYangle(1:2*qTf);
% tlag = 80;
% shoulderYangle(tlag+1:1800) = shoulderYangle(1:1800-tlag);
% shoulderYangle(1:tlag) = 0;
% k = 0:0.01:17.5;
% plot(k,shoulderYangle(1:1751,1),k,angleShoulder(2250:4000,3),'--','linewidth',2.5);
% title('shoulder flexion/extension');
% ylim([-40 10]);
% xlabel('time(s)');
% ylabel('q1(\circ)');
% 
% % adduction/abduction
% subplot(2,2,2);
% shoulderXStart = 0;
% shoulderXEnd = -43;
% qTf = 190;
% shoulderXangle = zeros(1800,1);
% for t=1:qTf
%     shoulderXangle(t) = CubicPolynomial(shoulderXStart,shoulderXEnd,qTf,t);% 三次规划函数
% end
% for t=qTf+1:2*qTf
%     shoulderXangle(t) = CubicPolynomial(shoulderXEnd,shoulderXStart,qTf,t-qTf);% 三次规划函数
% end
% shoulderXangle(2*qTf+1:4*qTf) = shoulderXangle(1:2*qTf);
% shoulderXangle(4*qTf+1:6*qTf) = shoulderXangle(1:2*qTf);
% shoulderXangle(6*qTf+1:8*qTf) = shoulderXangle(1:2*qTf);
% tlag = 55;
% shoulderXangle(tlag+1:1800) = shoulderXangle(1:1800-tlag);
% shoulderXangle(1:tlag) = 0;
% plot(k,shoulderXangle(1:1751,1),k,angleShoulder(4250:6000,2),'--','linewidth',2.5);
% title('shoulder adduction/abduction');
% ylim([-50 10]);
% xlabel('time(s)');
% ylabel('q2(\circ)');
% 
% % lateral/medial motion
% subplot(2,2,3);
% shoulderZStart = 23;
% shoulderZEnd = -23;
% qTf = 195;
% shoulderZangle = zeros(1800,1);
% for t=1:qTf
%     shoulderZangle(t) = CubicPolynomial(shoulderZStart,shoulderZEnd,qTf,t);% 三次规划函数
% end
% for t=qTf+1:2*qTf
%     shoulderZangle(t) = CubicPolynomial(shoulderZEnd,shoulderZStart,qTf,t-qTf);% 三次规划函数
% end
% shoulderZangle(2*qTf+1:4*qTf) = shoulderZangle(1:2*qTf);
% shoulderZangle(4*qTf+1:6*qTf) = shoulderZangle(1:2*qTf);
% shoulderZangle(6*qTf+1:8*qTf) = shoulderZangle(1:2*qTf);
% tlag = 400;
% shoulderZangle(tlag+1:1800) = shoulderZangle(1:1800-tlag);
% shoulderZangle(1:tlag) = 23;
% plot(k,shoulderZangle(1:1751,1),k,angleShoulder(9250:11000,1),'--','linewidth',2.5);
% title('shoulder lateral/medial motion');
% ylim([-30 30]);
% xlabel('time(s)');
% ylabel('q3(\circ)');
% 
% 
% %% elbow angle
% subplot(2,2,4);
% elbowStart = 0;
% elbowEnd = -38;
% qTf = 185;
% elbowAngle = zeros(1800,1);
% for t=1:qTf
%     elbowAngle(t) = CubicPolynomial(elbowStart,elbowEnd,qTf,t);% 三次规划函数
% end
% for t=qTf+1:2*qTf
%     elbowAngle(t) = CubicPolynomial(elbowEnd,elbowStart,qTf,t-qTf);% 三次规划函数
% end
% elbowAngle(2*qTf+1:4*qTf) = elbowAngle(1:2*qTf);
% elbowAngle(4*qTf+1:6*qTf) = elbowAngle(1:2*qTf);
% elbowAngle(6*qTf+1:8*qTf) = elbowAngle(1:2*qTf);
% tlag = 80;
% elbowAngle(tlag+1:1800) = elbowAngle(1:1800-tlag);
% elbowAngle(1:tlag) = 0;
% plot(k,elbowAngle(1:1751,1),k,angleElbow(4250:6000,2),'--','linewidth',2.5);
% ylim([-40 10]);
% title('elbow flexion/extension')
% xlabel('time(s)');
% ylabel('q4(\circ)');
% legend('Planned','Measured');

%% 采集到的传感器数据
number = 204;
t = 1:number;
%t = 1:310;
data = xlsread("C:\Users\TEMP\Desktop\test.xlsx");
tension = data(t,1:6);
angleElbow = data(t,7:9);
angleShoulder = data(t,10:12);
encoder = data(1:204,13:18);  
encoder_diff = diff(encoder);
figure();
plot(encoder_diff);
figure();
plot(tension);
figure();
plot(angleElbow);
figure();
plot(angleShoulder);
figure();
subplot(2,1,1);
plot(t,tension(:,5),t,tension(:,6));
legend("tension5","tension6");
subplot(2,1,2);
plot(t,angleElbow(:,3));
legend("elbow");
figure();
plotyy(t,tension(:,5),t,angleElbow(:,3));

figure();
subplot(3,1,1);
plot(t,tension(:,5),t,tension(:,6));
legend("tension5","tension6");
xlim([1900 2800]);
subplot(3,1,2);
plot(t,angleElbow(:,3));
legend("elbow");
xlim([1900 2800]);
subplot(3,1,3);
plot(K_ELBOW);
legend('KEL');
xlim([1900 2800]);


