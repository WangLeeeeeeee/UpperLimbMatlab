% 绘制simulink的仿真结果
error_data = error.data;
q_data = q.data;
dq_data = dq.data;
stiffness_data = Stiffness.data;
tension_data = Tension.data;
torque_data = torque.data;
input_data = Input.data;
time = error.time;
len = length(time);

%% 误差曲线
q_error = error_data(:,1:4);
figure();
plot(time,q_error(:,1),'r',time,q_error(:,2),'b',time,q_error(:,3),'c',time,q_error(:,4),'g','LineWidth',1.5);
title('Joint angle error');
xlabel('time ( s )');
ylabel('error(rad)');
legend('error_1','error_2','error_3','error_4');

dq_error = error_data(:,5:8);
figure();
plot(time,dq_error(:,1),'r',time,dq_error(:,2),'b',time,dq_error(:,3),'c',time,dq_error(:,4),'g','LineWidth',1.5);
title('Joint speed error');
xlabel('time ( s )');
ylabel('error(rad/s)');
legend('error_1','error_2','error_3','error_4');

%% 关节角度曲线
% input_data(:,1:9) = zeros(len,9);
% input_data(:,10) = (pi/3)*sin(0.5*pi*time);
% input_data(:,11) = (pi/3)*(0.5*pi)*cos(0.5*pi*time);
% input_data(:,12) = -(pi/3)*(0.5*pi)^2*sin(0.5*pi*time);
q_input = zeros(len,4);
q_input(:,1) = input_data(:,1);
q_input(:,2) = input_data(:,4);
q_input(:,3) = input_data(:,7);
q_input(:,4) = input_data(:,10);
figure();
plot(time,q_input(:,1),'r',time,q_data(:,1),'b','LineWidth',1.5);
title('Joint 1');
xlabel('time ( s )');
ylabel('\theta(rad/s)');
legend('expected','real');
figure();
plot(time,q_input(:,2),'r',time,q_data(:,2),'b','LineWidth',1.5);
title('Joint 2');
xlabel('time ( s )');
ylabel('\theta(rad/s)');
legend('expected','real');
figure();
plot(time,q_input(:,3),'r',time,q_data(:,3),'b','LineWidth',1.5);
title('Joint 3');
xlabel('time ( s )');
ylabel('\theta(rad/s)');
legend('expected','real');
figure();
plot(time,q_input(:,4),'r',time,q_data(:,4),'b','LineWidth',1.5);
title('Joint 4');
xlabel('time ( s )');
ylabel('\theta(rad/s)');
legend('expected','real');

%% 关节力矩曲线
figure();
plot(time,torque_data(:,1),'k',time,torque_data(:,2),'g',time,torque_data(:,3),'b',time,torque_data(:,4),'r','LineWidth',1.5);
legend('\tau_1','\tau_2','\tau_3','\tau_4');
xlabel('s');
ylabel('N\cdotm');
title('关节力矩');

%% 绳索张力曲线
figure();
subplot(2,1,1);
plot(time,tension_data(:,1),'r',time,tension_data(:,2),'b',time,tension_data(:,3),'c',time,tension_data(:,4),'g','LineWidth',1.5);
title('shoulder module');
xlabel('time ( s )');
ylabel('cable tension ( N )');
legend('F1','F2','F3','F4');
grid on;
subplot(2,1,2);
plot(time,tension_data(:,5),'m',time,tension_data(:,6),'k','LineWidth',1.5);
title('elbow module');
xlabel('time ( s )');
ylabel('cable tension ( N )');
legend('F5','F6');
grid on;

%% 关节刚度曲线（simulink中的计算结果）
figure();
plot(time,stiffness_data(:,1),'r','LineWidth',1.5);
title('Stiffness of Shoulder joint');
xlabel('time ( s )');
figure();
plot(time,stiffness_data(:,2),'b','LineWidth',1.5);
title('Stiffness of Elbow joint');
xlabel('time ( s )');



