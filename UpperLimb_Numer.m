%����ֵ������֫��ģ����ý��Ϊ������������ֵ������Ͽ�
clc;
clear;
% theta1: �������� theta1>0 ����
% theta2: ��ǰ��
% theta3: ����չ
% theta4: ��ǰ��
global LenUpperarm
global LenForeArm
global mShoulder
global mElbow
global B10
global B20
global B30
global B40
global B50
global B60
global U13
global U23
global U33
global U43
global U53
global U63
global F14
global F24
global g

%% ��֫����
% ���˳���(mm)
LenUpperarm = 0.3035;
LenForeArm = 0.135*2;
% ��������(kg)
mShoulder = 0.5;
mElbow = 0.3;
% ������С
g = 9.81; 

%% �����ڵ�����
% ���������������ڵ㣨�����0����ϵ��
B10 = [-0.142,-0.110,0.033];
B20 = [-0.142,0,0.033];
B30 = [-0.037,0.137,0.033];%z=0.02
B40 = [0.037,0.137,0.033];%z=0.02
B50 = [0.142,0,0.033];
B60 = [0.142,-0.110,0.033];
% �ϱ��ĸ������ڵ㣨�����3����ϵ��
U13 = [0.1715,-0.030,0.125];
U23 = [0.1715,0,0.125];
U33 = [0.1715,0,-0.125];
U43 = [0.1715,-0.030,-0.125];
U53 = [0.1715,0,0.125];
% ǰ�����������ڵ㣨�����4����ϵ��
F14 = [0.118,-0.118,0];
F24 = [0.118,0.118,0];

% %% �����ڵ�����
% % ���������������ڵ㣨�����0����ϵ��
% B10 = [-0.230,-0.1125,0.017];
% B20 = [-0.230,0,0.017];
% B30 = [-0.158,0.152,0.017];%z=0.02
% B40 = [0.158,0.152,0.017];%z=0.02
% B50 = [0.230,0,0.017]*1000;
% B60 = [0.230,-0.1125,0.017];
% % �ϱ����������ڵ㣨�����3����ϵ��
% U13 = [0.165,-0.030,0.135];
% U23 = [0.165,0.0197,0.154];
% U33 = [0.165,0.0197,-0.154];
% U43 = [0.165,-0.030,-0.135];
% U53 = [0.165,0,0.155];
% U63 = [0.165,0,-0.155];
% % ǰ�����������ڵ㣨�����4����ϵ��
% F14 = [0.122,-0.148,0];
% F24 = [0.122,0.148,0];

%% ���������ռ�
% thetaStart = [-50,-45,0,0]*pi/180;
% thetaEnd = [50,45,45,70]*pi/180;
% traverseWorkSpace(thetaStart,thetaEnd);

%% �ؽڿռ�켣�滮
% %�趨��ʼ�Ƕȣ���ֹ�Ƕȣ�����ʱ��
qStart = [0,0,0,0]*pi/180;
qEnd = [0,25,0,0]*pi/180;
qTf = 2;
qStep = 0.05;
[Theta,torque,JF_SH,JF_EL] = trajectoryPlanOnJoint(qStart,qEnd,qTf,qStep);
%% ���ι滮�����������
Tension = zeros(6,qTf/qStep+1);
% ��ؽ�
kk=0;
for t=0:qStep:qTf
    kk=kk+1;
    H2 = eye(2); f2= zeros(2,1); Aeq_elbow = -JF_EL(:,:,kk); beq_elbow  = [0;0;torque(4,kk)] ; lb2 = 2 * ones(2,1); ub2 = 100*ones(2,1); x02 = 5 * ones(2,1);
    [TT_elbow,fval,exitflag] = quadprog(H2,f2,[],[],Aeq_elbow ,beq_elbow ,lb2,ub2,x02);
    Tension(5,kk)=TT_elbow(1);
    Tension(6,kk)=TT_elbow(2);
end
% ��ؽ�
kk=0;
for t=0:qStep:qTf
    kk=kk+1;
    TorReq_GH=[torque(1,kk);torque(2,kk);torque(3,kk)];
    H1 = eye(4); f1= zeros(4,1); Aeq_GH = -JF_SH(:,:,kk); beq_GH = TorReq_GH; lb1 = 2 * ones(4,1); ub1 = 100*ones(4,1); x01 = 5 * ones(4,1);
    [TT_GH,fval,exitflag] = quadprog(H1,f1,[],[],Aeq_GH,beq_GH,lb1,ub1,x01);
    Tension(1,kk)=TT_GH(1);
    Tension(2,kk)=TT_GH(2);
    Tension(3,kk)=TT_GH(3);
    Tension(4,kk)=TT_GH(4);
end
% ���������仯����
figure();
t=0:qStep:qTf;
subplot(2,1,1);
plot(t,Tension(1,:),'r',t,Tension(2,:),'b',t,Tension(3,:),'c',t,Tension(4,:),'g','LineWidth',1.5);
title('shoulder module');
xlabel('time ( s )');
ylabel('cable tension ( N )');
legend('F1','F2','F3','F4');
grid on;

subplot(2,1,2);
plot(t,Tension(5,:),'m',t,Tension(6,:),'k','LineWidth',1.5);
title('elbow module');
xlabel('time ( s )');
ylabel('cable tension ( N )');
legend('F5','F6');
grid on;

%% �նȱ仯
[K_ELBOW_Value,K_SH_Value] = Stiffness(Theta,Tension);

%% ֱ�߲岹
% % ѡ����ʼ�㣬��ֹ�㣬�ٶȣ����ٶȣ��岹��������ʼ��ѡȡ����theta1=theta2=theta3=theta4=0,��ֹ��ѡȡ����theta1=0,theta2=pi/6,theta3=pi/6,theta4=pi/3��
% q0 = [0;0;0;pi/60];
% T010 = DH(0,0,0,q0(1));
% T120 = DH(0,-pi/2,0,q0(2)-pi/2);
% T230 = DH(0,pi/2,0,q0(3));
% T340 = DH(LenUpperarm,-pi/2,0,q0(4));
% T450 = DH(LenForeArm,0,0,0);
% T050 = T010*T120*T230*T340*T450;
% pos_start = T050;
% ps = pos_start(1:3,4); % ��ʼ��λ��
% os = pos_start(1:3,1:3); % ��ʼ����̬
% q1 = [0,10,20,15]*pi/180;
% T011 = DH(0,0,0,q1(1));
% T121 = DH(0,-pi/2,0,q1(2)-pi/2);
% T231 = DH(0,pi/2,0,q1(3));
% T341 = DH(LenUpperarm,-pi/2,0,q1(4));
% T451 = DH(LenForeArm,0,0,0);
% T051 = T011*T121*T231*T341*T451;
% pos_end = T051;
% pe = pos_end(1:3,4); % ĩ�˵�λ��
% oe = pos_end(1:3,1:3); % ĩ�˵���̬
% vel=50; % �ٶ�
% acc=100; % ���ٶ�
% k=100; % �岹����
% [R,p,t] = interpolation(pe,ps,oe,os,vel,acc,k);
% % t_interval = 0.1;
% % k = 20;
% % k_acc = 2;
% % [R,p,t] = interpolation_forQt(pe,ps,oe,os,t_interval,k_acc,k);
% 
% % ���˶�ѧ���Ƕ�
% q = zeros(4,k+1);
% %q0 = [0;0;0;1]*pi/180;
% pos = zeros(3,101);
% for i=2:(k+1)
%     q(:,i) = JacobiIK(R(:,:,i),p(:,i),q0);
%     q0 = q(:,i);
%     pos(:,i) = p(:,i);
%     hold off;
% end
% hold on;
% h = plot3(p(1,:),p(2,:),p(3,:),'--b');
% h.LineWidth = 2;
% 
% length_Line = zeros(6,k+1);
% for i=1:(k+1)
%     length_Line(:,i) = CableLength(q(:,i));
% end
% 
% % ����ؽڽǶ�����
% figure();
% xlabel('ʱ��/s');
% ylabel('�ؽڽǶ�/rad');
% title('�ؽڽǶȱ仯');
% grid on;
% plot(t(2:end),real(q(1,2:end)),'k',t(2:end),real(q(2,2:end)),'g',t(2:end),real(q(3,2:end)),'b',t(2:end),real(q(4,2:end)),'r','LineWidth',2);
% legend('theta1','theta2','theta3','theta4');
% 
% figure();
% subplot(1,2,1);
% plot(t(2:end),length_Line(1,2:end),'k',t(2:end),length_Line(2,2:end),'g',t(2:end),length_Line(3,2:end),'b',...
%     t(2:end),length_Line(4,2:end),'r','LineWidth',2);
% legend('L1','L2','L3','L4');
% xlabel('ʱ��/s');
% ylabel('��������/mm');
% title('��ؽ�ģ��');
% subplot(1,2,2);
% plot(t(2:end),length_Line(5,2:end),'g',t(2:end),length_Line(6,2:end),'b','LineWidth',2);
% legend('L5','L6');
% xlabel('ʱ��/s');
% ylabel('��������/mm');
% title('��ؽ�ģ��');
% 
% % �����˶�ѧ��֤
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
% %hold on;
% %scatter3(p(1,:),p(2,:),p(3,:),'r');
% title('����ؽڽǶ�ֵ��ֱ֤��');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% view(-30,10);
% grid on;
% 
% figure();
% scatter3(p(1,:),p(2,:),p(3,:),'r');
% title('�岹ֱ��');
% xlabel('x/mm');
% ylabel('y/mm');
% zlabel('z/mm');
% view(-30,10);
% grid on;

%% ���ؽڸն�
% K_module = JF_SH(JF_EL)*K_NSU*JF_SH'(JF_EL')
% K_NSU = diag(k1,...,kn)
T = [0,0,0,0,0,0];

%% ��ؽڸն�
% q4_max = 45;
% q4_min = -45;
% interval_q4 = 1;
% theta4_nsu = q4_min:interval_q4:q4_max;
% length = (q4_max-q4_min)/interval_q4 + 1;
% K_ELBOW = zeros(6,length);
% 
% T(6) = 5.1;
% KCable_6 = nsu(T(6));
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = nsu(T(5));
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(1,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% figure();
% % plot(theta4_nsu,K_ELBOW(1,:),'k','LineWidth',1.5);
% % hold on;
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(1,:),x,'PCHIP');
% semilogy(x*pi/180,y,'k','LineWidth',1.5);
% xlabel('\theta(rad)');
% ylabel('K(N\cdotmm/rad)');
% hold on;
% y1 = K_ELBOW(1,:);
% 
% T(6) = 5.3;
% KCable_6 = nsu(T(6));
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = nsu(T(5));
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(2,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% % plot(theta4_nsu,K_ELBOW(2,:),'g','LineWidth',1.5);
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(2,:),x,'PCHIP');
% semilogy(x*pi/180,y,'b','LineWidth',1.5);
% hold on;
% y2 = K_ELBOW(2,:);
% 
% T(6) = 5.6;
% KCable_6 = nsu(T(6));
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = nsu(T(5));
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(3,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% % plot(theta4_nsu,K_ELBOW(3,:),'b','LineWidth',1.5);c
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(3,:),x,'PCHIP');
% semilogy(x*pi/180,y,'m','LineWidth',1.5);
% hold on;
% y3 = K_ELBOW(3,:);
% 
% T(6) = 6;
% KCable_6 = nsu(T(6));
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = nsu(T(5));
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(4,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% % plot(theta4_nsu,K_ELBOW(4,:),'r','LineWidth',1.5);
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(4,:),x,'PCHIP');
% semilogy(x*pi/180,y,'r','LineWidth',1.5);
% hold on;
% y4 = K_ELBOW(4,:);
% 
% T(6) = 2;
% KCable_6 = KCable(round(T(6)/0.05)+1);
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = KCable(round(T(5)/0.05)+1);
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(5,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% % plot(theta4_nsu,K_ELBOW(4,:),'r','LineWidth',1.5);
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(5,:),x,'PCHIP');
% semilogy(x*pi/180,y,'k','LineWidth',1.5);
% hold on;
% y5 = K_ELBOW(5,:);
% 
% T(6) = 20;
% KCable_6 = KCable(round(T(6)/0.05)+1);
% for k=1:length
%     q = [0,0,0,theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(5) = (mElbow*g*LenForeArm*sin(theta4_nsu(k)*pi/180)/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_5 = KCable(round(T(5)/0.05)+1);
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(6,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% % plot(theta4_nsu,K_ELBOW(4,:),'r','LineWidth',1.5);
% x = linspace(min(theta4_nsu),max(theta4_nsu));
% y = interp1(theta4_nsu,K_ELBOW(6,:),x,'PCHIP');
% semilogy(x*pi/180,y,'k','LineWidth',1.5);
% hold on;
% y6 = K_ELBOW(6,:);
% 
% y1_y2 = [y5;y6];
% maxY1vsY2=max(y1_y2);
% minY1vsY2=min(y1_y2);
% yForFill=[maxY1vsY2,fliplr(minY1vsY2)];
% xForFill=[theta4_nsu,fliplr(theta4_nsu)];
% fill(xForFill,yForFill,'y','FaceAlpha',0.5,'EdgeAlpha',1,'EdgeColor','k'); % ��䲢����ͼ�θ�ʽ
% % 
% % Ӧ�����������ԽǶ���Ϊ������ѭ����ȷ��һ�����ϵ�������С���ֵ����һ�����ϵ��������ݾ���ƽ�⣨�������������
% % ����Ӧ�þ��������ڽ�һ������Ϊλ�ÿ��ƣ���һ�������������ƣ����Ǿ�ֹ���������ƽ�⣬�������Ƶ��Ǹ��������仯����һ�����ϵ�����Ҳ����ݾ���ƽ����仯��
% % �ͳ���˵һ�£�����������������
% 
% % ������ĳ���̶��Ƕȣ�һ�������������仯����һ�����ϵ��������ݾ���ƽ�⣨�������������
% q = [0,0,0,30]*pi/180;
% T6_max = 20;
% T6_min = 3;
% interval_T6 = 0.05;
% T6 = T6_min:interval_T6:T6_max;
% length = (T6_max-T6_min)/interval_T6 + 1;
% K_ELBOW = zeros(2,length);
% for k=1:length
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     T(6) = T6(k);
%     T(5) = (mElbow*g*LenForeArm*sin(q(4))/2-JF_EL(3,2)*T(6))/JF_EL(3,1);
%     KCable_6 = KCable(round(T(6)/0.05)+1);
%     KCable_5 = KCable(round(T(5)/0.05)+1);
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(1,k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end
% plot(T6,K_ELBOW(1,:),'LineWidth',1.5);
% 
% % ������ά���ߣ�����x����T6��y����theta4��z����T5
% % ������ά���ߣ�����x����T6��y����theta4��z����K_EL
% T6_Start = 3;
% T6_End = 20;
% T6_step = 0.5;
% Theta4_Start = 0;
% Theta4_End = 45;
% Theta4_step = 1;
% num  = ((T6_End-T6_Start)/T6_step+1)*((Theta4_End-Theta4_Start)/Theta4_step+1);
% T5_tra = zeros(3,ceil(num));
% K_EL = zeros(3,ceil(num));
% i = 0;
% for T6_sp=T6_Start:T6_step:T6_End
%     for Theta4_sp=Theta4_Start:Theta4_step:Theta4_End
%         i = i + 1;
%         q = [0,0,0,Theta4_sp*pi/180];
%         [JF_SH,JF_EL] = TensiontoTorque(q);
%         T5_tra(1,i) = T6_sp;
%         T5_tra(2,i) = Theta4_sp;
%         T5_tra(3,i) = (mElbow*g*LenForeArm*sin(q(4))/2-JF_EL(3,2)*T6_sp)/JF_EL(3,1);
%         KCable_6 = nsu(T6_sp);
%         KCable_5 = nsu(T5_tra(3,i));
% %         KCable_6 = KCable(round(T6_sp/0.05)+1);
% %         KCable_5 = KCable(round(T5_tra(3,i)/0.05)+1);
%         KCable_el = diag([KCable_5,KCable_6]);
%         K_EL(1,i) = T6_sp;
%         K_EL(2,i) = Theta4_sp;
%         K_EL(3,i) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
%     end
% end
% % ��������
% % һ���뷨��ʹ��griddata��ֵ�õ�nsu�ն�ֵ���ȼ���l4���������Ĺ�ϵ���ٲ�ֵ����������Ӧ��l4���ټ���նȣ�����Ӧ�û��ࣩܶ
% % �����뷨��������ʹ��interp1(һά��ֵ)
% % ��������뷨��������
% % f(t6,theta4,t5)
% figure();
% [X,Y] = meshgrid(T6_Start:0.05:T6_End,Theta4_Start:0.1:Theta4_End);
% Z = griddata(T5_tra(1,:),T5_tra(2,:),T5_tra(3,:),X,Y);
% colormap(jet);
% surf(X,Y*pi/180,Z);
% shading interp;
% xlabel('\tau_1(N)');
% ylabel('\theta_4(rad)');
% zlabel('\tau_2(N)')
% colorbar;
% 
% % f(t6,theta4,k_el)
% figure();
% [X,Y] = meshgrid(T6_Start:0.05:T6_End,Theta4_Start:0.1:Theta4_End);
% Z = griddata(K_EL(1,:),K_EL(2,:),K_EL(3,:),X,Y);
% colormap(jet);
% surf(X,Y*pi/180,-Z);
% shading interp;
% xlabel('\tau_1(N)');
% ylabel('\theta4(rad)');
% zlabel('K_el(N\cdotmm/rad)');
% colorbar;
% 
% % f(t6,t5,k_el)
% figure();
% [X,Y] = meshgrid(T6_Start:0.05:T6_End,min(T5_tra(3,:)):0.05:max(T5_tra(3,:)));
% Z = griddata(K_EL(1,:),T5_tra(3,:),K_EL(3,:),X,Y);
% colormap(jet);
% surf(X,Y,Z);
% shading interp;
% xlabel('\tau_1(N)');
% ylabel('\tau_2(N)');
% zlabel('K_el(N\cdotmm/rad)');
% colorbar;

% % �����ն����ߣ�����T5��T6
% q4_max = 45;
% interval_q4 = 1;
% theta4_nsu = 0:interval_q4:q4_max;
% length = q4_max/interval_q4 + 1;
% K_ELBOW = zeros(2,length);
% 
% AimK_EL_Start = -6.4e4;
% AimK_EL_End = -7.4e4;
% Theta4_Start = 0;
% Theta4_End = 45;
% Theta4_step = 1;
% Theta4_nsu = Theta4_Start:Theta4_step:Theta4_End;
% num = (Theta4_End-Theta4_Start)/Theta4_step+1;
% Aim_T5 = zeros(1,num);
% Aim_T6 = zeros(1,num);
% AimK_EL = zeros(1,num);
% K_elResult = zeros(1,num);
% % ˣһ��Сթ������t6���ڲ��ϼ�С�ģ���ζ���ҵ���һ��t6�󣬼�ס�����ϼ�С������t5
% t6_last = 0;
% t6found_Flag = 0;
% t6increase_Flag = 0;
% KEL_Last = 0;
% for k=1:num
%     q = [0,0,0,Theta4_nsu(k)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);  
%     if t6found_Flag == 0
%         for t6=5.1:0.01:20     
%             t5 = (mElbow*g*LenForeArm*sin(q(4))/2-JF_EL(3,2)*t6)/JF_EL(3,1);
%             if t5<50
%                 if t5>0
%                     KCable_6 = double(nsu(t6));
%                     KCable_5 = double(nsu(t5));
%     %                 KCable_6 = KCable(round(t6/0.05)+1);
%     %                 KCable_5 = KCable(round(t5/0.05)+1);
%                     KCable_el = diag([KCable_5,KCable_6]);
%                     K_EL = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
%                     KEL_Last = K_EL;
%                     AimK_EL(k) = AimK_EL_Start+(AimK_EL_End-AimK_EL_Start)*k/num;
%                     %K_EL/AimK_EL
%                     if K_EL/AimK_EL(k)>0.999
%                         Aim_T5(k) = t5;
%                         Aim_T6(k) = t6;
%                         K_elResult(k) = K_EL;
%                         t6_last = t6;
%                         break;                        
%                     end
%                 end
%             end   
%         end
%         t6found_Flag = 1;
%     else
%         for loop=0:100
%             t6 = t6 + loop*0.01;
%             t5 = (mElbow*g*LenForeArm*sin(q(4))/2-JF_EL(3,2)*t6)/JF_EL(3,1); 
%             if t5<50
%                 if t5>0
%                     KCable_6 = double(nsu(t6));
%                     KCable_5 = double(nsu(t5));
%                     KCable_el = diag([KCable_5,KCable_6]);
%                     K_EL = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
%                     KEL_Last = K_EL;
%                     AimK_EL(k) = AimK_EL_Start+(AimK_EL_End-AimK_EL_Start)*k/num;
%                     %K_EL/AimK_EL
%                     if K_EL/AimK_EL(k)>0.999
%                         Aim_T5(k) = t5;
%                         Aim_T6(k) = t6;
%                         K_elResult(k) = K_EL;
%                         t6_last = t6;
%                         break;                        
%                     end
%                 end
%             end 
%         end
%     end
% 
% %     syms x y; % x:t5, y:t6
% %     eq1 = JF_EL(3,1)*x+JF_EL(3,2)*y == mElbow*g*LenForeArm*sin(q(4))/2;
% %     eq2 = JF_EL(3,:)*diag([KCable(round(x/0.05)+1),KCable(round(y/0.05)+1)])*(JF_EL(3,:))' == AimK_EL; % KCable(round(x/0.05)+1)������Unable to convert expression into double array.
% %     [Aim_T5(k),Aim_T6(k)] = vpasolve([eq1,eq2],[x,y]);
% end
% figure();
% x = linspace(min(Theta4_nsu),max(Theta4_nsu));
% y = interp1(Theta4_nsu,Aim_T5,x,'PCHIP');
% semilogy(x*pi/180,y,'k','LineWidth',1.5);
% % plot(Theta4_nsu,Aim_T5,'LineWidth',1.5);
% hold on;
% x = linspace(min(Theta4_nsu),max(Theta4_nsu));
% y = interp1(Theta4_nsu,Aim_T6,x,'PCHIP');
% semilogy(x*pi/180,y,'b','LineWidth',1.5);
% % plot(Theta4_nsu,Aim_T6,'LineWidth',1.5);
% 
% figure();
% plot(Theta4_nsu,K_elResult,'LineWidth',1.5);
% hold on;
% plot(Theta4_nsu,AimK_EL,'LineWidth',1.5);
% clear global
% 
% figure();
% subplot(2,1,1);
% plot(Theta4_nsu*pi/180,Aim_T5,'k','LineWidth',1.5);
% hold on;
% plot(Theta4_nsu*pi/180,Aim_T6,'b','LineWidth',1.5);
% legend('\tau_5','\tau_6');
% xlabel('\theta_4(rad)');
% ylabel('\tau(N)');
% subplot(2,1,2);
% plot(Theta4_nsu*pi/180,AimK_EL,'r','LineWidth',1.5);
% hold on;
% plot(Theta4_nsu*pi/180,K_elResult,'b--','LineWidth',0.5);
% legend('simulation result','experiment result');
% xlabel('\theta_4(rad)');
% ylabel('K_{el}(N\cdotmm/rad)');

%% ��ؽڸն�
% K_module = JF_SH(JF_EL)*K_NSU*JF_SH'(JF_EL')
% K_NSU = diag(k1,...,kn)

%% ���ؽ����ĸ���ؽڽ��£�t1�仯���¹ؽڸնȱ仯����
% ��һ�ֹؽڽ�
% q = [0,0,0,0]*pi/180;
% ��������3����ϵ�µĵ����� r*f
% R
% R_Upper = [LenUpperarm/2;0;0];
% T01 = DH(0,0,0,q(1));
% T12 = DH(0,-pi/2,0,q(2)-pi/2);
% T23 = DH(0,pi/2,0,q(3));
% T34 = DH(LenUpperarm,-pi/2,0,q(4));
% T03 = T01*T12*T23;
% T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
% �ϱ���3�����µ�����
% Upper_Gravity_0 = [0;0;mShoulder*g];
% R30 = T30(1:3,1:3);
% Upper_Gravity_3 = R30*Upper_Gravity_0;
% M_Upper = cross(R_Upper,Upper_Gravity_3);
% ǰ����3����ϵ�µ�����
% R_Fore_4 = [LenForeArm/2;0;0;1];
% R_Fore_3 = T34*R_Fore_4;
% R_Fore_3 = R_Fore_3(1:3);
% ǰ����3�����µ�����
% Fore_Gravity_0 = [0;0;mElbow*g];
% R30 = T30(1:3,1:3);
% Fore_Gravity_3 = R30*Fore_Gravity_0;
% M_Fore = cross(R_Fore_3,Fore_Gravity_3);
% M_Shoulder = M_Upper+M_Fore;
% ����ڹ̶��Ƕ��£�ͨ��ȷ��һ�������ϵ�������������������ϵ�����
% �����Ļ�����3�����̽�3��δ֪��
% [JF_SH,~] = TensiontoTorque(q); 
% t1_max = 20;
% t1_min = 2;
% interval_t1 = 2;
% t1 = t1_min:interval_t1:t1_max;
% length = (t1_max-t1_min)/interval_t1 + 1;
% tension2_4 = zeros(3,length);
% K_SH_Value = zeros(4,length);
% for k=1:length
%     A = JF_SH(:,2:4);
%     b = [-M_Shoulder(1)-JF_SH(1,1)*t1(k);-M_Shoulder(2)-JF_SH(2,1)*t1(k);-M_Shoulder(3)-JF_SH(3,1)*t1(k)];
%     x = A\b;
%     tension2_4(:,k) = x;
%     KCable_1 = double(nsu(t1(k)));
%     KCable_2 = double(nsu(x(1)));
%     KCable_3 = double(nsu(x(2)));
%     KCable_4 = double(nsu(x(3)));
%     KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
%     K_SH = JF_SH*KCable_sh*(JF_SH)';
%     ���ؽڸնȾ���ļ�
%     K_SH_Value(1,k) = trace(K_SH);
% end
% figure();
% plot(t1,tension2_4,'LineWidth',1.5);
% figure();
% plot(t1,-K_SH_Value(1,:),'k','LineWidth',1.5);
% hold on;
% 
% �ڶ��ֹؽڽ�
% q = [20,20,20,0]*pi/180;
% ��������3����ϵ�µĵ����� r*f
% R
% R_Upper = [LenUpperarm/2;0;0];
% T01 = DH(0,0,0,q(1));
% T12 = DH(0,-pi/2,0,q(2)-pi/2);
% T23 = DH(0,pi/2,0,q(3));
% T34 = DH(LenUpperarm,-pi/2,0,q(4));
% T03 = T01*T12*T23;
% T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
% �ϱ���3�����µ�����
% Upper_Gravity_0 = [0;0;mShoulder*g];
% R30 = T30(1:3,1:3);
% Upper_Gravity_3 = R30*Upper_Gravity_0;
% M_Upper = cross(R_Upper,Upper_Gravity_3);
% ǰ����3����ϵ�µ�����
% R_Fore_4 = [LenForeArm/2;0;0;1];
% R_Fore_3 = T34*R_Fore_4;
% R_Fore_3 = R_Fore_3(1:3);
% ǰ����3�����µ�����
% Fore_Gravity_0 = [0;0;mElbow*g];
% R30 = T30(1:3,1:3);
% Fore_Gravity_3 = R30*Fore_Gravity_0;
% M_Fore = cross(R_Fore_3,Fore_Gravity_3);
% M_Shoulder = M_Upper+M_Fore;
% ����ڹ̶��Ƕ��£�ͨ��ȷ��һ�������ϵ�������������������ϵ�����
% �����Ļ�����3�����̽�3��δ֪��
% [JF_SH,~] = TensiontoTorque(q); 
% t1_max = 20;
% t1_min = 2;
% interval_t1 = 1;
% t1 = t1_min:interval_t1:t1_max;
% length = (t1_max-t1_min)/interval_t1 + 1;
% tension2_4 = zeros(3,length);
% for k=1:length
%     A = JF_SH(:,2:4);
%     b = [-M_Shoulder(1)-JF_SH(1,1)*t1(k);-M_Shoulder(2)-JF_SH(2,1)*t1(k);-M_Shoulder(3)-JF_SH(3,1)*t1(k)];
%     x = A\b;
%     tension2_4(:,k) = x;
%     KCable_1 = double(nsu(t1(k)));
%     KCable_2 = double(nsu(x(1)));
%     KCable_3 = double(nsu(x(2)));
%     KCable_4 = double(nsu(x(3)));
%     KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
%     K_SH = JF_SH*KCable_sh*(JF_SH)';
%     ���ؽڸնȾ���ļ�
%     K_SH_Value(2,k) = trace(K_SH);
% end
% plot(t1,-K_SH_Value(2,:),'b','LineWidth',1.5);
% hold on;
% 
% �����ֹؽڽ�
% q = [10,13,15,0]*pi/180;
% ��������3����ϵ�µĵ����� r*f
% R
% R_Upper = [LenUpperarm/2;0;0];
% T01 = DH(0,0,0,q(1));
% T12 = DH(0,-pi/2,0,q(2)-pi/2);
% T23 = DH(0,pi/2,0,q(3));
% T34 = DH(LenUpperarm,-pi/2,0,q(4));
% T03 = T01*T12*T23;
% T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
% �ϱ���3�����µ�����
% Upper_Gravity_0 = [0;0;mShoulder*g];
% R30 = T30(1:3,1:3);
% Upper_Gravity_3 = R30*Upper_Gravity_0;
% M_Upper = cross(R_Upper,Upper_Gravity_3);
% ǰ����3����ϵ�µ�����
% R_Fore_4 = [LenForeArm/2;0;0;1];
% R_Fore_3 = T34*R_Fore_4;
% R_Fore_3 = R_Fore_3(1:3);
% ǰ����3�����µ�����
% Fore_Gravity_0 = [0;0;mElbow*g];
% R30 = T30(1:3,1:3);
% Fore_Gravity_3 = R30*Fore_Gravity_0;
% M_Fore = cross(R_Fore_3,Fore_Gravity_3);
% M_Shoulder = M_Upper+M_Fore;
% ����ڹ̶��Ƕ��£�ͨ��ȷ��һ�������ϵ�������������������ϵ�����
% �����Ļ�����3�����̽�3��δ֪��
% [JF_SH,~] = TensiontoTorque(q); 
% t1_max = 20;
% t1_min = 2;
% interval_t1 = 1;
% t1 = t1_min:interval_t1:t1_max;
% length = (t1_max-t1_min)/interval_t1 + 1;
% tension2_4 = zeros(3,length);
% for k=1:length
%     A = JF_SH(:,2:4);
%     b = [-M_Shoulder(1)-JF_SH(1,1)*t1(k);-M_Shoulder(2)-JF_SH(2,1)*t1(k);-M_Shoulder(3)-JF_SH(3,1)*t1(k)];
%     x = A\b;
%     tension2_4(:,k) = x;
%     KCable_1 = double(nsu(t1(k)));
%     KCable_2 = double(nsu(x(1)));
%     KCable_3 = double(nsu(x(2)));
%     KCable_4 = double(nsu(x(3)));
%     KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
%     K_SH = JF_SH*KCable_sh*(JF_SH)';
%     ���ؽڸնȾ���ļ�
%     K_SH_Value(3,k) = trace(K_SH);
% end
% plot(t1,-K_SH_Value(3,:),'r','LineWidth',1.5);
% hold on;
% 
% �����ֹؽڽ�
% q = [0,0,20,0]*pi/180;
% ��������3����ϵ�µĵ����� r*f
% R
% R_Upper = [LenUpperarm/2;0;0];
% ���˱任����
% T01 = DH(0,0,0,q(1));
% T12 = DH(0,-pi/2,0,q(2)-pi/2);
% T23 = DH(0,pi/2,0,q(3));
% T34 = DH(LenUpperarm,-pi/2,0,q(4));
% T03 = T01*T12*T23;
% T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
% �ϱ���3�����µ�����
% Upper_Gravity_0 = [0;0;mShoulder*g];
% R30 = T30(1:3,1:3);
% Upper_Gravity_3 = R30*Upper_Gravity_0;
% M_Upper = cross(R_Upper,Upper_Gravity_3);
% ǰ����3����ϵ�µ�����
% R_Fore_4 = [LenForeArm/2;0;0;1];
% R_Fore_3 = T34*R_Fore_4;
% R_Fore_3 = R_Fore_3(1:3);
% ǰ����3�����µ�����
% Fore_Gravity_0 = [0;0;mElbow*g];
% R30 = T30(1:3,1:3);
% Fore_Gravity_3 = R30*Fore_Gravity_0;
% M_Fore = cross(R_Fore_3,Fore_Gravity_3);
% M_Shoulder = M_Upper+M_Fore;
% ����ڹ̶��Ƕ��£�ͨ��ȷ��һ�������ϵ�������������������ϵ�����
% �����Ļ�����3�����̽�3��δ֪��
% [JF_SH,JF_EL] = TensiontoTorque(q); 
% t1_max = 20;
% t1_min = 2;
% interval_t1 = 1;
% t1 = t1_min:interval_t1:t1_max;
% length = (t1_max-t1_min)/interval_t1 + 1;
% tension2_4 = zeros(3,length);
% for k=1:length
%     A = JF_SH(:,2:4);
%     b = [-M_Shoulder(1)-JF_SH(1,1)*t1(k);-M_Shoulder(2)-JF_SH(2,1)*t1(k);-M_Shoulder(3)-JF_SH(3,1)*t1(k)];
%     x = A\b;
%     tension2_4(:,k) = x;
%     KCable_1 = double(nsu(t1(k)));
%     KCable_2 = double(nsu(x(1)));
%     KCable_3 = double(nsu(x(2)));
%     KCable_4 = double(nsu(x(3)));
%     KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
%     K_SH = JF_SH*KCable_sh*(JF_SH)';
%     ���ؽڸնȾ���ļ�
%     K_SH_Value(4,k) = trace(K_SH);
% end
% plot(t1,-K_SH_Value(4,:),'m','LineWidth',1.5);
% legend("q_1=0,q_2=0.q_3=0","q_1=20,q_2=0,q_3=0","q_1=0,q_2=20,q_3=0","q_1=0,q_2=0,q_3=20");
% 


% % ����
% t1 = 10;
% qStart = [0,0,0,0]*pi/180;
% qEnd = [12,24,24,0]*pi/180;
% the1_start = qStart(1);
% the1_end = qEnd(1);
% the2_start = qStart(2);
% the2_end = qEnd(2);
% the3_start = qStart(3);
% the3_end = qEnd(3);
% step = 4*pi/180;
% num = ((the1_end-the1_start)/step+1)*((the2_end-the2_start)/step+1)*((the3_end-the3_start)/step+1);
% K_SH_Value = zeros(4,ceil(num));
% length1 = round((the1_end-the1_start)/step)+1;
% length2 = round((the2_end-the2_start)/step)+1;
% length3 = round((the3_end-the3_start)/step)+1;
% theta1 = the1_start:step:the1_end;
% theta2 = the2_start:step:the2_end;
% theta3 = the3_start:step:the3_end;
% X = zeros(length1,length2,length3);
% Y = zeros(length1,length2,length3);
% Z = zeros(length1,length2,length3);
% V = zeros(length1,length2,length3);
% for i = 1:length1
%     for j = 1:length2
%         for k = 1:length3      
%             X(i,j,k) = theta1(i);
%             Y(i,j,k) = theta2(j);
%             Z(i,j,k) = theta3(k);
%             % ��������3����ϵ�µĵ����� r*f
%             % R
%             R_Upper = [LenUpperarm/2;0;0];
%             % ���˱任����
%             T01 = DH(0,0,0,theta1(i));
%             T12 = DH(0,-pi/2,0,theta2(j)-pi/2);
%             T23 = DH(0,pi/2,0,theta3(k));
%             T34 = DH(LenUpperarm,-pi/2,0,0);
%             T03 = T01*T12*T23;
%             T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
%             % �ϱ���3�����µ�����
%             Upper_Gravity_0 = [0;0;mShoulder*g];
%             R30 = T30(1:3,1:3);
%             Upper_Gravity_3 = R30*Upper_Gravity_0;
%             M_Upper = cross(R_Upper,Upper_Gravity_3);
%             % ǰ����3����ϵ�µ�����
%             R_Fore_4 = [LenForeArm/2;0;0;1];
%             R_Fore_3 = T34*R_Fore_4;
%             R_Fore_3 = R_Fore_3(1:3);
%             % ǰ����3�����µ�����
%             Fore_Gravity_0 = [0;0;mElbow*g];
%             R30 = T30(1:3,1:3);
%             Fore_Gravity_3 = R30*Fore_Gravity_0;
%             M_Fore = cross(R_Fore_3,Fore_Gravity_3);
%             M_Shoulder = M_Upper+M_Fore;
% 
%             % ����ڹ̶��Ƕ��£�ͨ��ȷ��һ�������ϵ�������������������ϵ�����
%             % �����Ļ�����3�����̽�3��δ֪��
%             [JF_SH,~] = TensiontoTorque([theta1(i),theta2(j),theta3(k),0]); 
%             A = JF_SH(:,2:4);
%             b = [-M_Shoulder(1)-JF_SH(1,1)*t1;-M_Shoulder(2)-JF_SH(2,1)*t1;-M_Shoulder(3)-JF_SH(3,1)*t1];
%             x = A\b;
%             KCable_1 = double(nsu(t1));
%             KCable_2 = double(nsu(x(1)));
%             KCable_3 = double(nsu(x(2)));
%             KCable_4 = double(nsu(x(3)));
%             KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
%             K_SH = JF_SH*KCable_sh*(JF_SH)';
%             % ���ؽڸնȾ���ļ�
% 
%             K_SH_Value(4,i) = -trace(K_SH);
%             V(i,j,k) = trace(K_SH);
%         end
%     end
% end
% figure();
% [x,y,z] = meshgrid(the3_start:step:the3_end,the1_start:step:the1_end,the2_start:step:the2_end);
% xslice = [0.1 0.25 0.4];
% yslice = 0.1;
% zslice = [0.1 0.3];
% slice(x,y,z,-V,xslice,yslice,zslice);
% colormap(jet);
% shading interp;
% colorbar;
% xlabel('\theta_1(rad)');
% ylabel('\theta_2(rad)');
% zlabel('\theta_3(rad)');

% ����ĳ���̶��Ƕȣ�ʹ��նȱ仯�������������
q = [20,20,20,0]*pi/180;
[JF_SH,~] = TensiontoTorque(q);
KSH_Start = 4e5;
KSH_End = 12e5;
KSH_Step = 0.5e5;
KSH_Aim = KSH_Start:KSH_Step:KSH_End;
num = round((KSH_End-KSH_Start)/KSH_Step)+1;
AimTension1 = zeros(1,num);
AimTension2 = zeros(1,num);
AimTension3 = zeros(1,num);
AimTension4 = zeros(1,num);
% ��������3����ϵ�µĵ����� r*f
% R
R_Upper = [LenUpperarm/2;0;0];
% ���˱任����
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,0);
T45 = DH(LenForeArm,0,0,0);
T30 = (T01*T12*T23)';
T04 = T01*T12*T23*T34;
% �ϱ���3�����µ�����
Upper_Gravity_0 = [0;0;mShoulder*g];
R30 = T30(1:3,1:3);
Upper_Gravity_3 = R30*Upper_Gravity_0;
M_Upper = cross(R_Upper,Upper_Gravity_3);
% ǰ����3����ϵ�µ�����
R_Fore_4 = [LenForeArm/2;0;0;1];
R_Fore_3 = T34*R_Fore_4;
R_Fore_3 = R_Fore_3(1:3);
% ǰ����3�����µ�����
Fore_Gravity_0 = [0;0;mElbow*g];
R30 = T30(1:3,1:3);
Fore_Gravity_3 = R30*Fore_Gravity_0;
M_Fore = cross(R_Fore_3,Fore_Gravity_3);
M_Shoulder = M_Upper+M_Fore;
% �նȲ�����������Ҳ�ǲ�������
t1_last = 2;
K_shResult = zeros(1,num);
for k=1:num
    for t1=t1_last:0.1:20
        A = JF_SH(:,2:4);
        b = [-M_Shoulder(1)-JF_SH(1,1)*t1;-M_Shoulder(2)-JF_SH(2,1)*t1;-M_Shoulder(3)-JF_SH(3,1)*t1];
        x = A\b;
        KCable_1 = double(nsu(t1));
        KCable_2 = double(nsu(x(1)));
        KCable_3 = double(nsu(x(2)));
        KCable_4 = double(nsu(x(3)));
        KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
        K_SH = JF_SH*KCable_sh*(JF_SH)';
        % ���ؽڸնȾ���ļ� 
        K_SH_Result = -1*trace(K_SH);
        if K_SH_Result/KSH_Aim(k)>0.99
            AimTension1(k) = t1;
            AimTension2(k) = x(1);
            AimTension3(k) = x(2);
            AimTension4(k) = x(3);
            t1_last = t1;
            K_shResult(k) =  K_SH_Result;
            break;
        end      
    end
end
x = 1:num;
subplot(2,1,1);
plot(x,AimTension1,x,AimTension2,x,AimTension3,x,AimTension4,'LineWidth',1.5);
legend('\tau_1','\tau_2','\tau_3','\tau_4');
ylabel('\tau(N)');
subplot(2,1,2);
plot(x,KSH_Aim,'k','LineWidth',1.5);
hold on;
plot(x,K_shResult,'r--','LineWidth',1.5);
ylabel('K_{sh}(N\cdotmm/rad)');
legend('simulation result','experiment result');

%% �ɼ����Ĵ���������
% number = 2666;
% t = 1:number;
% %t = 1:310;
% data = xlsread("C:\Users\user\Desktop\20190506.xlsx");
% tension = data(t,1:6);
% angleElbow = data(t,7:9);
% angleShoulder = data(t,10:12);
% encoder = data(1:180,13:18);       
% figure();
% plot(tension);
% figure();
% plot(angleElbow);
% figure();
% plot(angleShoulder);
% figure();
% subplot(2,1,1);
% plot(t,tension(:,5),t,tension(:,6));
% legend("tension5","tension6");
% subplot(2,1,2);
% plot(t,angleElbow(:,3));
% legend("elbow");
% figure();
% plotyy(t,tension(:,5),t,angleElbow(:,3));
% 
% %% ���ݲɼ��������ݣ���5��6����������ؽڽǶȣ�������ؽڸն�
% K_ELBOW = zeros(1,number);
% for i=1:number
%     q = [0,0,0,-angleElbow(i,3)*pi/180];
%     [JF_SH,JF_EL] = TensiontoTorque(q);
%     if tension(i,6)<0
%         tension(i,6) = 0;
%     end
%     if tension(i,5)<0
%         tension(i,5) = 0;
%     end
%     KCable_6 = nsu(tension(i,6));
%     KCable_5 = nsu(tension(i,5));
%     KCable_el = diag([KCable_5,KCable_6]);
%     K_ELBOW(1,i) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
% end

% theta = [0,0,0,0]*pi/180;
% PlotUpperLimb(theta);

% detaL5 = zeros(1,11);
% k = 1;
% % test no meaning 
% for q=0:0.01:0.1
%     theta = [0,0,0,q]*pi/180;
%     length = CableLength(theta);
%     detaL5(k) = length(5);
%     k = k + 1;
% end
% detaL5 = detaL5 - detaL5(1);


