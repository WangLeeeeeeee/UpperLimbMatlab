%% 二次规划求解绳索张力
Tension = zeros(6,len);
% 肘关节
for kk = 1:len
    [JF_SH,JF_EL] = TensiontoTorque(q_data(kk,:));
    H2 = eye(2); f2= zeros(2,1); Aeq_elbow = -JF_EL; beq_elbow  = [0;0;torque_data(kk,4)] ; lb2 = 2 * ones(2,1); ub2 = 100*ones(2,1); x02 = 5 * ones(2,1);
    [TT_elbow,fval,exitflag] = quadprog(H2,f2,[],[],Aeq_elbow ,beq_elbow ,lb2,ub2,x02);
    Tension(5,kk)=TT_elbow(1);
    Tension(6,kk)=TT_elbow(2);
end
% 肩关节
for kk=1:len
    [JF_SH,JF_EL] = TensiontoTorque(q_data(kk,:));
    TorReq_GH=[torque_data(kk,1);torque_data(kk,2);torque_data(kk,3)];
    H1 = eye(4); f1= zeros(4,1); Aeq_GH = JF_SH; beq_GH = TorReq_GH; lb1 = 2 * ones(4,1); ub1 = 100*ones(4,1); x01 = 5 * ones(4,1);
    [TT_GH,fval,exitflag] = quadprog(H1,f1,[],[],Aeq_GH,beq_GH,lb1,ub1,x01);
    Tension(1,kk)=TT_GH(1);
    Tension(2,kk)=TT_GH(2);
    Tension(3,kk)=TT_GH(3);
    Tension(4,kk)=TT_GH(4);
end
% 绳索张力变化曲线
figure();
t=q.time;
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

%% 刚度变化
[K_ELBOW_Value,K_SH_Value] = Stiffness(q_data',Tension);