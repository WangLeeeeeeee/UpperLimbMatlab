function PlotUpperLimb(q)

%% 上肢运动学参数
global LenUpperarm
global LenForeArm

%% 绳索节点坐标
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
global F14
global F24
% 基座上六个绳索节点（相对于0坐标系）
BArc0 = [0,0.142,0.033]; % 为了绘制圆弧
% 上臂四个绳索节点（相对于3坐标系）
UArc3 = [0.1715,0.125,0]; % 为了绘制圆弧
% 前臂两个绳索节点（相对于4坐标系）
FArc4 = [0.118,0,0.118];

%% 求变换矩阵
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

U10 = T03*[U13,1]';
U20 = T03*[U23,1]';
U30 = T03*[U33,1]';
U40 = T03*[U43,1]';
UArc0 = T03*[UArc3,1]';
F10 = T04*[F14,1]';
F20 = T04*[F24,1]';
FArc0 = T04*[FArc4,1]';

%% 绘制
%figure();
% 关节节点
JointSize = 0.5;
plot3(0,0,0,'ko','linewidth',JointSize);
hold on;
plot3(T04(1,4),T04(2,4),T04(3,4),'ko','linewidth',JointSize);
hold on;
plot3(T05(1,4),T05(2,4),T05(3,4),'ko','linewidth',JointSize);
hold on;
% 关节节点之间连杆
LinkSize = 0.6;
plot3([0,T04(1,4)],[0,T04(2,4)],[0,T04(3,4)],'k','LineWidth',LinkSize);
hold on;
plot3([T04(1,4),T05(1,4)],[T04(2,4),T05(2,4)],[T04(3,4),T05(3,4)],'k','LineWidth',LinkSize);
hold on;
% 绳节点
CableNodeSize = 0.2;
scatter3(B10(1),B10(2),B10(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(B20(1),B20(2),B20(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(B30(1),B30(2),B30(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(B40(1),B40(2),B40(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(B50(1),B50(2),B50(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(B60(1),B60(2),B60(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(U10(1),U10(2),U10(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(U10(1),U10(2),U10(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(U20(1),U20(2),U20(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(U30(1),U30(2),U30(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(U40(1),U40(2),U40(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(F10(1),F10(2),F10(3),'r','LineWidth',CableNodeSize);
hold on;
scatter3(F20(1),F20(2),F20(3),'r','LineWidth',CableNodeSize);
hold on;
plot3([B10(1),B20(1)],[B10(2),B20(2)],[B10(3),B20(3)],'r','LineWidth',CableNodeSize);
hold on;
plot3([B50(1),B60(1)],[B50(2),B60(2)],[B50(3),B60(3)],'r','LineWidth',CableNodeSize);
hold on;
plot3([U10(1),U20(1)],[U10(2),U20(2)],[U10(3),U20(3)],'r','LineWidth',CableNodeSize);
hold on;
plot3([U30(1),U40(1)],[U30(2),U40(2)],[U30(3),U40(3)],'r','LineWidth',CableNodeSize);
% 绳节点之间连线
NodeLineWidth = 0.1;
hold on;
plot3([B10(1),U10(1)],[B10(2),U10(2)],[B10(3),U10(3)],'b','LineWidth',NodeLineWidth);
hold on;
plot3([B30(1),U20(1)],[B30(2),U20(2)],[B30(3),U20(3)],'b','LineWidth',NodeLineWidth);
hold on;
plot3([B40(1),U30(1)],[B40(2),U30(2)],[B40(3),U30(3)],'b','LineWidth',NodeLineWidth);
hold on;
plot3([B60(1),U40(1)],[B60(2),U40(2)],[B60(3),U40(3)],'b','LineWidth',NodeLineWidth);
hold on;
plot3([B20(1),F10(1)],[B20(2),F10(2)],[B20(3),F10(3)],'b','LineWidth',NodeLineWidth);
hold on;
plot3([B50(1),F20(1)],[B50(2),F20(2)],[B50(3),F20(3)],'b','LineWidth',NodeLineWidth);
% 外骨骼圆弧
arcPlot([B20;BArc0;B50],0,'k');
arcPlot([U20(1:3)';UArc0(1:3)';U30(1:3)'],0,'k');
arcPlot([F10(1:3)';FArc0(1:3)';F20(1:3)'],0,'k');
% xlim([-400 400]);
% ylim([-350 350]);
% zlim([0 700]);
set(gca,'ZDir','reverse')%对Z方向反转
set(gca,'XDir','reverse')%对X轴方向反转
view(3);
hold off;
end