% 通过关节角求得外力矩和绳索张力之间的关系，其关系与绳索速度和关节角速度之间的关系是相等的
% 其关系用雅可比矩阵（这里的雅可比和一般定义的雅可比（关节角速度与末端速度的关系）不一样）表示
% 肩关节雅可比矩阵是参考3坐标系，肘关节雅可比是参考4坐标系
function [JF_SH,JF_EL] = TensiontoTorque(q)
% 上肢运动学参数
global LenUpperarm
global LenForeArm
% 绳索节点坐标
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
% 连杆变换矩阵
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T03 = T01*T12*T23;
% 求解变换矩阵的逆（可以直接求逆，也可以通过下面这种方法）
T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
T04 = T01*T12*T23*T34;
T40 = [(T04(1:3,1:3))',-(T04(1:3,1:3))'*T04(1:3,4);0 0 0 1];

%肩关节力雅可比(3坐标系)
% 将0坐标系转换到3坐标系
JF_SH = zeros(3,4);
B13 = T30(1:3,1:3)*B10';
B33 = T30(1:3,1:3)*B30';
B43 = T30(1:3,1:3)*B40';
B63 = T30(1:3,1:3)*B60';
L1 = B13 - U13'; L2 = B33 - U23'; % cable vector
L3 = B43 - U33'; L4 = B63 - U43';
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
end