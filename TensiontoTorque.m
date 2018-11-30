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
T04 = T01*T12*T23*T34;

%肩关节力雅可比(3坐标系)
JF_SH = zeros(3,4);
B13 = T03(1:3,1:3)*B10';
B33 = T03(1:3,1:3)*B30';
B43 = T03(1:3,1:3)*B40';
B63 = T03(1:3,1:3)*B60';
L1 = U13' - B13; L2 = U23' - B33; % cable vector
L3 = U33' - B43; L4 = U43' - B63;
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
T40 = inv(T04);
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