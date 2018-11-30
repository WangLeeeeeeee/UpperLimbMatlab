function length = CableLength(q)
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

% 肩关节绳长
U10 = T03*[U13,1]';
U20 = T03*[U23,1]';
U30 = T03*[U33,1]';
U40 = T03*[U43,1]';
L1 = U10(1:3)' - B10; L2 = U20(1:3)' - B30; % cable vector
L3 = U30(1:3)' - B40; L4 = U40(1:3)' - B60;
l1 = norm(L1); l2 = norm(L2); % cable length
l3 = norm(L3); l4 = norm(L4);

%肘关节绳长
F10 = T04*[F14,1]';
F20 = T04*[F24,1]';
L5 = F10(1:3)' - B20;
L6 = F20(1:3)' - B50;
l5 = norm(L5); l6 = norm(L6);

length = [l1,l2,l3,l4,l5,l6];
end