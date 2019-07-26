% ͨ���ؽڽ���������غ���������֮��Ĺ�ϵ�����ϵ�������ٶȺ͹ؽڽ��ٶ�֮��Ĺ�ϵ����ȵ�
% ���ϵ���ſɱȾ���������ſɱȺ�һ�㶨����ſɱȣ��ؽڽ��ٶ���ĩ���ٶȵĹ�ϵ����һ������ʾ
% ��ؽ��ſɱȾ����ǲο�3����ϵ����ؽ��ſɱ��ǲο�4����ϵ
function [JF_SH,JF_EL] = TensiontoTorque(q)
% ��֫�˶�ѧ����
global LenUpperarm
global LenForeArm
% �����ڵ�����
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
% ���˱任����
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T03 = T01*T12*T23;
% ���任������棨����ֱ�����棬Ҳ����ͨ���������ַ�����
T30 = [(T03(1:3,1:3))',-(T03(1:3,1:3))'*T03(1:3,4);0 0 0 1];
T04 = T01*T12*T23*T34;
T40 = [(T04(1:3,1:3))',-(T04(1:3,1:3))'*T04(1:3,4);0 0 0 1];

%��ؽ����ſɱ�(3����ϵ)
% ��0����ϵת����3����ϵ
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

% ��ؽ����ſɱȣ�4����ϵ��
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