% 计算动力学参数D,H,C
% 拉格朗日动力学方程：tau = D(q)*beta + h(q,omega) + C(q)
% 参考：http://www.cse.zju.edu.cn/eclass/attachments/2016-11/01-1479736065-215680.pdf
function tau = Dynamic(q,omega,beta)

% 上肢运动学参数
global LenUpperarm
global LenForeArm
global mShoulder 
global mElbow
% 重力大小
g = [0,0,9.81,0]; 
% 连杆变换矩阵
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T13 = T12*T23;
T14 = T13*T34;
T05 = T14*T45;
% 创建矩阵存放变换矩阵，注意下标
T = zeros(4,4,5,5);
T(:,:,1,2) = T01;
T(:,:,2,3) = T12;
T(:,:,3,4) = T23;
T(:,:,4,5) = T34;
T(:,:,1,3) = T02;
T(:,:,1,4) = T03;
T(:,:,1,5) = T04;
T(:,:,2,4) = T13;
T(:,:,2,5) = T14;
T(:,:,1,1) = eye(4);
T(:,:,2,2) = eye(4);
T(:,:,3,3) = eye(4);
T(:,:,4,4) = eye(4);
T(:,:,5,5) = eye(4);

rShoulder = 0.06;
rElbow = 0.05;
weight = zeros(4,1);
weight(1) = 0;
weight(2) = 0;
weight(3) = mShoulder;
weight(4) = mElbow;

% 计算伪惯性矩阵
% I = [(-Ixx+Iyy+Izz)/2 Ixy Ixz mi*xi; Ixy (Ixx-Iyy+Izz)/2 Iyz mi*yi; Ixz Iyz (Ixx+Iyy-Izz)/2 mi*zi; mi*xi mi*yi mi*zi mi]
% xi, yi, zi为质心相对于向前坐标的位置
I1 = zeros(4,4);
I2 = zeros(4,4);
Ixx3 = mShoulder*rShoulder*rShoulder/2;
Iyy3 = mShoulder*LenUpperarm*LenUpperarm/12;
Izz3 = mShoulder*LenUpperarm*LenUpperarm/12;
[Ixy3,Ixz3,Iyz3] = deal(0);
x3 = LenUpperarm/2;
y3 = 0;
z3 = 0;
m3 = mShoulder;
I3 = [(-Ixx3+Iyy3+Izz3)/2 Ixy3 Ixz3 m3*x3; Ixy3 (Ixx3-Iyy3+Izz3)/2 Iyz3 m3*y3; Ixz3 Iyz3 (Ixx3+Iyy3-Izz3)/2 m3*z3; m3*x3 m3*y3 m3*z3 m3];
Ixx4 = mElbow*rElbow*rElbow/2;
Iyy4 = mElbow*LenForeArm*LenForeArm/12;
Izz4 = mElbow*LenForeArm*LenForeArm/12;
[Ixy4,Ixz4,Iyz4] = deal(0);
x4 = LenForeArm/2;
y4 = 0;
z4 = 0;
m4 = mElbow;
I4 = [(-Ixx4+Iyy4+Izz4)/2 Ixy4 Ixz4 m4*x4; Ixy4 (Ixx4-Iyy4+Izz4)/2 Iyz4 m4*y4; Ixz4 Iyz4 (Ixx4+Iyy4-Izz4)/2 m4*z4; m4*x4 m4*y4 m4*z4 m4];
I = zeros(4,4,4);
I(:,:,1) = I1;
I(:,:,2) = I2;
I(:,:,3) = I3;
I(:,:,4) = I4;

% 计算D矩阵
% dT/dq = T*Q(注意和参考网站中不同)
Q = [0 -1 0 0; 1 0 0 0; 0 0 0 0; 0 0 0 0];
D = zeros(4,4);
U = zeros(4,4,4,4);
for i = 1:4
    for k = 1:4
        for j=max([i,k]):4
            U(:,:,j,k) = T(:,:,1,k+1)*Q*T(:,:,k+1,j+1);
            U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
            D(i,k) = D(i,k) + trace(U(:,:,j,k)*I(:,:,j)*U(:,:,j,i)');
        end
    end
end
% 计算H矩阵
H = zeros(4,1);
h = 0;
for i=1:4
    for k=1:4
        for m=1:4
            for j=max([i,k,m]):4
                if j>=m && m>=k
                    U_dev = T(:,:,1,k+1)*Q*T(:,:,k+1,m+1)*Q*T(:,:,m+1,j+1);
                else
                    U_dev = T(:,:,1,m+1)*Q*T(:,:,m+1,k+1)*Q*T(:,:,k+1,j+1);
                end
                U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
                h = h + trace(U_dev*I(:,:,j)*U(:,:,j,i));
            end
            H(i) = H(i) + h*omega(k)*omega(m);
        end
    end
end
% 计算C矩阵
C = zeros(4,1);
r = zeros(4,4);
r(:,1) = [0;0;0;1];
r(:,2) = [0;0;0;1];
r(:,3) = [LenUpperarm/2;0;0;1];
r(:,4) = [LenForeArm/2;0;0;1];
for i = 1:4
    for j=i:4
        U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
        C(i) = C(i) - weight(j)*g*U(:,:,j,i)*r(:,j);
    end
end
% 计算外部负载力矩在关节上产生的关节力矩
% 根据静力学公式：load_tau = Jacobi'*F
T(:,:,1) = T01;
T(:,:,2) = T12;
T(:,:,3) = T23;
T(:,:,4) = T34;
T(:,:,5) = T45;
Jacobi = Jacobi_dif(T);
Jp = pinv(Jacobi);
mload = 1.5;
load_F = [mload*g(1:3)';0;0;0];
load_J = Jp*load_F;
load_J(4) = -load_J(4);
% 计算关节力矩
tau = D*beta + H + C + load_J;
end
