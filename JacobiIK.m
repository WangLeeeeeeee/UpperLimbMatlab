% 利用雅可比解逆运动学
% R姿态矩阵，p位置矩阵, q0上一次迭代后的值，下一次迭代的初始值
function q = JacobiIK(R,p,q0)
% 上肢运动学参数
global LenUpperarm
global LenForeArm
%q = [0;0;0;3]*pi/180;
q = q0;
dq = [5;5;5;5]*pi/180;
Td = [R,p;zeros(1,3),1];
count = 0;
while (norm(dq)>0.0001 && count<200)
    count = count + 1;
    T01 = DH(0,0,0,q(1));
    T12 = DH(0,-pi/2,0,q(2)-pi/2);
    T23 = DH(0,pi/2,0,q(3));
    T34 = DH(LenUpperarm,-pi/2,0,q(4));
    T45 = DH(LenForeArm,0,0,0);
    T05 = T01*T12*T23*T34*T45;
    T(:,:,1) = T01;
    T(:,:,2) = T12;
    T(:,:,3) = T23;
    T(:,:,4) = T34;
    T(:,:,5) = T45;
    dt = Td - T05;
    [m,n]=size(dt);
    if (m==4 && n==4)
        v=[dt(3,2),-dt(3,1),dt(2,1)];
        w=[dt(1,4),dt(2,4),dt(3,4)];
        kesi=[w,v];
    else
        kesi=zeros(6,1);
    end
    DT = kesi;
    J = Jacobi_dif(T);
    Jp = pinv(J);
    dq = Jp*DT.';
    q = dq + q;
end
PlotUpperLimb([q(1),q(2),q(3),q(4)]);
drawnow();
end