function [q] = Inverse_kinetic(p,o)
q = zeros(4,1);
syms nx ny nz ox oy oz ax ay az px piy pz;
L04 = [nx ox ax px;
       ny oy ay piy;
       nz oz az pz;
       0 0 0 1];
% 末端位置
L = [o(1,1),o(1,2),o(1,3),p(1);
     o(2,1),o(2,2),o(2,3),p(2);
     o(3,1),o(3,2),o(3,3),p(3);
     0,0,0,1];
% 1. 利用L14(3,3)/L14(3,4) == R14(3,3)/R14(3,4)
% az = -s3c2, pz = 607c3c2/2
theta3_inv = atan2(-607*az,2*pz);
q(3) = subs(theta3_inv,L04(1:3,:),L(1:3,:));

%% test
%theta2_inv = acos((2*pz)/(607*cos(q(3))));
%q(2) = subs(theta2_inv,L04(1:3,:),L(1:3,:));
a = 607*cos(theta3_inv)/2;
b = 607*sin(theta3_inv)/2;
midsin = sqrt((px^2+piy^2-b^2)/a^2);
midcos = sqrt(1-midsin^2);
theta2_inv = atan2(midsin,midcos);
q(2) = subs(theta2_inv,L04(1:3,:),L(1:3,:));

%theta1_inv = atan2(607*cos(theta3_inv)*sin(theta2_inv)/2,607*sin(theta3_inv/2)/2) - atan2(px,piy);
theta1_inv = atan2(piy,px) - atan2(607*sin(theta3_inv)/2,607*cos(theta3_inv)*sin(theta2_inv)/2);
q(1) = subs(theta1_inv,L04(1:3,:),L(1:3,:));

% 2. 利用L14(2,4) == R14(2,4)
% pyc1-pxs1 = 607*s3/2
midSintheta1 = 607*sin(theta3_inv)/(2*sqrt(px^2+piy^2));
theta1_inv = atan2(piy,px) - atan2(midSintheta1,sqrt(1-midSintheta1^2));
q(1) = subs(theta1_inv,L04(1:3,:),L(1:3,:)); 
%theta1_inv = 0;
%q(1) = 0;
% 3. 利用L14(2,2)/L14(2,1) == R14(2,2)/R14(2,1)
% -s3s4/c4s3 = (oyc1-oxs1)/(nyc1-nxs1)
theta4_inv = atan2(-(oy*cos(theta1_inv)-ox*sin(theta1_inv)),(ny*cos(theta1_inv)-nx*sin(theta1_inv)));
%theta4_inv = asin(-oy*cos(theta1_inv)/sin(theta3_inv));
q(4) = subs(theta4_inv,L04(1:3,:),L(1:3,:));
q(4) = 0;
% 4. 利用L(1,3)/L(3,3) == R(1,3)/R(3,3)
% -s2s3/-s3c2 = (axc1+ays1)/az
%theta2_inv = atan2(-ax*cos(theta1_inv)+ay*sin(theta1_inv),-az);
%q(2) = subs(theta2_inv,L04(1:3,:),L(1:3,:));
end