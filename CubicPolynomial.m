%使用三次多项式插值对关节角进行规划
function [Theta,Omega,Beta]=CubicPolynomial(theta0,thetaf,tf,t)%输入参数包括起始角度、最终角度、总运行时间、当前时刻
%根据边界条件和公式求得相应系数
a0=theta0;%起始角度
a1=0;%第二项系数为零
a2=3*(thetaf-theta0)/(tf)^2;
a3=-2*(thetaf-theta0)/(tf)^3;
Theta=a0+a1*t+a2*t^2+a3*t^3;%当前角度
Omega=a1+2*a2*t+3*a3*t^2;%当前角速度
Beta=2*a2+6*a3*t;%当前角加速度