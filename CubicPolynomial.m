%ʹ�����ζ���ʽ��ֵ�Թؽڽǽ��й滮
function [Theta,Omega,Beta]=CubicPolynomial(theta0,thetaf,tf,t)%�������������ʼ�Ƕȡ����սǶȡ�������ʱ�䡢��ǰʱ��
%���ݱ߽������͹�ʽ�����Ӧϵ��
a0=theta0;%��ʼ�Ƕ�
a1=0;%�ڶ���ϵ��Ϊ��
a2=3*(thetaf-theta0)/(tf)^2;
a3=-2*(thetaf-theta0)/(tf)^3;
Theta=a0+a1*t+a2*t^2+a3*t^3;%��ǰ�Ƕ�
Omega=a1+2*a2*t+3*a3*t^2;%��ǰ���ٶ�
Beta=2*a2+6*a3*t;%��ǰ�Ǽ��ٶ�