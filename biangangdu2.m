clear;
clc;
L3r = 25;
L2 = 30;
K = 8;
% F = zeros(1,901);
k = 1;
%syms L1;
%F = K*(L3r-sqrt(L1^2-L2^2))*L1/(2*sqrt(L1^2-L2^2));
L1 = zeros(1,801);
k=0;
FC = 0:0.1:80;
for F=0:0.1:80
k = k+1;
eq = @(x)F-K*(L3r-sqrt(x^2-L2^2))*x/(2*sqrt(x^2-L2^2));
[L1(k),fval] = fzero(eq,40);
end
plot(FC,L1);
detaL1 = 2*(L1 - L1(1)*ones(1,801));
detaL1 = -detaL1;
figure();
plot(detaL1,FC);
resultL1 = L1;
syms L1;
F = K*(L3r-sqrt(L1^2-L2^2))*L1/(2*sqrt(L1^2-L2^2));
Keq=diff(F,L1);
KeqResult = subs(Keq,L1,resultL1);
KeqResult = -KeqResult;
kkk = fliplr(KeqResult);
figure();
plot(resultL1(end:-1:1),KeqResult,'ko');
figure();
plot(resultL1,KeqResult,'ko');
figure();
plot(FC,KeqResult,'ko');

%求解绳索张力与绳索长度的关系
for L1=30:0.01:39
F(1,k) = K*(L3r-sqrt(L1^2-L2^2))*L1/(2*sqrt(L1^2-L2^2));
k = k+1;
end
figure(1);
plot(30:0.01:39,F(end:-1:1));
% % 
% % 求解绳索刚度与绳索张力的关系
% Keq=diff()
% syms L1;
% Fc= K*(L3r-sqrt(L1^2-L2^2))*L1/(2*sqrt(L1^2-L2^2));
% Keq=diff(Fc,L1);
% 
% for i=31:0.01:39
% Keq(1,k) =subs(Keq,L1,i);
% k = k+1;
% end
% figure(1);
% plot(31:0.01:39,Keq(end:-1:1));