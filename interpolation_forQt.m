% ͨ��ָ����ʼλ�ˣ���ֹλ�ˣ�ÿС�εĲ岹ʱ�䣨��ӦQt�����з���ָ��Ķ�ʱ�����ڣ�һ��Ϊ100ms�������Ӽ��ٵĲ岹�������ܵĲ岹����
function [R,p,t] = interpolation_forQt(pe,ps,oe,os,t_interval,k_acc,k_all)
dist = norm(pe-ps);
R = zeros(3,3,k_all+1);
p = zeros(3,k_all+1);
t = 0:t_interval:k_all*t_interval;
p(:,1) = ps;
R(:,:,1) = os;
S = 0;
log = logm(os'*oe);
acc = dist/(k_acc^2+k_acc*(k_all-2*k_acc));
for k=2:k_all+1
    if(k<k_acc)
        dertaS = 0.5*(k*acc+(k-1)*acc)*1;
    elseif(k<=k_all-k_acc)
        dertaS = acc*k_acc*1;
    else
        dertaS = 0.5*((k_all-k)*acc+(k_all-k+1)*acc)*1;
    end
    p(:,k) = p(:,k-1) + (pe-ps)*dertaS/dist;
    S = S + dertaS;
    R(:,:,k) = os*expm(log*S/dist);
end   
end