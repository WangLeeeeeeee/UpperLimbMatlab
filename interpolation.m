function [R,p,t] = interpolation(pe,ps,oe,os,vel,acc,k)
tA = vel/acc; % 加速时间
dist = norm(pe-ps);
if (acc*tA*tA >= dist) 
    tA = sqrt(dist/acc);
    tS = 0; % 匀速时间
else
    tS = (dist-acc*tA*tA)/vel;
end
t_all = 2*tA + tS; % 总时间
t_interval = t_all/k; % 每段插补的时间
t=0:t_interval:t_all;
R = zeros(3,3,k+1);
p = zeros(3,k+1);
p(:,1) = ps;
R(:,:,1) = os;
S = 0;
log = logm(os'*oe);

if(tS == 0)
% no constant velocity
    for k=2:k+1
        if(t(k)<tA)
            dertaS = 0.5*acc*t_interval*(t(k)+t(k-1));
        elseif(t(k)>tA && t(k-1)<tA)
            dertaS = 0.5*acc*(tA-t(k-1))*(tA+t(k-1))+0.5*acc*(t(k)-tA)*(tA+tA-(t(k)-tA));
        else
            dertaS = 0.5*acc*((t_all-t(k))+(t_all-t(k-1)))*t_interval;
        end
        p(:,k) = p(:,k-1)+(pe-ps)*dertaS/dist;
        S = S + dertaS;
        R(:,:,k) = os*expm(log*S/dist);
    end
else
% with constant velocity
    for k=2:k+1
        if(t(k)<tA)
            dertaS = 0.5*acc*t_interval*(t(k)+t(k-1));
        elseif(t(k)<=tA+tS && t(k-1)<=tA)
            dertaS=vel*(t(k)-tA)+0.5*vel*tA-0.5*acc*t(k-1)*t(k-1);
        elseif(t(k)<=tA+S && t(k-1)<=tA+tS)
            dertaS = vel*t_interval;
        elseif(t(k)<=t_all && t(k-1)<=tA+tS)
            dertaS = -0.5*acc*(t(k)-tA-tS)*(t(k)-tA-tS)+vel*(t(k)-t(k-1));
        else
            dertaS=0.5*acc*((t_all-t(k))+(t_all-t(k-1)))*t_interval;
        end
        p(:,k) = p(:,k-1) + (pe-ps)*dertaS/dist;
        S = S + dertaS;
        R(:,:,k) = os*expm(log*S/dist);
    end                     
end
end
