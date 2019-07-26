% nsu: 变刚度模块刚度计算
function KCable = nsu(tension)

% 初始化参数
L40 = 50; % 弹簧初始长度
L2 = 30; % 左右滑轮之间距离的一半
K = 1; % 两个弹簧刚度之和
H = 75; % 变刚度模块高度

% 求解绳索刚度与绳索张力之间的关系
syms l1 Fc;
Fc = K*(H-sqrt(l1^2-L2^2)-L40)*l1/(2*sqrt(l1^2-L2^2));
dfl1 = diff(Fc,l1);
syms x;
a = tension-K*(H-sqrt(x^2-L2^2)-L40)*x/(2*sqrt(x^2-L2^2));
L1_Result = solve(a,x);
for i=1:size(L1_Result,1)
    if double(L1_Result(i))>=L2
        if double(L1_Result(i))<=sqrt(L2^2+(H-L40)^2)
            L1 = double(L1_Result(i));
            break;
        end
    end
end
KCable = subs(dfl1,l1,L1);

% % 绳索张力与绳索长度的关系：2*Fc*sqrt(l1^2-L2^2)/l1 = K*(H-sqrt(l1^2-L2^2)-L40);
% % Kc = dFc/dl1
% % 不要用detaFc/detaL1
% % 求解绳索张力与L1(绳长)的关系
% interval_L = -0.01;
% L11 = 30.5;
% L10 = 39.06;
% length = round((L11-L10)/interval_L) + 1;
% F = zeros(4,length);
% L1 = L10:interval_L:L11;
% for i=1:4
%     K_spring = K + i*0.3;
%     for k=1:length
%         F(i,k) = K_spring*(H-sqrt(L1(k)^2-L2^2)-L40)*L1(k)/(2*sqrt(L1(k)^2-L2^2));
%     end
% end
% figure();
% plot(L1,F(1,:),L1,F(2,:),L1,F(3,:),L1,F(4,:),'LineWidth',1.5);
% legend('k_1=1N/mm','k_2=1.3N/mm','k_3=1.6N/mm','k_4=1.9N/mm');
% xlabel('L1(mm)');
% ylabel('Cable Tension(N)');
% 
% % 求解绳索刚度与L1(绳长的)的关系
% syms l1 Fc;
% interval_L = -0.01;
% L11 = 30.5;
% L10 = 39.06;
% length = round((L11-L10)/interval_L) + 1;
% L1 = L10:interval_L:L11;
% KC = zeros(4,length);
% for i=1:4
%     K_spring = K + i*0.3;
%     Fc = K_spring*(H-sqrt(l1^2-L2^2)-L40)*l1/(2*sqrt(l1^2-L2^2));
%     dfl1 = diff(Fc,l1);
%     for k=1:length
%         KC(i,k) = subs(dfl1,l1,L1(k));
%     end
% end
% figure();
% plot(L1,KC(1,:),L1,KC(2,:),L1,KC(3,:),L1,KC(4,:),'LineWidth',1.5);
% legend('k_1=1N/mm','k_2=1.3N/mm','k_3=1.6N/mm','k_4=1.9N/mm');
% xlabel('L1(mm)');
% ylabel('K(N/mm)');
% 
% % 求解绳索刚度与绳索张力之间的关系
% F_MAX = 70;
% interval_F = 5;
% F = 0:interval_F:F_MAX;
% length = F_MAX/interval_F + 1;
% KC = zeros(4,length);
% L1_Plot = zeros(4,length);
% for j=1:4
%     K_spring = K + j*0.3;
%     Fc = K_spring*(H-sqrt(l1^2-L2^2)-L40)*l1/(2*sqrt(l1^2-L2^2));
%     dfl1 = diff(Fc,l1);
%     for k=1:length
%         Tension = F(k);
%         syms x;
%         a = Tension-K_spring*(H-sqrt(x^2-L2^2)-L40)*x/(2*sqrt(x^2-L2^2));
%         L1_Result = solve(a,x);
%         for i=1:size(L1_Result,1)
%             if double(L1_Result(i))>=L2
%                 if double(L1_Result(i))<=sqrt(L2^2+(H-L40)^2)
%                     L1 = double(L1_Result(i));
%                     break;
%                 end
%             end
%         end
%         KC(j,k) = subs(dfl1,l1,L1);
%         L1_Plot(j,k) = L1;
%     end
% end
% figure();
% plot(F,KC(1,:),'k',F,KC(2,:),'g',F,KC(3,:),'b',F,KC(4,:),'r','LineWidth',1.5);
% legend('k_1=0.2N/mm','k_2=0.6N/mm','k_3=1N/mm','k_4=1.4N/mm');
% xlabel('Cable Tension(N)');
% ylabel('K(N/mm)');
% 
% figure();
% plot(F,L1_Plot(1,:),'k',F,L1_Plot(2,:),'g',F,L1_Plot(3,:),'b',F,L1_Plot(4,:),'r','LineWidth',1.5);
% legend('k_1=1N/mm','k_2=1.3N/mm','k_3=1.6N/mm','k_4=1.9N/mm');
% xlabel('Cable Tension(N)');
% ylabel('Cable Length(mm)');

end
