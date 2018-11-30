% 遍历工作空间
% 从起始关节角qStart到终止关节角qEnd
function traverseWorkSpace(qStart,qEnd)
% 上肢运动学参数
global LenUpperarm
global LenForeArm
the1_start = qStart(1);
the1_end = qEnd(1);
the2_start = qStart(2);
the2_end = qEnd(2);
the3_start = qStart(3);
the3_end = qEnd(3);
the4_start = qStart(4);
the4_end = qEnd(4);
step = 0.05;
num = ((the1_end-the1_start)/step+1)*((the2_end-the2_start)/step+1)*((the3_end-the3_start)/step+1)*((the4_end-the4_start)/step+1);
space = zeros(3,ceil(num));
i = 0;
for the1_sp = the1_start:step:the1_end
    for the2_sp = the2_start:step:the2_end
        for the3_sp = the3_start:step:the3_end
            for the4_sp = the4_start:step:the4_end
                i = i + 1;
                T01 = DH(0,0,0,the1_sp);
                T12 = DH(0,-pi/2,0,the2_sp-pi/2);
                T23 = DH(0,pi/2,0,the3_sp);
                T34 = DH(LenUpperarm,-pi/2,0,the4_sp);
                T45 = DH(LenForeArm,0,0,0);
                T05 = T01*T12*T23*T34*T45;
                space(1,i) = T05(1,4);
                space(2,i) = T05(2,4);
                space(3,i) = T05(3,4);
            end
        end
    end
end
x = space(1,:);
y = space(2,:);
z = space(3,:);
figure();
plot3(x,y,z,'*');
%scatter3(space(1,:), space(2,:), space(3,:),'b','filled');
title('末端工作空间');
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
grid on;
hold on;
% 将散点图转化为曲面图 参考：https://blog.csdn.net/u012302488/article/details/51201238
[X,Y] = meshgrid(min(x):1:max(x),min(y):1:max(y));
Z= griddata(x,y,z,X,Y);
%绘制曲面
% figure();
% mesh(X,Y,Z);
% hold on;
figure();
surf(X,Y,Z);
shading interp;
colormap(jet);
hold on;
PlotUpperLimb([0,0,0,0]);
end
