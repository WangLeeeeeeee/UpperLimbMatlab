%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% QPSO算法主程序 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

popsize = 30;           % 群体规模
MAXITER = 2000;         % 最大迭代次数
dimension = 20;         % 问题维数
irange_l = -100;        % 位置初始化下界
irange_r = 100;         % 位置初始化上界
xmax = 100;             % 搜索范围上界
xmin = 0;               % 搜索范围下界
M = (xmax-xmin)/2;      % 搜索范围中值
sum1 = 0;
st = 0;
runno = 50;             % 算法运行50轮
data1 = zeros(runno,MAXITER);   % 记录每一轮中每一次迭代步数的最好适应值
gbest = zeros(1,dimension); % 初始化全局最好位置变量

for run=1:runno         % 计算runno = 50轮开始
    T = cputime;        % 记录cpu时间
    x = (irange_r - irange_l) * rand(popsize,dimension,1) + irange_l;   % 初始化粒子当前位置
    
    pbest = x;          % 将粒子个体最好位置初始化为当前最好位置
    gbest = zeros(1,dimension); % 初始化全局最好位置变量
    
    for i=1:popsize     % 计算当前位置和个体最好位置的适应值
        f_x(i) = QPSOtest(x(i,:));
        f_pbest(i) = f_x(i);
    end
    
    g = find(f_pbest==min(f_pbest(1:popsize)), 1);    % 找到全局最好位置的粒子下标
    gbest = pbest(g,:);     % 找到全局最好位置
    
    f_gbest = f_pbest(g);   % 记录全局最好位置的适应值
    
    MINIMUM = f_pbest(g);   % 记录算法每次迭代找到的最好适应值
    
    for t=1:MAXITER
        
        alpha = (1.0 - 0.5)*(MAXITER - t)/MAXITER + 0.5; % 搜索扩张系数计算
        
        mbest = sum(pbest)/popsize;     %计算mbest
        
        for i=1:popsize
            fi1 = rand(1,dimension);    
            fi2 = rand(1,dimension);
            p = (2*fi1.*pbest(i,:)+2.1*fi2.*gbest)./(2*fi1+2.1*fi2);  % 计算随机点P
            
            u = rand(1,dimension);
            b = alpha*abs(mbest - x(i,:));
            v = -log(u);
            x(i,:) = p + ((-1).^ceil(0.5+rand(1,dimension))).*b.*v; %粒子位置的更新
            
            %%%%% 将粒子位置限制在搜索范围内 %%%%%%
            z = x(i,:)-(xmax+xmin)/2;
            z = sign(z).*min(abs(z),M);
            x(i,:) = z+(xmax+xmin)/2;
            
            f_x(i) = QPSOtest(x(i,:));     % 计算粒子当前位置适应值
            
            if(f_x(i)<f_pbest(i))   % 更新粒子个体最好位置
                pbest(i,:) = x(i,:);
                f_pbest(i) = f_x(i);
            end
            
            if f_pbest(i)<f_gbest   % 更新粒子全局最好位置
                gbest = pbest(i,:);
                f_gbest = f_pbest(i);
            end
            
            MINIMUM = f_gbest;      % 记录全局最好位置适应值
        end
    MINIMUM;
        data1(run,t) = MINIMUM;
    end
    sum1 = sum1 + MINIMUM;
    
    time = cputime-T;       % 得到一轮计算的CPU时间
    st = st + time;
end
av = sum1/runno;        % 计算runno轮得到的最好适应值的平均值
st/50                   % 每轮使用的平均CPU时间