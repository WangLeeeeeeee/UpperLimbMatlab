%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% QPSO�㷨������ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

popsize = 30;           % Ⱥ���ģ
MAXITER = 2000;         % ����������
dimension = 20;         % ����ά��
irange_l = -100;        % λ�ó�ʼ���½�
irange_r = 100;         % λ�ó�ʼ���Ͻ�
xmax = 100;             % ������Χ�Ͻ�
xmin = 0;               % ������Χ�½�
M = (xmax-xmin)/2;      % ������Χ��ֵ
sum1 = 0;
st = 0;
runno = 50;             % �㷨����50��
data1 = zeros(runno,MAXITER);   % ��¼ÿһ����ÿһ�ε��������������Ӧֵ
gbest = zeros(1,dimension); % ��ʼ��ȫ�����λ�ñ���

for run=1:runno         % ����runno = 50�ֿ�ʼ
    T = cputime;        % ��¼cpuʱ��
    x = (irange_r - irange_l) * rand(popsize,dimension,1) + irange_l;   % ��ʼ�����ӵ�ǰλ��
    
    pbest = x;          % �����Ӹ������λ�ó�ʼ��Ϊ��ǰ���λ��
    gbest = zeros(1,dimension); % ��ʼ��ȫ�����λ�ñ���
    
    for i=1:popsize     % ���㵱ǰλ�ú͸������λ�õ���Ӧֵ
        f_x(i) = QPSOtest(x(i,:));
        f_pbest(i) = f_x(i);
    end
    
    g = find(f_pbest==min(f_pbest(1:popsize)), 1);    % �ҵ�ȫ�����λ�õ������±�
    gbest = pbest(g,:);     % �ҵ�ȫ�����λ��
    
    f_gbest = f_pbest(g);   % ��¼ȫ�����λ�õ���Ӧֵ
    
    MINIMUM = f_pbest(g);   % ��¼�㷨ÿ�ε����ҵ��������Ӧֵ
    
    for t=1:MAXITER
        
        alpha = (1.0 - 0.5)*(MAXITER - t)/MAXITER + 0.5; % ��������ϵ������
        
        mbest = sum(pbest)/popsize;     %����mbest
        
        for i=1:popsize
            fi1 = rand(1,dimension);    
            fi2 = rand(1,dimension);
            p = (2*fi1.*pbest(i,:)+2.1*fi2.*gbest)./(2*fi1+2.1*fi2);  % ���������P
            
            u = rand(1,dimension);
            b = alpha*abs(mbest - x(i,:));
            v = -log(u);
            x(i,:) = p + ((-1).^ceil(0.5+rand(1,dimension))).*b.*v; %����λ�õĸ���
            
            %%%%% ������λ��������������Χ�� %%%%%%
            z = x(i,:)-(xmax+xmin)/2;
            z = sign(z).*min(abs(z),M);
            x(i,:) = z+(xmax+xmin)/2;
            
            f_x(i) = QPSOtest(x(i,:));     % �������ӵ�ǰλ����Ӧֵ
            
            if(f_x(i)<f_pbest(i))   % �������Ӹ������λ��
                pbest(i,:) = x(i,:);
                f_pbest(i) = f_x(i);
            end
            
            if f_pbest(i)<f_gbest   % ��������ȫ�����λ��
                gbest = pbest(i,:);
                f_gbest = f_pbest(i);
            end
            
            MINIMUM = f_gbest;      % ��¼ȫ�����λ����Ӧֵ
        end
    MINIMUM;
        data1(run,t) = MINIMUM;
    end
    sum1 = sum1 + MINIMUM;
    
    time = cputime-T;       % �õ�һ�ּ����CPUʱ��
    st = st + time;
end
av = sum1/runno;        % ����runno�ֵõ��������Ӧֵ��ƽ��ֵ
st/50                   % ÿ��ʹ�õ�ƽ��CPUʱ��