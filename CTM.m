function [sys,x0,str,ts] = CTM(t,x,u,flag)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.


switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
%   case 1,
%     sys=mdlDerivatives(t,x,u);
  case 2,
    sys=[];    
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=[];
  case 9,
    sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
% sizes.NumContStates  = 2;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 20;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;   % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];
% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
% function sys=mdlDerivatives(t,x,u,A,B,C,D)
% sys = A*x + B*u;

% end mdlDerivatives


%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
q1_d = u(1); dq1_d = u(2); ddq1_d = u(3);
q2_d = u(4); dq2_d = u(5); ddq2_d = u(6);
q3_d = u(7); dq3_d = u(8); ddq3_d = u(9);
q4_d = u(10); dq4_d = u(11); ddq4_d = u(12);
q1 = u(13); dq1 = u(14);
q2 = u(15); dq2 = u(16);
q3 = u(17); dq3 = u(18);
q4 = u(19); dq4 = u(20);

q_d = [q1_d;q2_d;q3_d;q4_d];
dq_d = [dq1_d;dq2_d;dq3_d;dq4_d];
ddq_d = [ddq1_d;ddq2_d;ddq3_d;ddq4_d];

q = [q1;q2;q3;q4];
dq = [dq1;dq2;dq3;dq4];

e = q - q_d;
de = dq - dq_d;

% �趨Kp��Kv����
Kp = [150,0,0,0;0 150 0 0;0 0 150 0;0 0 0 150];
Kv = [10 0 0 0;0 10 0 0;0 0 10 0;0 0 0 10];

% ���㶯��ѧ����D,H,C
% �������ն���ѧ���̣�tau = D(q)*beta + h(q,omega) + C(q)
% �ο���http://www.cse.zju.edu.cn/eclass/attachments/2016-11/01-1479736065-215680.pdf
% ���˱任����
% ��֫�˶�ѧ����
global LenUpperarm
global LenForeArm
global mShoulder 
global mElbow
% ������С
g = [0,0,9.81,0]; 
T01 = DH(0,0,0,q(1));
T12 = DH(0,-pi/2,0,q(2)-pi/2);
T23 = DH(0,pi/2,0,q(3));
T34 = DH(LenUpperarm,-pi/2,0,q(4));
T45 = DH(LenForeArm,0,0,0);
T02 = T01*T12;T03 = T02*T23;
T04 = T03*T34;T13 = T12*T23;
T14 = T13*T34;
% ���������ű任����ע���±�
T = zeros(4,4,5,5);
T(:,:,1,2) = T01;T(:,:,2,3) = T12;T(:,:,3,4) = T23;
T(:,:,4,5) = T34;T(:,:,1,3) = T02;T(:,:,1,4) = T03;
T(:,:,1,5) = T04;T(:,:,2,4) = T13;T(:,:,2,5) = T14;
T(:,:,1,1) = eye(4);T(:,:,2,2) = eye(4);T(:,:,3,3) = eye(4);
T(:,:,4,4) = eye(4);T(:,:,5,5) = eye(4);

weight = zeros(4,1);
weight(1) = 0;
weight(2) = 0;
weight(3) = mShoulder;
weight(4) = mElbow;
I = zeros(4,4,4);
I(:,:,1) = zeros(4,4);
I(:,:,2) = zeros(4,4);
I(:,:,3) = [0.0034 0 0 0.0759; 0 0.0005 0 0; 0 0 0.0005 0; 0.0759 0 0 0.5];
I(:,:,4) = [0.0016 0 0 0.0405; 0 0.0002 0 0; 0 0 0.0002 0; 0.0405 0 0 0.3];
% ����D����
% dT/dq = T*Q(ע��Ͳο���վ�в�ͬ)
Q = [0 -1 0 0; 1 0 0 0; 0 0 0 0; 0 0 0 0];
D = zeros(4,4);
U = zeros(4,4,4,4);
for i = 1:4
    for k = 1:4
        for j=max([i,k]):4
            U(:,:,j,k) = T(:,:,1,k+1)*Q*T(:,:,k+1,j+1);
            U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
            D(i,k) = D(i,k) + trace(U(:,:,j,k)*I(:,:,j)*U(:,:,j,i)');
        end
    end
end
% ����H����
H = zeros(4,1);
h = 0;
for i=1:4
    for k=1:4
        for m=1:4
            for j=max([i,k,m]):4
                if j>=m && m>=k
                    U_dev = T(:,:,1,k+1)*Q*T(:,:,k+1,m+1)*Q*T(:,:,m+1,j+1);
                else
                    U_dev = T(:,:,1,m+1)*Q*T(:,:,m+1,k+1)*Q*T(:,:,k+1,j+1);
                end
                U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
                h = h + trace(U_dev*I(:,:,j)*U(:,:,j,i));
            end
            H(i) = H(i) + h*dq(k)*dq(m);
        end
    end
end
% ����C����
C = zeros(4,1);
r = zeros(4,4);
r(:,1) = [0;0;0;1];
r(:,2) = [0;0;0;1];
r(:,3) = [LenUpperarm/2;0;0;1];
r(:,4) = [LenForeArm/2;0;0;1];
for i = 1:4
    for j=i:4
        U(:,:,j,i) = T(:,:,1,i+1)*Q*T(:,:,i+1,j+1);
        C(i) = C(i) - weight(j)*g*U(:,:,j,i)*r(:,j);
    end
end
% �����ⲿ���������ڹؽ��ϲ����Ĺؽ�����
% ���ݾ���ѧ��ʽ��load_tau = Jacobi'*F
T(:,:,1) = T01;
T(:,:,2) = T12;
T(:,:,3) = T23;
T(:,:,4) = T34;
T(:,:,5) = T45;
Jacobi = Jacobi_dif(T);
Jp = pinv(Jacobi);
mload = 1.5;
load_F = [mload*g(1:3)';0;0;0];
load_J = Jp*load_F;
load_J(4) = -load_J(4);

tau = D*(ddq_d-Kp*e-Kv*de) + C + H + load_J;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = tau(3);
sys(4) = tau(4);
sys(5) = e(1);
sys(6) = e(2);
sys(7) = e(3);
sys(8) = e(4);
sys(9) = de(1);
sys(10) = de(2);
sys(11) = de(3);
sys(12) = de(4);

% end mdlOutputs

