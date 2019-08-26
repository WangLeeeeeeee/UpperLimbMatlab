function [sys,x0,str,ts] = Stiffness_simulink(t,x,u,flag)
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

%
% The following outlines the general structure of an S-function.
%

switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
%  case 1,
   % sys=mdlDerivatives(t,x,u);
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=[];
  case 9,
    sys=[];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

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
sizes.NumOutputs     = 8; % 6根绳索张力 肩关节和肘关节刚度
sizes.NumInputs      = 8; % 4个关节角 4个关节力矩
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
q1 = u(1);
q2 = u(2);
q3 = u(3);
q4 = u(4);
tau1 = u(5);
tau2 = u(6);
tau3 = u(7);
tau4 = u(8);

q = [q1;q2;q3;q4];
tau = [tau1;tau2;tau3;tau4];

[JF_SH,JF_EL] = TensiontoTorque(q);
Tension = zeros(1,6);

%% 二次规划求解绳索张力
% 肘关节
ElbowThreshold = 2;
H2 = eye(2); f2= zeros(2,1); Aeq_elbow = -JF_EL; beq_elbow  = [0;0;tau(4)] ; lb2 = ElbowThreshold * ones(2,1); ub2 = 100*ones(2,1); x02 = 5 * ones(2,1);
[TT_elbow,fval,exitflag] = quadprog(H2,f2,[],[],Aeq_elbow ,beq_elbow ,lb2,ub2,x02);
Tension(5)=TT_elbow(1);
Tension(6)=TT_elbow(2);
% 肩关节
ShoulderThreshold = 2;
TorReq_GH=[tau(1);tau(2);tau(3)];
H1 = eye(4); f1= zeros(4,1); Aeq_GH = -JF_SH; beq_GH = TorReq_GH; lb1 = ShoulderThreshold * ones(4,1); ub1 = 100*ones(4,1); x01 = 5 * ones(4,1);
[TT_GH,fval,exitflag] = quadprog(H1,f1,[],[],Aeq_GH,beq_GH,lb1,ub1,x01);
Tension(1)=TT_GH(1);
Tension(2)=TT_GH(2);
Tension(3)=TT_GH(3);
Tension(4)=TT_GH(4);
% 求解关节刚度
KCable_1 = nsu(Tension(1));
KCable_2 = nsu(Tension(2));
KCable_3 = nsu(Tension(3));
KCable_4 = nsu(Tension(4));
KCable_5 = nsu(Tension(5));
KCable_6 = nsu(Tension(6));   
KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
KCable_el = diag([KCable_5,KCable_6]);
K_SH = JF_SH*KCable_sh*(JF_SH)';
K_SH_Value = trace(K_SH);
K_ELBOW_Value = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';

sys(1) = Tension(1);
sys(2) = Tension(2);
sys(3) = Tension(3);
sys(4) = Tension(4);
sys(5) = Tension(5);
sys(6) = Tension(6);
sys(7) = K_SH_Value;
sys(8) = K_ELBOW_Value;
% end mdlOutputs

