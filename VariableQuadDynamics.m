function [sys,x0,str,ts,simStateCompliance] = VariableQuadDynamics(t,x,u,flag,Params)
%==========================================================================
%This is the 12-state quadrotor dynamic model s-function code given to ECE 6320 Students
%
%
%01/16/2015: Last modified by Hans Guentert
%==========================================================================

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Params);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,Params);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,Params);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Params)

sizes = simsizes;

sizes.NumContStates  = 17;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 17;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  =Params.x0;

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,Params)
% Inputs
T_Desired = u(1);
pddt      = u(2);
qddt      = u(3);
rddt      = u(4);

% Position
px = x(1);
py = x(2);
pz = x(3);
% Velocity
ui = x(4); 
vi = x(5);
wi = x(6);
% Quaternion Pose
qw = x(7);
qx= x(8);
qy= x(9);
qz = x(10);
% Angular Velocity
p = x(11);
q = x(12);
r = x(13);
% Thrust Coefficient
CT1 = x(14);
CT2 = x(15);
CT3 = x(16);
CT4 = x(17);

T = Params.K*(CT1 + CT2 + CT3 + CT4);
l = Params.K*Params.d*(CT1 - CT2 - CT3 + CT4);
m = Params.K*Params.d*(CT1 + CT2 - CT3 - CT4);
n = (Params.K*Params.R/sqrt(2))*(CT1^3/2 - (CT2^3/2) + CT3^3/2 - (CT4^3/2));

Th = [Params.K                          Params.K                           Params.K                           Params.K;...
      Params.K*Params.d                 -Params.K*Params.d                 -Params.K*Params.d                 Params.K*Params.d;...
      Params.K*Params.d                 Params.K*Params.d                  -Params.K*Params.d                -Params.K*Params.d;...
      Params.K*Params.R*sqrt(CT1/2) -Params.K*Params.R*sqrt(CT2/2) Params.K*Params.R*sqrt(CT3/2) -Params.K*Params.R*sqrt(CT4/2)];

  
moment.body                = [l; m; n];
force.body                 = [0; 0; T];
%Velocity.body              = [ub; vb; wb];
quat.scalar                = qw;
quat.vector                = [qx; qy; qz];
quat.full                  = [quat.scalar; quat.vector];
omega.body                 = [p; q; r];                     
gravity.inertial           = [0; 0; 9.81];
                                                       

xdot=zeros(17,1);
%[0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
% Velocity.inertial =  quatmultiply(quatmultiply(quatconj(quat.full'),[0 Velocity.body']),quat.full');
% xdot(1:3) = Velocity.inertial(2:4);
xdot(1:3) = [ui vi wi];

% [0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
Accleration.inertial =  (1/Params.m)*quatmultiply(quatmultiply(quatconj(transpose(quat.full)),[0 transpose(force.body)]),transpose(quat.full)) - [0 gravity.inertial.'];
xdot(4:6) = Accleration.inertial(2:4)';

% Inertial-Frame time derivative of q
%xdt.q = 0.5*quatmultiply(quat.full',[0 omega.body']);
xdt.q = 0.5*Params.Iota(transpose(quat.full))*omega.body;
xdot(7:10) =  xdt.q;

% Omega Body
xdt.omegabody = [((Params.Jy-Params.Jz)/Params.Jx)*q*r;
                 ((Params.Jz-Params.Jx)/Params.Jy)*p*r;
                 ((Params.Jx-Params.Jy)/Params.Jz)*p*q] + [(1/Params.Jx)*l; (1/Params.Jy)*m; (1/Params.Jz)*n];
     
xdot(11:13) = xdt.omegabody;

Tdot = Params.Thrustkp*(T_Desired - T);
ldot = Params.Jx*pddt - (Params.Jy - Params.Jx)*(q*xdot(13) + r*xdot(12));
mdot = Params.Jy*qddt - (Params.Jz - Params.Jx)*(p*xdot(13) + r*xdot(11));
ndot = Params.Jz*rddt - (Params.Jx - Params.Jy)*(p*xdot(12) + q*xdot(11));

xdot(14:17) = (Th^-1)*[Tdot; ldot; mdot; ndot];

sys = real(xdot);

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%===============tn15565 nano tech==============================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,Params)
% All the states are send as output.
sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Paramserform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
