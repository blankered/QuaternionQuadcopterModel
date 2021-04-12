%==========================================================================
%parameter file
%
%03/7/2017: Last modified by Hans Guentert
%==========================================================================

global Params Desired Integral prev;

Params.NextWaypoint = 1;

% Function Handles
Params.K        = eye(3); 
Params.dt       = 1/1000;
Params.E3       = [0 0 1]';
Params.Skew     = @(x)[0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
Params.Quat2Rot = @(q)eye(3) - 2*q(1)*Params.Skew(q(2:4)) + 2*Params.Skew(q(2:4))^2;
Params.Iota     = @(q)[-q(2:4); Params.Skew(q(2:4)) + q(1)*eye(3)];
Params.xdt.WT   = @(F,dF,dq_0,dq_1,dq_2,dq_3,dqe_0,dqe_1,dqe_2,dqe_3,m,q_0,q_1,q_2,q_3,qe_0,qe_1,qe_2,qe_3)reshape([(F.*qe_2.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_3.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m+(F.*dqe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(dF.*qe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_0.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_1.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_1.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_0.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*dqe_1.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_1.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_1.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,(F.*qe_2.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_0.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m+(F.*dqe_2.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(dF.*qe_2.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m-(F.*dqe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_3.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(dF.*qe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,-(F.*qe_0.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_2.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_2.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_2.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,(F.*qe_0.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*qe_3.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(F.*dqe_3.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_3.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,(F.*qe_0.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m-(F.*qe_3.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*dqe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(F.*dqe_3.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(dF.*qe_3.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*qe_2.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(F.*dqe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m],[3,3])


Integral.ErrorQuat        = [0;0;0];
Integral.ErrorPos         = [0;0;0];
Integral.ErrorPQR         = [0;0;0];
Desired.acceleration      = [0;0;0];
Desired.OmegaBody         = [0;0;0];
Desired.pqrddt            = [0;0;0];
Desired.XddtQuat          = [0;0;0;0];
Desired.pqrdt             = [0;0;0];
Desired.pqr               = [0;0;0];
Desired.vi                = [0;0;0];      

% Quadrotor Inertial Parameters
TotalMass       = 1.240;   ... kg
TotalLength     = 0.635;    ... m
TotalWidth      = 0.365;    ... m
RotorRadius     = 0.118;    ... m 
MassLessBattery = 0.986;   ... kg
BatteryMass     = TotalMass - MassLessBattery;

%       X
%     Width
% x-----------x
%       |
%       |  L
%       |  e
%       |  n  Y
%       |  g
%       |  t
%       |  h
%       |
% x-----------x

MassLength = (51 + (2*26) + (2*48) + (2*27) + (2*30) + 12)*10^-3;
MassWidth =  ((2*26) + (2*27) + (2*30) + 12)*10^-3;
MassCenter = TotalMass - MassLength - MassWidth;
Radius=TotalWidth/4;
Params.m = TotalMass;
Params.g = 9.81;% gravity (m/s^2)

Params.Jx = (2*MassCenter*Radius^2)/5 + 4*MassWidth*(TotalWidth/2)^2 ;
Params.Jy = (2*MassCenter*Radius^2)/5 + 2*MassLength*(TotalLength/2)^2;
Params.Jz = (2*MassCenter*Radius^2)/5 + 4*MassWidth*(TotalWidth/2)^2 + 2*MassLength*(TotalLength/2)^2;
       
Params.J = [Params.Jx 0 0; 0 Params.Jz 0; 0 0 Params.Jy];

Params.Thrust = Params.m*Params.g;
Params.AngularSpeed = 850;
Params.Thrustkp = 8;

%% States
% x = [px;...     % inertial frame north postion
%      py;...     % inertial frame east position
%      pz;...     % interial frame down position
%      ui;...     % inertial velocity north 
%      vi;...     % inertial velocity east
%      wi;...     % inertial velocity down
%      qw;...
%      qx;...
%      qy;...
%      qz;...
%      p;...
%      q;...
%      r];

 
 Params.x0=[0;...pn
            0;...pe
            0;...pd
            0;...ui
            0;...vi
            0;...wi
            1;...qw
            0;...qx
            0;...qy
            0;...qz
            0;...p
            0;...q
            0;...r
            0.56184;...ct1
            0.56184;...ct2 
            0.56184;...ct3
            0.56184]; %ct4


%% PID gains
a = 0.14;
b = 0.14;
c = 0.14;

Params.pos.kp = [a 0 0; 0 b 0; 0 0 c];
Params.pos.kd = [2*0.707*sqrt(Params.pos.kp(1,1)) 0 0;
                 0 2*0.707*sqrt(Params.pos.kp(2,2)) 0;
                 0 0 2*0.707*sqrt(Params.pos.kp(3,3))];

Params.pos.ki = [0  0  0; 0  0  0; 0  0  0];
                         
prev.feedacclereation     = [0; 0; 0];
prev.Desiredacceleration  = [0; 0; 0];

%                      QX           QY           QZ 
% Params.att.Quat.kp = [18.03   0 0; 0 18.03   0; 0 0 3.803];
% Params.att.Quat.ki = [0.4579  0 0; 0 0.1604  0; 0 0 0.03294];
% Params.att.Quat.kd = [3.21    0 0; 0 3.21    0; 0 0 0.233];

% % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [8.49   0 0;0 8.49  0; 0 0 8.49];
% Params.att.Quat.ki = [0.733  0 0;0 0.733 0; 0 0 0.733];
% Params.att.Quat.kd = [3.4    0 0;0 3.4   0; 0 0 3.37];




% % % %                   P             Q            R 
% Params.att.pqr.kp =  [22.84  0 0; 0 22.84 0; 0 0 82.49];
% Params.att.pqr.ki =  [13.22  0 0; 0 13.22 0; 0 0 47.72];
% Params.att.pqr.kd =  [9.868  0 0; 0 9.868 0; 0 0 35.64];

% % % % %                   P             Q            R 
% Params.att.pqr.kp =  [3.295   0 0; 0 3.295   0; 0 0 0.126];
% Params.att.pqr.ki =  [0.2106  0 0; 0 0.2106  0; 0 0 0.0011];
% Params.att.pqr.kd =  [3.907   0 0; 0 3.907   0; 0 0 3.61];

% % % TRIAL TUNING 1
% % % % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [2.567   0 0; 0 2.406 0; 0 0 1.53];
% Params.att.Quat.ki = [0.7404  0 0; 0 0.7404 0; 0 0 9.711];
% Params.att.Quat.kd = [0.001   0 0; 0 0.001 0; 0 0 0.04091];
% % % % %                   P             Q            R 
% Params.att.pqr.kp =  [0.035     0 0; 0 0.077    0; 0 0 0.126];
% Params.att.pqr.ki =  [0.000306  0 0; 0 0.00223  0; 0 0 0.0011];
% Params.att.pqr.kd =  [1         0 0; 0 0.666    0; 0 0 3.61];


% % TRIAL TUNING 2
% % % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [1.58   0 0; 0 2  0; 0 0 0];
% Params.att.Quat.ki = [2.8  0 0; 0 17 0; 0 0 0.816];
% Params.att.Quat.kd = [0   0 0; 0 0  0; 0 0 0];
% % % % %                   P             Q            R 
% Params.att.pqr.kp =  [0.228   0 0; 0 0.308   0; 0 0 0.825];
% Params.att.pqr.ki =  [0.0132  0 0; 0 0.0178  0; 0 0 0.00477];
% Params.att.pqr.kd =  [0.987   0 0; 0 1.33    0; 0 0 3.56];

% TRIAL TUNING 3
% % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [2.12   0 0; 0 2.12  0; 0 0 1.35];
% Params.att.Quat.ki = [0.701  0 0; 0 0.701 0; 0 0 0.81];
% Params.att.Quat.kd = [1.44   0 0; 0 1.44  0; 0 0 0];
% % % % %                   P             Q            R 
% Params.att.pqr.kp =  [0.228   0 0; 0 0.228   0; 0 0 1.11];
% Params.att.pqr.ki =  [0.0132  0 0; 0 0.0132  0; 0 0 0.0644];
% Params.att.pqr.kd =  [0.987   0 0; 0 0.987   0; 0 0 4.81];

% TRIAL TUNING 4
% % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [20.47   0 0; 0 20.47  0; 0 0 104.5];
% Params.att.Quat.ki = [0  0 0; 0 0 0; 0 0 48.79];
% Params.att.Quat.kd = [1.768   0 0; 0 1.768  0; 0 0 27.04];
% % % % %                   P             Q            R 
% Params.att.pqr.kp =  [1.75   0 0; 0 1.75 0; 0 0 0.709];
% Params.att.pqr.ki =  [0     0 0; 0 0     0; 0 0 0];
% Params.att.pqr.kd =  [9.998 0 0; 0 9.998 0; 0 0 14.05];

% TRIAL TUNING 5
% % % % %                       QX           QY           QZ 
% Params.att.Quat.kp = [2.12   0 0; 0 2.12  0; 0 0 1.35];
% Params.att.Quat.ki = [0.701  0 0; 0 0.701 0; 0 0 0.81];
% Params.att.Quat.kd = [1.44   0 0; 0 1.44  0; 0 0 0];
% % % % % %                   P             Q            R 
% Params.att.pqr.kp =  [0.228   0 0; 0 0.228   0; 0 0 1.11];
% Params.att.pqr.ki =  [0.0132  0 0; 0 0.0132  0; 0 0 0.0644];
% Params.att.pqr.kd =  [0.987   0 0; 0 0.987   0; 0 0 4.81];

% TRIAL TUNING 6
% % % % %                       QX           QY           QZ 
Params.att.Quat.kp = [9      0 0; 0 9     0; 0 0 1.35];
Params.att.Quat.ki = [4.6    0 0; 0 0.98  0; 0 0 0.81];
Params.att.Quat.kd = [4.39   0 0; 0 4.3  0; 0 0 0];
% % % %                   P             Q            R 
Params.att.pqr.kp =  [29     0 0; 0 29     0; 0 0 54];
Params.att.pqr.ki =  [21.86  0 0; 0 21.86  0; 0 0 31];
Params.att.pqr.kd =  [9.78   0 0; 0 9.78   0; 0 0 23];
 
%% Variable Pitch
Params.R = RotorRadius;  %meters
Params.d = sqrt((TotalLength/2)^2 + (TotalWidth/2)^2); %meters
Params.density = 0.0123;
Params.Vtip = Params.AngularSpeed*Params.R;
Params.K = Params.density*pi*Params.R^2*Params.Vtip^2;                       
Params.c = 0.1;

AirDensity = 1.225; % kg/m3 
maxThick = 2.95; % mm
ChordLength = 28.21; % mm
Nb = 8; % Number of Blades
Reynolds = 80000;
RPM = 7000;
Vtip = (RPM/60)*RotorRadius;

Params.BladeThrust = @(CT)(CT*AirDensity*pi()*((RotorRadius/1000)^2)*(Vtip^2));

NACA = '0010';

solidity = Nb * ChordLength / (pi()*RotorRadius);
Params.BladeAngle = @(CT) (6*CT)/(solidity*Params.CLalpha) + (3/2)*sqrt(CT/2);

CL = 2*Params.g*Params.m/(AirDensity*(Vtip^2)*Nb*ChordLength*RotorRadius)

Params.theta = @(CT) ((3*CT/Nb*ChordLength*RotorRadius) + 0.75*sqrt(CT))

%% LQR Parameters
A = [zeros(3)   eye(3)     zeros(3,1);
     zeros(3)   zeros(3)   zeros(3,1);
     zeros(1,3) zeros(1,3) zeros(1)];

B = [zeros(3)   zeros(3,1);
     eye(3)     zeros(3,1);
     zeros(1,3) 1];

Z = 8;
U = 12;
Q = (1/(max(Z))^2) * diag([1, 1, 1, 1, 1, 1, 1]);
R = (1/(max(U))^2) * diag([1, 1, 1, 1]);

%X'*Q*X + U'*R*U
 
%Params.K_gain = lqr(A,B,Q,R);
