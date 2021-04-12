clc; clear all;

run parameters;

syms px py pz ui vi wi qw qx qy qz p q r CT1 CT2 CT3 CT4 Tdot T
global Params Desired Integral prev;

%% States
x = [px;...     % inertial frame north postion
     py;...     % inertial frame east position
     pz;...     % interial frame down position
     ui;...     % inertial velocity north 
     vi;...     % inertial velocity east
     wi;...     % inertial velocity down
     qw;...
     qx;...
     qy;...
     qz;...
     p;...
     q;...
     r;...
     CT1;...    %Thrust Coefficient
     CT2;...    %Thrust Coefficient
     CT3;...    %Thrust Coefficient
     CT4];...    %Thrust Coefficient];

T = Params.K*(CT1 + CT2 + CT3 + CT4);
l = Params.K*Params.d*(CT1 - CT2 - CT3 + CT4);
m = Params.K*Params.d*(CT1 + CT2 - CT3 - CT4);
n = (Params.K*Params.R/sqrt(2))*(CT1^3/2 - (CT2^3/2) + CT3^3/2 - (CT4^3/2));

Th = [Params.K                                 Params.K                                 Params.K                                 Params.K;...
      Params.K*Params.d                       -Params.K*Params.d                       -Params.K*Params.d                        Params.K*Params.d;...
      Params.K*Params.d                        Params.K*Params.d                       -Params.K*Params.d                       -Params.K*Params.d;...
      1.5*Params.K*Params.R*sqrt(CT1/2) -1.5*Params.K*Params.R*sqrt(CT2/2)  1.5*Params.K*Params.R*sqrt(CT3/2) -1.5*Params.K*Params.R*sqrt(CT4/2)];

pqrdt = [((Params.Jy-Params.Jz)/Params.Jx)*q*r;
         ((Params.Jz-Params.Jx)/Params.Jy)*p*r;
         ((Params.Jx-Params.Jy)/Params.Jz)*p*q] + [(1/Params.Jx)*l; (1/Params.Jy)*m; (1/Params.Jz)*n];
     
% Rotation of Velocity from Inertial Frame to Body FrameParams.
velocity.interial          = [ui; vi; wi];
quat.scalar                =  qw;
quat.vector                = [qx; qy; qz];
quat.full                  = [quat.scalar; quat.vector];
omega.body                 = [p; q; r];  

% Find Desired acceleration from position error
% Error.proportional   = [Desired.x;     Desired.y;     Desired.z]     - [px; py; pz];
% Error.dot            = [Desired.vi(1); Desired.vi(2); Desired.vi(3)] - [ui; vi; wi];
%Integral.ErrorPos     = Integral.ErrorPos + Error.proportional*Params.dt

%feedback.acceleration = Params.pos.kp*Error.proportional + Params.pos.ki*Integral.ErrorPos + Params.pos.kd*Error.dot;
Desired.yaw = 0;
Psidt = 0;

syms xddt yddt zddt s
u = [xddt; yddt; zddt];
feedback.acceleration = u;

velocity.error       = - [ui; vi; wi];
velocity.xdt.error   = feedback.acceleration;

% Equation 3.9 - Page 57 - Mark Cutler
force.Inertial       = Params.m * (feedback.acceleration + Desired.acceleration + [0; 0; Params.g]);

% Equation 12  - Page 57 - Mark Culter
ForceBar.Inertial    = force.Inertial/norm(force.Inertial);

% Mark Culter: Page 58 
force.total          = norm(force.Inertial);

% Mark Culter: Page 55
force.body           = [0; 0; force.total];
ForceBar.body        = force.body/norm(force.body);

% Calculate the Desired orientation
Desired.QuatNoYaw = (1/sqrt(2*(1 + transpose(ForceBar.Inertial) * ForceBar.body))) * [ 1 + transpose(ForceBar.Inertial) * ForceBar.body; cross(ForceBar.Inertial, ForceBar.body)];  
Desired.quat      = quatmultiply(transpose(Desired.QuatNoYaw),[cos(Desired.yaw/2) 0 0 sin(Desired.yaw/2)]);

feedback.jerk = (feedback.acceleration - prev.feedacclereation)/Params.dt;
Desired.jerk  = (Desired.acceleration  - prev.Desiredacceleration)/Params.dt;

prev.feedacclereation     = feedback.acceleration;
prev.Desiredacceleration  = Desired.acceleration;

xdt.ForceInertial    = Params.m*(feedback.jerk + Desired.jerk);
xdt.ForceBarInertial = xdt.ForceInertial/norm(force.Inertial) - (force.Inertial*(force.Inertial' * xdt.ForceInertial)/(norm(force.Inertial)^3));

% Calculate the Desired attitude rate
Desired.pqr    = cross(ForceBar.Inertial,xdt.ForceBarInertial);
Desired.pqr(3) = Psidt;

xdt.ForceBarInterialxy = cross(Desired.OmegaBody,ForceBar.Inertial);
xdt.ForceBarInterialz  = Psidt;

%% Desired vehicle orientation in the Inertial-frame
quat.inertial  = quat.full';
quat.error     = quatmultiply(quat.inertial,quatconj(Desired.quat));

%Calulate Desired Angular Acceleration in the Body Frame
%H  = sign(quat.error(1))
%WT = (force.total/Params.m)*(quat.error(1)*eye(3) - Params.Skew(quat.error(2:4)))*Params.Skew(Params.Quat2Rot(quat.inertial)*Params.E3)
     
%Desired.pqr = -Params.K3 * H * transpose(quat.error(2:4)) - transpose(WT) * velocity.error - Params.Quat2Rot(quat.error) * omega.body  
%Desired.pqr = -H * Params.att.Quat.kp * transpose(quat.error(2:4)) - transpose(WT) * velocity.error - Params.att.Quat.kd*(Params.Quat2Rot(quat.error)*omega.body) 

omega.error       = omega.body - Desired.pqr;
quat.xdt.inertial = 0.5*Params.Iota(quat.inertial)*omega.body;
quat.xdt.error    = 0.5*Params.Iota(quat.error)*omega.error;

% quat.xddt =  H * Params.att.Quat.kp * quat.error(2:4) +  Params.att.Quat.kd * quat.xdt.error 
% Desired.pqrdt = [-q(2) q(1) q(4) -q(3);
%                  -q(3) -q(4) q(1) q(2);
%                  -q(4) q(3) -q(2) q(1)] * quat.xddt

%Integral.ErrorQuat = Integral.ErrorQuat + (quat.error(2:4).')*Params.dt;

Desired.pqrdt =  -sign(quat.error(1)) * Params.att.Quat.kp * quat.error(2:4).' -  Params.att.Quat.kd * quat.xdt.error(2:4) - Params.att.Quat.ki*Integral.ErrorQuat;
%% Integrate to get feedforward jerk
%moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - Desired.OmegaBody);
Error.prop        = [Desired.pqr(1)   - p;
                     Desired.pqr(2)   - q;
                     Desired.pqr(3)   - r];
                 
Error.dot         = [Desired.pqrdt(1) - pqrdt(1);
                     Desired.pqrdt(2) - pqrdt(2);
                     Desired.pqrdt(3) - pqrdt(3)];
                 
Integral.ErrorPQR = Integral.ErrorPQR + Error.prop*Params.dt;
pqrddt = Desired.pqrddt + Params.att.pqr.kp * Error.prop + Params.att.pqr.ki * Integral.ErrorPQR + Params.att.pqr.kd * Error.dot;


Desired.Thrust = force.total;
output = [Desired.Thrust, pqrddt(1), pqrddt(2), pqrddt(3)];


%% Dynamics
moment.body = output(2:4);

%[0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
xdot(1:3) = [ui; vi; wi];

%[0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
gravity.inertial           = [0; 0; 9.81];
force.body            = [0; 0; T];
Accleration.inertial  = (1/Params.m)*quatmultiply(quatmultiply(quatconj(quat.full'),[0 force.body']),quat.full')' - [0; gravity.inertial];
xdot(4:6)             = Accleration.inertial(2:4);

xdt.q                 = 0.5*quatmultiply(quat.full',[0 omega.body']);
xdot(7:10)            = xdt.q;

xdt.omegabody         = Params.J^-1 *[moment.body.' - cross(omega.body,Params.J*omega.body)];
xdot(11:13)           = xdt.omegabody;

Tdot = Params.Thrustkp*(output(1) - T);
ldot = Params.Jx*output(2) - (Params.Jy - Params.Jx)*(q*xdot(13) + r*xdot(12));
mdot = Params.Jy*output(3) - (Params.Jz - Params.Jx)*(p*xdot(13) + r*xdot(11));
ndot = Params.Jz*output(4) - (Params.Jx - Params.Jy)*(p*xdot(12) + q*xdot(11));

xdot(14:17) = (Th^-1)*[Tdot; ldot; mdot; ndot];


% l1 = [px; py; pz; phi; theta; psi; ub; vb; wb; p; q; r; CT1;  CT2;  CT3;  CT4; Tdot; pddt; qddt; rddt]
% l2 = [0   0   0   0    0      0    0   0   0   0  0  0  0.348 0.348 0.348 0.348 0 0 0 0]';
 
A  = jacobian(xdot,x)
B  = jacobian(xdot,u)

l1 = [px py pz ui vi wi qw qx qy qz p q r CT1   CT2   CT3   CT4   Tdot xddt yddt zddt];
l2 = [0  0  0  0  0  0  0  0  0  0  0 0 0 0.348 0.348 0.348 0.348 0    0    0    0   ];

A1 = subs(A,l1,l2)
B1 = subs(B,l1,l2)

C = eye(size(A1,1))

D = zeros(17,3)

tf1 = C*(s*eye(size(A1,1)) - A1)^-1*B1


%% Tune phi
figure(1)
phitf = tf1(11,1)
[numexpr, denexpr] = numden(phitf);

%// Extract numerator coefficients
[numcoef, numpow] = coeffs(expand(numexpr), s);
num = rot90(sym(sym2poly(sum(numpow))), 2);
num(num ~= 0) = coeffs(expand(numexpr), s);
phinum = num

%// Extract denominator coefficients
[dencoef, denpow] = coeffs(expand(denexpr), s);
den = rot90(sym(sym2poly(sum(denpow))), 2);
den(den ~= 0) = coeffs(expand(denexpr), s);
phidem = flip(den)

sysPhi = tf(double(phinum), [double(phidem)])
opts = pidtuneOptions('DesignFocus','reference-tracking');
[Cphi,info] = pidtune(sysPhi,'PID',opts);
Tphi = feedback(sysPhi*Cphi,1);
Gphi = feedback(sysPhi,Cphi);
stepplot(Tphi)
%stepplot(Gphi)
Cphi
%% Tune Theta
figure(2)
thetatf = tf1(12,2)

[numexpr, denexpr] = numden(thetatf);

%// Extract numerator coefficients
[numcoef, numpow] = coeffs(expand(numexpr), s);
num = rot90(sym(sym2poly(sum(numpow))), 2);
num(num ~= 0) = coeffs(expand(numexpr), s);
thetanum = num

%// Extract denominator coefficients
[dencoef, denpow] = coeffs(expand(denexpr), s);
den = rot90(sym(sym2poly(sum(denpow))), 2);
den(den ~= 0) = coeffs(expand(denexpr), s);
thetadem = flip(den)

sysTheta = tf(double(thetanum), [double(thetadem)])
opts = pidtuneOptions('DesignFocus','reference-tracking');
[CTheta,info] = pidtune(sysTheta,'PID',opts);
TTheta = feedback(sysTheta*CTheta,1);
GTheta = feedback(sysTheta,CTheta);
stepplot(TTheta)
%stepplot(GTheta)
CTheta

%% Tune psi
figure(3)
psitf = tf1(13,3)

[numexpr, denexpr] = numden(psitf);

%// Extract numerator coefficients
[numcoef, numpow] = coeffs(expand(numexpr), s);
num = rot90(sym(sym2poly(sum(numpow))), 2);
num(num ~= 0) = coeffs(expand(numexpr), s);
psinum = num

%// Extract denominator coefficients
[dencoef, denpow] = coeffs(expand(denexpr), s);
den = rot90(sym(sym2poly(sum(denpow))), 2);
den(den ~= 0) = coeffs(expand(denexpr), s);
psidem = flip(den)
sysPsi = tf(double(psinum), [double(psidem)])
opts = pidtuneOptions('DesignFocus','reference-tracking');
[Cpsi,info] = pidtune(sysPsi,'PID',opts);
Tpsi = feedback(sysPsi*Cpsi,1);
Gpsi = feedback(sysPsi,Cpsi);
stepplot(Tpsi)
%stepplot(Gpsi)
Cpsi
