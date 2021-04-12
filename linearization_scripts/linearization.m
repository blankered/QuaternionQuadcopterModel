clc; clear all;

syms px py pz ub vb wb qw qx qy qz p q r

Params.dt = 0.1;

% Quadrotor Inertial Parameters
TotalMass       = 1.240;   ... kg
TotalLength     = .635;    ... m
TotalWidth      = .365;    ... m
RotorRadius     = .118;    ... m 
MassLessBattery = 0.986;   ... kg
BatteryMass = TotalMass - MassLessBattery;

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
Params.g = 9.81          ;% gravity (m/s^2)

Params.Jx = (2*MassCenter*Radius^2)/5 + 4*MassWidth*(TotalWidth/2)^2 ;
Params.Jy = (2*MassCenter*Radius^2)/5 + 2*MassLength*(TotalLength/2)^2;
Params.Jz = (2*MassCenter*Radius^2)/5 + 4*MassWidth*(TotalWidth/2)^2 + 2*MassLength*(TotalLength/2)^2;
       
Params.J = [Params.Jx 0 0; 0 Params.Jz 0; 0 0 Params.Jy]

Params.Thrust = Params.m*Params.g;
Params.AngularSpeed = 850;

%% States
x = [px;...     % inertial frame north postion
     py;...     % inertial frame east position
     pz;...     % interial frame down position
     ub;...     % inertial velocity north 
     vb;...     % inertial velocity east
     wb;...     % inertial velocity down
     qw;...
     qx;...
     qy;...
     qz;...
     p;...
     q;...
     r];

 
%  Params.x0=[0;...pn
%             0;...pe
%             0;...pd
%             0;...ui
%             0;...vi
%             0;...wi
%             1;...qw
%             0;...qx
%             0;...qy
%             0;...qz
%             0;...p
%             0;...q
%             0];...r

%% Variable Pitch
Params.R = RotorRadius;  %meters
Params.d = sqrt((TotalLength/2)^2 + (TotalWidth/2)^2); %meters
Params.density = 0.0123;
Params.Vtip = Params.AngularSpeed*Params.R;
Params.K = Params.density*pi*Params.R^2*Params.Vtip^2;                       
Params.c = 0.1;

%% Controller
Velocity.body              = [ub; vb; wb];
quat.scalar                =  qw;
quat.vector                = [qx; qy; qz];
quat.full                  = [quat.scalar; quat.vector];
omega.body                 = [p; q; r];  

%[force.total; moment.body] = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c]*[f1; f2; f3; f4]
%force.body                 = [0; 0; force.total];
gravity.inertial            = [0; 0; 9.81];
% 
% desired.x = 0;
% desired.y = 0;
% desired.z = 0;
% 
% desired.ub = 0;
% desired.vb = 0;
% desired.ub = 0;
% 
% % Find desired acceleration from position error
% error.proportional = [state.px; state.py; state.pz] - [desired.x; desired.y; desired.z];
% error.dot          = [state.ub; state.vb; state.wb] - [desired.ub; desired.vb; desired.ub];
% integral.error     = integral.error + error.proportional*Params.dt;
% 
% feedback.acceleration = -Params.pos.kp*error.proportional - Params.pos.ki*integral.error - Params.pos.kd*error.dot;
% 
% % Calculate the change in force on the body
% 
% force.inertial       = Params.m * (desired.acceleration + feedback.acceleration + gravity.inertial);
% ForceBar.inertial    = force.inertial/norm(force.inertial);
% force.total          = norm(force.inertial);
% 
% %force.body = [0; 0; force.total];
% %ForceBar.body = force.body/normal(force.body);
% ForceBar.body = [0; 0; 1];
% 
% % Calculate the desired orientation
% desired.QuatNoYaw = 1/sqrt(2*(1 + ForceBar.inertial' * ForceBar.body)) * [ 1 + ForceBar.inertial' * ForceBar.body; cross(ForceBar.inertial, ForceBar.body)];  
% desired.quat = quatmultiply(desired.QuatNoYaw',[cos(desired.yaw) 0 0 sin(desired.yaw/2)]); 
% 
% feedback.jerk = (feedback.acceleration - prev.feedacclereation)/Params.dt;
% desired.jerk  = (desired.acceleration  - prev.desiredacceleration)/Params.dt;
% prev.feedacclereation     = feedback.acceleration;
% prev.desiredacceleration  = desired.acceleration;
% 
% xdt.ForceInertial = Params.m*(feedback.jerk + desired.jerk);
% 
% xdt.ForceBarInertial = xdt.ForceInertial/norm(force.inertial) - force.inertial*(force.inertial' * xdt.ForceInertial)/(norm(force.inertial)^3); 
% 
% % Calculate the desired attitude rate
% desired.OmegaBody = cross(ForceBar.inertial,xdt.ForceBarInertial);
% desired.OmegaBody(3) = Psidt;
% %xdt.ForceBarInterialxy = cross(desired.OmegaBody,ForceBar.Inertial);
% %xdt.ForceBarInterialz = Psidt;
% 
% %Quaternion Error in body frame
% quat.inertial = quat.full';
% error.quat = quatmultiply(quat.inertial, desired.quat);


syms f1 m1 m2 m3 s
u = [f1 m1 m2 m3]
% moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - desired.OmegaBody);
force.total = f1
moment.body = [m1; m2; m3]

Th = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c];
individialforces = inv(Th)*[force.total; moment.body];


%% Dynamics
out = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c]*[individialforces]
% force.total = out(1);
% moment.body = out(2:4);

%[0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
Velocity.inertial =  quatmultiply(quatmultiply(quatconj(quat.full'),[0 Velocity.body']),quat.full');
xdot(1:3) = Velocity.inertial(2:4);

%[0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
force.body            = [0; 0; force.total];
Accleration.inertial  = (1/Params.m)*quatmultiply(quatmultiply(quatconj(quat.full'),[0 force.body']),quat.full')' - [0; gravity.inertial];
xdot(4:6)             = Accleration.inertial(2:4);

xdt.q                 = 0.5*quatmultiply(quat.full',[0 omega.body']);
xdot(7:10)            = xdt.q;

xdt.omegabody         = Params.J^-1 *[moment.body - cross(omega.body,Params.J*omega.body)];
xdot(11:13)           = xdt.omegabody;


l1 = [px py pz ub vb wb qw qx qy qz p q r]
l2 = [0  0  0  0  0  0  0  0  0  0  0 0 0];
 
A  = jacobian(xdot,x)
B  = jacobian(xdot,u)

A1 = subs(A,l1,l2)
B1 = subs(B,l1,l2)

C = eye(size(A1,1))

D = zeros(16,4)

tf1 = C*(s*eye(size(A1,1)) - A1)^-1*B1


%% Tune psi
figure(1)
phitf = tf1(11,2)
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
[Cphi,info] = pidtune(sysPhi,'PD',opts);
Tphi = feedback(sysPhi*Cphi,1);
Gphi = feedback(sysPhi,Cphi);
stepplot(Tphi)
%stepplot(Gphi)
Cphi
%% Tune Theta
figure(2)
thetatf = tf1(12,3)

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
[CTheta,info] = pidtune(sysTheta,'PI',opts);
TTheta = feedback(sysTheta*CTheta,1);
GTheta = feedback(sysTheta,CTheta);
stepplot(TTheta)
%stepplot(GTheta)
CTheta

%% Tune psi
figure(3)
psitf = tf1(13,4)

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
[Cpsi,info] = pidtune(sysPsi,'PD',opts);
Tpsi = feedback(sysPsi*Cpsi,1);
Gpsi = feedback(sysPsi,Cpsi);
stepplot(Tpsi)
%stepplot(Gpsi)
Cpsi
