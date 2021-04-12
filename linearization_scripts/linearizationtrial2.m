clc; clear all;

run parameters;

syms px py pz ub vb wb qw qx qy qz p q r CT1 CT2 CT3 CT4 Tdot T
global Params Desired Integral prev;

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
     r;...
     CT1;...    %Thrust Coefficient
     CT2;...    %Thrust Coefficient
     CT3;...    %Thrust Coefficient
     CT4];...    %Thrust Coefficient];

T = Params.K*(CT1 + CT2 + CT3 + CT4);
l = Params.K*Params.d*(CT1 - CT2 - CT3 + CT4);
m = Params.K*Params.d*(CT1 + CT2 - CT3 - CT4);
n = (Params.K*Params.R/sqrt(2))*(CT1^3/2 - (CT2^3/2) + CT3^3/2 - (CT4^3/2));

Th = [Params.K                          Params.K                           Params.K                           Params.K;...
      Params.K*Params.d                 -Params.K*Params.d                 -Params.K*Params.d                 Params.K*Params.d;...
      Params.K*Params.d                 Params.K*Params.d                  -Params.K*Params.d                -Params.K*Params.d;...
      1.5*Params.K*Params.R*sqrt(CT1/2) -1.5*Params.K*Params.R*sqrt(CT2/2) 1.5*Params.K*Params.R*sqrt(CT3/2) -1.5*Params.K*Params.R*sqrt(CT4/2)];
 
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

% Desired.quat       = eul2quat([Desired.phi Desired.theta Desired.psi],'ZYZ');
% 
% force.total        = Params.m*Params.g;
% Desired.Thrust     = force.total;
% quat.inertial      = quat.full';
% error.quat         = quatmultiply(quat.inertial, Desired.quat);
% xdt.q              = 0.5*quatmultiply(quat.full',[0 omega.body']);
% Desired.OmegaBody  = [0;0;0];
% 
% Error.prop         = error.quat(2:4)';
% Error.dot          = xdt.q(2:4)';
% Integral.ErrorQuat = Integral.ErrorQuat + Error.prop *Params.dt;
% 
% Desired.pqrdt      = [0; 0; 0];
% Desired.pqrdt      = -Params.att.Quat.ki*Integral.ErrorQuat -Params.att.Quat.kd*Error.dot -Params.att.Quat.kp*Error.prop

force.total = Params.m*Params.g
Desired.Thrust     = force.total;
velocity.error = [0; 0; 0]

syms s DesiredThrust dpqr1 dpqr2 dpqr3

u = [dpqr1; dpqr2; dpqr3];

quat.desired       = angle2quat(0, 0, 0);
quat.inertial      = transpose(quat.full);
%% Command filtered backstepping controller step 3
% Calulate Desired Angular Acceleration in the Body Frame
% Body-fixed coordinates
% H= sign(quat.inertial(1))
% 
% quat.error = quatmultiply(quat.inertial,quatinv(quat.desired))
% 
% WT = 2*(force.total/Params.m)*(quat.error(1)*eye(3) - Params.Skew(quat.error(2:4)))*Params.Skew(Params.Quat2Rot(quat.inertial)*Params.E3)
% 
% Desired.pqr = -Params.K3 * H * transpose(quat.error(2:4)) - transpose(WT) * velocity.error + Params.Quat2Rot(quat.error) * omega.body  

Desired.pqr = u;

% Calculate Roll, Pitch, Yaw Rates - P Q R
pqrdt = [((Params.Jy-Params.Jz)/Params.Jx)*q*r;
         ((Params.Jz-Params.Jx)/Params.Jy)*p*r;
         ((Params.Jx-Params.Jy)/Params.Jz)*p*q] + [(1/Params.Jx)*l;
                                                   (1/Params.Jy)*m;
                                                   (1/Params.Jz)*n];
%% Integrate to get feedforward jerk
%moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - desired.OmegaBody);
Error.prop        = [Desired.pqr(1)   - p;
                     Desired.pqr(2)   - q;
                     Desired.pqr(3)   - r];
                 
Error.dot         = [Desired.pqrdt(1) - pqrdt(1);
                     Desired.pqrdt(2) - pqrdt(2);
                     Desired.pqrdt(3) - pqrdt(3)];
                 
Integral.ErrorPQR = Integral.ErrorPQR + Error.prop*Params.dt;
Desired.pqrddt    = [0; 0; 0];

pqrddt = (Desired.pqrddt + Params.att.pqr.kp * Error.prop + Params.att.pqr.ki * Integral.ErrorPQR + Params.att.pqr.kd * Error.dot)
output = [Desired.Thrust; pqrddt(1); pqrddt(2); pqrddt(3)];


%% Dynamics
moment.body = output(2:4);

%[0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
Velocity.inertial =  quatmultiply(quatmultiply(quatconj(quat.full'),[0 Velocity.body']),quat.full');
xdot(1:3) = Velocity.inertial(2:4);

%[0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
force.body            = [0; 0; T];
Accleration.inertial  = (1/Params.m)*quatmultiply(quatmultiply(quatconj(quat.full'),[0 force.body']),quat.full')' - [0; gravity.inertial];
xdot(4:6)             = Accleration.inertial(2:4);

xdt.q                 = 0.5*quatmultiply(quat.full',[0 omega.body']);
xdot(7:10)            = xdt.q;

xdt.omegabody         = Params.J^-1 *[moment.body - cross(omega.body,Params.J*omega.body)];
xdot(11:13)           = xdt.omegabody;

Tdot = Params.Thrustkp*(output(1) - T);
ldot = Params.Jx*output(2) - (Params.Jy - Params.Jx)*(q*xdot(13) + r*xdot(12));
mdot = Params.Jy*output(3) - (Params.Jz - Params.Jx)*(p*xdot(13) + r*xdot(11));
ndot = Params.Jz*output(4) - (Params.Jx - Params.Jy)*(p*xdot(12) + q*xdot(11));

xdot(14:17) = (Th^-1)*[Tdot; ldot; mdot; ndot];

l1 = [px py pz ub vb wb qw qx qy qz p q r CT1   CT2   CT3   CT4   Tdot dpqr1 dpqr2 dpqr3 DesiredThrust];
l2 = [0  0  0  0  0  0  0  0  0  0  0 0 0 0.348 0.348 0.348 0.348 0    0    0    0   0];

% l1 = [px; py; pz; phi; theta; psi; ub; vb; wb; p; q; r; CT1;  CT2;  CT3;  CT4; Tdot; pddt; qddt; rddt]
% l2 = [0   0   0   0    0      0    0   0   0   0  0  0  0.348 0.348 0.348 0.348 0 0 0 0]';
 
A  = jacobian(xdot,x)
B  = jacobian(xdot,u)

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
