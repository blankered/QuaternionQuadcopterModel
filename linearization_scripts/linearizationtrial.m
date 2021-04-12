clc; clear all;

run parameters;

syms px py pz ub vb wb qw qx qy qz p q r CT1 CT2 CT3 CT4 Tdot T

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
gravity.inertial            = [0; 0; 9.81];


syms pddt qddt rddt s
u = [pddt qddt rddt]
% moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - desired.OmegaBody);
force.total = T
moment.body = [l; m; n]


%% Dynamics

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

ldot = Params.Jx*u(1) - (Params.Jy - Params.Jx)*(q*xdot(13) + r*xdot(12));
mdot = Params.Jy*u(2) - (Params.Jz - Params.Jx)*(p*xdot(13) + r*xdot(11));
ndot = Params.Jz*u(3) - (Params.Jx - Params.Jy)*(p*xdot(12) + q*xdot(11));

xdot(14:17) = (Th^-1)*[Tdot; ldot; mdot; ndot];

l1 = [px py pz ub vb wb qw qx qy qz p q r CT1   CT2   CT3   CT4   Tdot pddt qddt rddt];
l2 = [0  0  0  0  0  0  0  0  0  0  0 0 0 0.348 0.348 0.348 0.348 0    0    0    0   ];

% l1 = [px; py; pz; phi; theta; psi; ub; vb; wb; p; q; r; CT1;  CT2;  CT3;  CT4; Tdot; pddt; qddt; rddt]
% l2 = [0   0   0   0    0      0    0   0   0   0  0  0  0.348 0.348 0.348 0.348 0 0 0 0]';
 
A  = jacobian(xdot,x)
B  = jacobian(xdot,u)

A1 = subs(A,l1,l2)
B1 = subs(B,l1,l2)

C = eye(size(A1,1))

D = zeros(17,3)

tf1 = C*(s*eye(size(A1,1)) - A1)^-1*B1


%% Tune psi
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
[CTheta,info] = pidtune(sysTheta,'PID',0.5);
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
