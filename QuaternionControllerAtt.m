function output = QuaternionControllerAtt(u,Params)
global prev Desired Integral

% Desired.p   = u(1);
% Desired.q   = u(2);
% Desired.r   = u(3);
% Desired.yaw = u(4);

Desired.phi   = u(1);
Desired.theta = u(2);
Desired.psi   = u(3);

Desired.pqr = zeros(3,1);
Desired.quat = angle2quat(Desired.phi, Desired.theta,Desired.psi, 'ZYX');

x = u(1+3:17+3);
% Position
state.px = x(1);
state.py = x(2);
state.pz = x(3);
% Velocity
state.ui = x(4); 
state.vi = x(5);
state.wi = x(6);
% Quaternion Pose
state.qw = x(7);
state.qx = x(8);
state.qy = x(9);
state.qz = x(10);
% Angular Velocity
state.p = x(11);
state.q = x(12);
state.r = x(13);
% Thrust Coefficients
state.CT1 = x(14);
state.CT2 = x(15);
state.CT3 = x(16);
state.CT4 = x(17);

T = Params.K*(state.CT1 + state.CT2 + state.CT3 + state.CT4);
l = Params.K*Params.d*(state.CT1 - state.CT2 - state.CT3 + state.CT4);
m = Params.K*Params.d*(state.CT1 + state.CT2 - state.CT3 - state.CT4);
n = (Params.K*Params.R/sqrt(2))*(state.CT1^3/2 - (state.CT2^3/2) + state.CT3^3/2 - (state.CT4^3/2));


pqrdt = [((Params.Jy-Params.Jz)/Params.Jx)*state.q*state.r;
         ((Params.Jz-Params.Jx)/Params.Jy)*state.p*state.r;
         ((Params.Jx-Params.Jy)/Params.Jz)*state.p*state.q] + [(1/Params.Jx)*l; (1/Params.Jy)*m; (1/Params.Jz)*n];
     
% Rotation of Velocity from Inertial Frame to Body FrameParams.
velocity.interial          = [state.ui; state.vi; state.wi];
quat.scalar                =  state.qw;
quat.vector                = [state.qx; state.qy; state.qz];
quat.full                  = [quat.scalar; quat.vector];
omega.body                 = [state.p; state.q; state.r];  



%% Desired vehicle orientation in the Inertial-frame
quat.inertial  = quat.full';
quat.error     = quatmultiply(quat.inertial,quatconj(Desired.quat));

%Calulate Desired Angular Acceleration in the Body Frame
omega.error       = omega.body - Desired.pqr;
quat.xdt.inertial = 0.5*Params.Iota(quat.inertial)*omega.body;
quat.xdt.error    = 0.5*Params.Iota(quat.error)*omega.error;

% Integral.ErrorQuat = Integral.ErrorQuat + (quat.error(2:4).')*Params.dt;
% Desired.pqrdt =  -sign(quat.error(1)) * Params.att.Quat.kp * quat.error(2:4).' -  Params.att.Quat.kd * quat.xdt.error(2:4) - Params.att.Quat.ki*Integral.ErrorQuat;
Desired.pqrdt =  -sign(quat.error(1)) * Params.att.Quat.kp * quat.error(2:4).' -  Params.att.Quat.kd * quat.xdt.error(2:4);
%% Integrate to get feedforward jerk
Error.prop        = [Desired.pqr(1)   - state.p;
                     Desired.pqr(2)   - state.q;
                     Desired.pqr(3)   - state.r];
                 
Error.dot         = [Desired.pqrdt(1) - pqrdt(1);
                     Desired.pqrdt(2) - pqrdt(2);
                     Desired.pqrdt(3) - pqrdt(3)];
                 
Integral.ErrorPQR = Integral.ErrorPQR + Error.prop*Params.dt;
pqrddt = Desired.pqrddt + Params.att.pqr.kp * Error.prop + Params.att.pqr.ki * Integral.ErrorPQR + Params.att.pqr.kd * Error.dot;


Desired.Thrust = Params.g * Params.m;
output = [Desired.Thrust, pqrddt(1), pqrddt(2), pqrddt(3)];


