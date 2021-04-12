function output = QuaternionController(u,Params)
global prev Desired Integral

Desired.phi   = u(1);
Desired.theta = u(2);
Desired.psi   = u(3);

%% Testing Purposes
force.total        = Params.m*Params.g;
Desired.Thrust     = force.total;
%% Define the state

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

Th = [Params.K                                 Params.K                                 Params.K                                 Params.K;...
      Params.K*Params.d                       -Params.K*Params.d                       -Params.K*Params.d                        Params.K*Params.d;...
      Params.K*Params.d                        Params.K*Params.d                       -Params.K*Params.d                       -Params.K*Params.d;...
      1.5*Params.K*Params.R*sqrt(state.CT1/2) -1.5*Params.K*Params.R*sqrt(state.CT2/2)  1.5*Params.K*Params.R*sqrt(state.CT3/2) -1.5*Params.K*Params.R*sqrt(state.CT4/2)];

% Rotation of Velocity from Inertial Frame to Body FrameParams.
velocity.inertial          = [state.ui;
                              state.vi;
                              state.wi];
                          
quat.scalar                =  state.qw;

quat.vector                = [state.qx;
                              state.qy;
                              state.qz];
                          
quat.full                  = [quat.scalar;
                              quat.vector];
                          
omega.body                 = [state.p;
                              state.q;
                              state.r];
                          
% Acceleration Error & Velocity Error                          
[Desired.ui, Desired.vi, Desired.wi] = deal(0, 0, 0)

velocity.error = [Desired.ui - state.ui;
                  Desired.vi - state.vi;
                  Desired.wi - state.wi];
              
velocity.xdt.error = [0; 0; 0];          
              
              
%% Desired vehicle orientation in the Inertial-frame
quat.inertial      = quat.full';
quat.desired       = eul2quat([Desired.psi, Desired.theta, Desired.phi],'ZYX');

%% Command filtered backstepping controller step 3

% (quat.xddt - Desired.XddtQuat) + Kd*(Traj.quat.xdt - quat.xdt) + kp*(Traj.quat.inertial - quat.inertial) + ki*(quat.integral)
% quat.xddt = Desired.XddtQuat - Kd*(Traj.quat.xdt - quat.xdt) - kp*(Traj.quat.inertial - quat.inertial) - ki*(quat.integral)
% quat.xddt = Desired.XddtQuat - Kd*(0 - quat.xdt) - kp*(quat.error) - ki*(quat.integral)
% quat.xddt  = quatmultiply(quat.xdt*Params.att.Quat.kd,quat.error*Params.att.Quat.kp)
% quat.xddt = Desired.XddtQuat + Params.att.Quat.kd*quat.xdt' + Params.att.Quat.kp*quat.error'  + Params.att.Quat.ki*Integral.QuatError


% Calulate Desired Angular Acceleration in the Body Frame
% Body-fixed coordinates
H= sign(quat.inertial(1))

quat.error = quatmultiply(quat.inertial,quatinv(quat.desired))

Herror = sign(quat.error(1))

omega.xdt = Params.alpha *(-Params.K3 * Herror * transpose(quat.error(2:4)) - omega.body)

Desired.omega.filtered = Desired.omega.filtered + omega.xdt*Params.dt

quat.xdt.inertial = 0.5*Params.Iota(quat.inertial)*Desired.omega.filtered

WT = (force.total/Params.m)*(quat.error(1)*eye(3) - Params.Skew(quat.error(2:4)))*Params.Skew(Params.Quat2Rot(quat.inertial)*Params.E3)

omega.error = omega.body - Desired.omega.unfiltered

quat.xdt.error = 0.5*Params.Iota(quat.error)*omega.error

omega.unfiltered.desired = -Params.K3 * H * quat.xdt.error(2:4) - transpose(WT) * velocity.error + Params.Quat2Rot(quat.xdt.error) * Desired.omega.filtered

 

%% Calculate Roll, Pitch, Yaw Rates - P Q R
pqrdt = [((Params.Jy-Params.Jz)/Params.Jx)*state.q*state.r;
         ((Params.Jz-Params.Jx)/Params.Jy)*state.p*state.r;
         ((Params.Jx-Params.Jy)/Params.Jz)*state.p*state.q] + [(1/Params.Jx)*l; (1/Params.Jy)*m; (1/Params.Jz)*n];

Desired.pqrddt = -(Params.Wn^2)*(omega.body - omega.desired) - 2*Params.Eta*Params.Wn*pqrdt
     
Desired.pqrdt = Desired.pqrdt + Desired.pqrddt*Params.dt

Desired.pqr = Desired.pqr + Desired.pqrdt*Params.dt

%% Integrate to get feedforward jerk
%moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - Desired.OmegaBody);
Error.prop        = [Desired.pqr(1)   - state.p;
                     Desired.pqr(2)  - state.q;
                     Desired.pqr(3)   - state.r];
                 
Error.dot         = [Desired.pqrdt(1) - pqrdt(1);
                     Desired.pqrdt(2) - pqrdt(2);
                     Desired.pqrdt(3) - pqrdt(3)];
                 
Integral.ErrorPQR = Integral.ErrorPQR + Error.prop*Params.dt;

pqrddt = Desired.pqrddt + Params.att.pqr.kp * Error.prop + Params.att.pqr.ki * Integral.ErrorPQR + Params.att.pqr.kd * Error.dot
output = [Desired.Thrust, pqrddt(1), pqrddt(2), pqrddt(3)];


%% Old Code
% 
% %Queternion Error in the body-frame
% quat.error         = quatmultiply(quatconj(quat.inertial), Desired.quat);
% 
% % Body-frame time derivative of quat
% quat.xdt           = 0.5*quatmultiply([1 omega.body'],quat.inertial);
% 
% % Inertial-frame time derivative of quat
% Integral.QuatError = Integral.ErrorQuat + quat.error' *Params.dt;
% quat.errordt = 0.5*quatmultiply(quat.inertial,quatmultply(omega.errorbody,quat.error))


