function output = QuaternionController(u,Params)
global prev Desired Integral

% Desired.p   = u(1);
% Desired.q   = u(2);
% Desired.r   = u(3);
% Desired.yaw = u(4);

Desired.phi   = u(1);
Desired.theta = u(2);
Desired.psi   = u(3);

% Desired.x   = u(1);
% Desired.y   = u(2);
% Desired.z   = u(3);
% Desired.yaw = u(4);

Psidt = 0;

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

pqrdt = [((Params.Jy-Params.Jz)/Params.Jx)*state.q*state.r;
         ((Params.Jz-Params.Jx)/Params.Jy)*state.p*state.r;
         ((Params.Jx-Params.Jy)/Params.Jz)*state.p*state.q] + [(1/Params.Jx)*l; (1/Params.Jy)*m; (1/Params.Jz)*n]
     
% Rotation of Velocity from Inertial Frame to Body FrameParams.
velocity.interial          = [state.ui; state.vi; state.wi];
quat.scalar                =  state.qw;
quat.vector                = [state.qx; state.qy; state.qz];
quat.full                  = [quat.scalar; quat.vector];
omega.body                 = [state.p; state.q; state.r];  

% Find Desired acceleration from position error
Error.proportional    = [Desired.x;     Desired.y;     Desired.z]     - [state.px; state.py; state.pz];
Error.dot             = [Desired.vi(1); Desired.vi(2); Desired.vi(3)] - [state.ui; state.vi; state.wi];
Integral.ErrorPos     = Integral.ErrorPos + Error.proportional*Params.dt;

feedback.acceleration = Params.pos.kp*Error.proportional + Params.pos.ki*Integral.ErrorPos + Params.pos.kd*Error.dot;

velocity.error        = Error.dot;
velocity.xdt.error    = feedback.acceleration;  

% Equation 3.9 - Page 57 - Mark Cutler
force.Inertial        = Params.m * (feedback.acceleration + Desired.acceleration  + [0; 0; Params.g])

% Equation 12  - Page 57 - Mark Culter
ForceBar.Inertial     = force.Inertial/norm(force.Inertial);

% Mark Culter: Page 58 
force.total          = norm(force.Inertial);

% Mark Culter: Page 55
force.body           = [0; 0; force.total];
ForceBar.body        = force.body/norm(force.body);

% Calculate the Desired orientation
Desired.quat       = ang2quat([Desired.psi, Desired.theta, Desired.phi],'ZYX');
%Desired.QuatNoYaw = 1/sqrt(2*(1 + transpose(ForceBar.Inertial) * ForceBar.body)) * [ 1 + transpose(ForceBar.Inertial) * ForceBar.body; cross(ForceBar.Inertial, ForceBar.body)];  
%Desired.quat      = quatmultiply(Desired.QuatNoYaw',[cos(Desired.yaw) 0 0 sin(Desired.yaw/2)]); 

feedback.jerk = (feedback.acceleration - prev.feedacclereation)/Params.dt;
Desired.jerk  = (Desired.acceleration  - prev.Desiredacceleration)/Params.dt;

prev.feedacclereation     = feedback.acceleration;
prev.Desiredacceleration  = Desired.acceleration;

xdt.ForceInertial    = Params.m*(feedback.jerk + Desired.jerk);
xdt.ForceBarInertial = xdt.ForceInertial/norm(force.Inertial) - (force.Inertial*(force.Inertial' * xdt.ForceInertial)/(norm(force.Inertial)^3)); 

% Calculate the Desired attitude rate
Desired.pqr    = cross(ForceBar.Inertial,xdt.ForceBarInertial);
Desired.pqr(3) = Psidt;

% xdt.ForceBarInterialxy = cross(Desired.OmegaBody,ForceBar.Inertial);
% xdt.ForceBarInterialz  = Psidt;
    
%% Desired vehicle orientation in the Inertial-frame
% quat.inertial  = quat.full';
% quat.error     = quatmultiply(quat.inertial,quatconj(Desired.quat))

% Calulate Desired Angular Acceleration in the Body Frame
% H  = sign(quat.error(1))
% WT = (force.total/Params.m)*(quat.error(1)*eye(3) - Params.Skew(quat.error(2:4)))*Params.Skew(Params.Quat2Rot(quat.inertial)*Params.E3)
%      
%% Desired.pqr = -Params.K3 * H * transpose(quat.error(2:4)) - transpose(WT) * velocity.error - Params.Quat2Rot(quat.error) * omega.body  
%Desired.pqr = -H * Params.att.Quat.kp * transpose(quat.error(2:4)) - transpose(WT) * velocity.error - Params.att.Quat.kd*(Params.Quat2Rot(quat.error)*omega.body) 

% omega.error       = Desired.OmegaBody - omega.body 
% quat.xdt.inertial = 0.5*Params.Iota(quat.inertial)*omega.body
% quat.xdt.error    = 0.5*Params.Iota(quat.error)*omega.error
% 
% [dq_0, dq_1, dq_2, dq_3]  = deal_array(transpose(quat.xdt.inertial));
% [dqe_0,dqe_1,dqe_2,dqe_3] = deal_array(quat.xdt.error);
% [q_0,q_1,q_2,q_3]         = deal_array(quat.inertial);
% [qe_0,qe_1,qe_2,qe_3]     = deal_array(quat.error)
% 
% %dF = norm(xdt.ForceBarInertial)
% dF  = 0
% 
% FuncWdt = Params.xdt.WT(force.total,dF,dq_0,dq_1,dq_2,dq_3,dqe_0,dqe_1,dqe_2,dqe_3,Params.m,q_0,q_1,q_2,q_3,qe_0,qe_1,qe_2,qe_3)
% 
% Desired.pqrdt = - H * quat.xdt.error(2:4) - (transpose(FuncWdt)*velocity.error + transpose(WT)*velocity.xdt.error) + ...
%     Params.att.Quat.kd*(Params.Quat2Rot(quat.xdt.error)*omega.body + Params.Quat2Rot(quat.error)*pqrdt)

%% Integrate to get feedforward jerk
%moment.body = -sign(error.quat(1))*(Params.quat.kp*error.quat(2:4)') - Params.quat.kd*(omega.body - Desired.OmegaBody);
Error.prop        = [Desired.pqr(1)   - state.p;
                     Desired.pqr(2)   - state.q;
                     Desired.pqr(3)   - state.r];            
                 
Error.dot         = [Desired.pqrdt(1) - pqrdt(1);
                     Desired.pqrdt(2) - pqrdt(2);
                     Desired.pqrdt(3) - pqrdt(3)];
                 
%Integral.ErrorPQR = Integral.ErrorPQR + Error.prop*Params.dt;

Desired.Thrust = force.total

pqrddt = Desired.pqrddt + Params.att.pqr.kp * Error.prop + Params.att.pqr.ki * Integral.ErrorPQR + Params.att.pqr.kd * Error.dot
output = [Desired.Thrust, pqrddt(1), pqrddt(2), pqrddt(3)];


