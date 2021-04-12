

Input Desired.x = 0;
Input Desired.y = 0;
Input Desired.z = 0;

Input Desired.ui = 0;
Input Desired.vi = 0;
Input Desired.wi = 0;

% Position
px = x(1);
py = x(2);
pz = x(3);
% Velocity
ub = x(4); 
vb = x(5);
wb = x(6);
% Quaternion Pose
qw = x(7);
qx= x(8);
qy= x(9);
qz = x(10);
% Angular Velocity
p = x(11);
q = x(12);
r = x(13);

% Rotation of Velocity from Inertial Frame to Body FrameParams.
Velocity.body              = [ub; vb; wb];
quat.scalar                = qw;
quat.vector                = [qx; qy; qz];
quat.full                  = [quat.scalar; quat.vector]
Omega.body                 = [p; q; r]   

[force.total; moment.body] = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c]*[f1; f2; f3; f4]
force.body                 = [0; 0; force.total];
gravity.inertial           = [0; 0; 9.81];

% Find desired acceleration from position error
error.proportional = [state.x; state.y; state.z] - [desired.x; desired.y; desired.z];
error.dot          = [state.ui; state.vi; state.wi] - [desired.ui; desired.vi; desired.ui];
error.integral     = 0.5*(error.proportional - Previous.error.proportional)*dt;

feedback.acceleration = -kp*error.proportional - ki*error.integral - kd*error.dot;

% Calculate the change in force on the body
desired.acceleration = [0; 0; 0];

force.inertial       = Params.mass * (desired.acceleration + feedback.acceleration + gravity.inerital);

ForceBar.inertial    = force.inertial/normal(force.inertial);

force.body = [0; 0; force.total];

ForceBar.body = force.body/normal(force.body);

% Calculate the desired orientation
desired.QuatNoYaw = 1/sqrt(2*(1 + ForceBar.Inertial' * ForceBar.body)) * [ 1 + ForceBar.Inertial' * ForceBar.body; cross(ForceBar.inertial, ForceBar.body);  
desired.Quat = quatmult(desired.QuatNoYaw,[cos(desired.yaw); 0; 0; sin(desired.yaw/2)]); 

xdt.ForceBar.Inertial = xdt.force.inertial/norm(force.inertial) - force.inertial*(force.inertial' * xdt.force.inertial)/(norm(force.inertial)^3); 

% Calculate the desired attitude rate
desired.OmegaBody = [cross(ForceBar.Inertial,xdt.ForceBar.Inertial); desired.yawrate];

xdt.ForceBarInterialxy = cross(desired.OmegaBody,ForceBar.Inertial);
xdt.ForceBarInterialz = Psidt;

feedback.jerk = (feedback.acceleration(time) - feedback.acceleration(time -1))/dt;
desired.jerk = (desired.acceleration(time) - desired.acceleration(time -1))/dt;
xdt.Force.inertial = m*(feedback.jerk + desired.jerk);

error.quat = quatmult(inertial.quat, desired.quat);
moment.body = -sgn(error.quat(1))*Gain.Porportional*error.quat(2:4)') - Gain.derivative*(omega.body - desired.omega.body);


Th = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c];

Motor.force = inv(Th)*[force.total; moment.body];[force.total; moment.body] = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c]*[f1; f2; f3; f4]


%%
% [0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
% xdot(1:3) = Velocity.Inertial;
% 
% [0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
% xdot(4:6) = Accleration.inertial;
% 
% xdt.q = 0.5*quatmultiply(quat.full,[0; omega.body]);
% xdot(7:10) =  xdt.q;Omega.body = [p; q; r] 
% 
% xdt.omega.body = J^-1 *[moment.body - cross(omega.body,J*omega.body)];
% xdot(11:14) = xdt.omega.body;

%% Dynamic Model


% Input to the Dynamic Model
[force.total; moment.body] = [1 1 1 1; Params.d 0 -Params.d 0; 0 Params.d 0 -Params.d; -Params.c Params.c -Params.c Params.c]*[f1; f2; f3; f4]

force.body = [0; 0; force.total];

quat.full = [quat.scalar;
             quat.vector]

% Rotation of Velocity from Inertial Frame to Body Frame
Velocity.inertial = [ui; vi; wi];
Velocity.body     = [ub; vb; wb];
Omega.body = [p; q; r]  

d = ArmLength;
c = DragCoefficient;

[0; Velocity.interial] =  quatmultiply(quatmultiply(quatconj(quat.full),Velocity.body),quat.full);
xdot(1:3) = Velocity.Inertial;

[0; Accleration.inertial] =  (1/mass)*quatmultiply(quatmultiply(quatconj(quat.full),force.body),quat.full) - [0; gravit.inertialy];
xdot(4:6) = Accleration.inertial;

xdt.q = 0.5*quatmultiply(quat.full,[0; omega.body]);
xdot(7:10) =  xdt.q;

xdt.omega.body = J^-1 *[moment.body - cross(omega.body,J*omega.body)];
xdot(11:14) = xdt.omega.body;



