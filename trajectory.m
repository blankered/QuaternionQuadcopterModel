function traj = trajectory(u,Params)

%Figure 8
% Params.lqr.a = 2.5;
% Params.lqr.b = 1.75;
% Params.lqr.c = 2;
% Params.lqr.n = -0.75;
% Params.lqr.T = 15;
% Params.lqr.w1 = 2*pi/Params.lqr.T;
% Params.lqr.w2 = Params.lqr.w1/2;
% Params.lqr.w3 = Params.lqr.w1;
% Params.lqr.psi_r = 0;

%%Circle
Params.lqr.a = 1.5;
Params.lqr.b = 1.5;
Params.lqr.c = 1;
Params.lqr.n = -0.75;
Params.lqr.T = 7;
Params.lqr.w1 = 2*pi/Params.lqr.T;
Params.lqr.w2 =Params.lqr.w1;
Params.lqr.w3 = Params.lqr.w1;
Params.lqr.psi_r = 0;

t = u;
traj = zeros(4,1);

traj(1) = Params.lqr.a*cos(Params.lqr.w2*t);
traj(2) = Params.lqr.b*sin(Params.lqr.w1*t);
traj(3) = Params.lqr.n + Params.lqr.c*sin(Params.lqr.w3*t);
traj(4) = -Params.lqr.a*Params.lqr.w2*sin(Params.lqr.w2*t);
traj(5) =  Params.lqr.b*Params.lqr.w1*cos(Params.lqr.w1*t);
traj(6) =  Params.lqr.c*Params.lqr.w3*cos(Params.lqr.w3*t);
traj(7) = 0;
traj(8)  = -Params.lqr.a*Params.lqr.w2^2*cos(Params.lqr.w2*t);
traj(9) = -Params.lqr.b*Params.lqr.w1^2*sin(Params.lqr.w1*t);
traj(10) = -Params.lqr.c*Params.lqr.w3^2*sin(Params.lqr.w3*t);
traj(11) = 0;

% traj(5) = 0;
% traj(6) = 0;
% traj(7) = 0;
% traj(8) = 0;
% 
% traj(9)  = 0;
% traj(10) = 0;
% traj(11) = 0;
% traj(12) = 0;

end