function Params = ret_param(Q,R,PD,mode)
  
Params = parameters_func();

switch mode
    case 1
        Params.lqr.a = 1.5;
        Params.lqr.b = 0.75;
        Params.lqr.c = 0;
        Params.lqr.n = -0.75;
        Params.lqr.T = 5;
        Params.lqr.w1 = 2*pi/Params.lqr.T;
        Params.lqr.w2 = Params.lqr.w1/2;
        Params.lqr.w3 = Params.lqr.w1;
    case 2
        Params.lqr.a = 1.5;
        Params.lqr.b = 0.75;
        Params.lqr.c = 0.5;
        Params.lqr.n = -0.75;
        Params.lqr.T = 10;
        Params.lqr.w1 = 2*pi/Params.lqr.T;
        Params.lqr.w2 = Params.lqr.w1/2;
        Params.lqr.w3 = Params.lqr.w1;
    case 3
        Params.lqr.a = 0.75;
        Params.lqr.b = 0.75;
        Params.lqr.c = 0;
        Params.lqr.n = -0.75;
        Params.lqr.T = 10;
        Params.lqr.w1 = 2*pi/Params.lqr.T;
        Params.lqr.w2 = Params.lqr.w1;
        Params.lqr.w3 = Params.lqr.w1;
end
Params.lqr.At = [zeros(3) eye(3) zeros(3,1)
                 zeros(3) zeros(3) zeros(3,1)
                 zeros(1,7)];
   
Params.lqr.Bt = [zeros(3)   zeros(3,1)
                 eye(3)     zeros(3,1)
                 zeros(1,3)  1];
   
[Params.K_gain,S,E] = lqr(Params.lqr.At,Params.lqr.Bt,Q,R);

xr = trajectory(0,Param);


Params.x0(1) = xr(1);
Params.x0(2) = xr(2);
Params.x0(3) = xr(3);
Params.x0(4) = xr(4);
Params.x0(5) = xr(5);
Params.x0(6) = xr(6);
Params.x0(7) = xr(7);


% Params.x0=[...
%     xr(1);...pn0
%     xr(2);...pe0
%     xr(3);...pd0
%     xr(5);...u0
%     xr(6);...v0
%     xr(7);...w0
%     0;...pn0
%     0;...pe0
%     0;...pd0
%     0;...u0
%     0;...v0
%     0;...w0
%     0;...phi0
%     0;...theta0
%     0;...psi0
%     0;...p0
%     0;...q0
%     0];%r0

% PD gains
% Kp_phi = 25.4648;
% Kd_phi = 2.4169;


% Kp_theta = 25.4648;
% Kd_theta = 1.7128;

Params.Kp_phi = PD.Kp_phi;
Params.Kd_phi = PD.Kd_phi;

Params.Kp_theta = PD.Kp_theta;
Params.Kd_theta = PD.Kd_theta;

Params.Kp_psidot = PD.Kp_psidot;
Params.Kd_psidot = PD.Kd_psidot;
% 
% Params.att.Quat.kp = [PD(1)   0 0; 0 PD(3)  0; 0 0 PD(5) ];
% Params.att.Quat.ki = [PD(2)   0 0; 0 PD(4)  0; 0 0 PD(6)];
% Params.att.Quat.kd = [3.21    0 0; 0 3.21   0; 0 0 0.233];




