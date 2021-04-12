function in=TrajControlI_diffFlatness(uu,Params)
%==========================================================================
%This is the position control code given to ECE 6320 Students
%Currently same pid gains are used for all the angles (need to change this)
%
%10/09/2014: Last modified by Rajikant Sharma
%==========================================================================
Xtrue  = uu(1:17);
% Position
state.pn = Xtrue(1);
state.pe = Xtrue(2);
state.pd = Xtrue(3);
% Velocity
state.vn = Xtrue(4); 
state.ve = Xtrue(5);
state.vd = Xtrue(6);
% Quaternion Pose
state.qw = Xtrue(7);
state.qx = Xtrue(8);
state.qy = Xtrue(9);
state.qz = Xtrue(10);
% Angular Velocity
state.p = Xtrue(11);
state.q = Xtrue(12);
state.r = Xtrue(13);
% Thrust Coefficients
state.CT1 = Xtrue(14);
state.CT2 = Xtrue(15);
state.CT3 = Xtrue(16);
state.CT4 = Xtrue(17);

state.psi = 0;
%% Control state
x=[state.pn state.pe state.pd  state.vn state.ve state.vd state.psi]';

%% reference trajectory
xr=uu(17+1:17+7);

%% reference input
ur=uu(17+7+1:17+7+4);

%% Rotation matrix R(psi) needded to compute z vector is
% R_psi=[cos(psi) sin(psi) 0;...
%       -sin(psi) cos(psi) 0;...
%       0         0        1];

%% Linear state-space system
x_error =  x-xr;
u_error = -Params.K_gain * x_error;
u = u_error + ur;

Upsi = 0;


in       = zeros(4,1);
in(1,1)  = u(1); 
in(2,1)  = u(2);
in(3,1)  = u(3);
in(4,1)  = Upsi;

%% Just a reminder the mass of quadrotor is P.m
%% ALso the Gain Matrix P.K is computed in the param file using LQR so use it
%T= Params.m * sqrt(u'*u); % thrust

%% define a vector

% Upos = Params.Rbi(phi,theta,psi)* [0; 0; -T]*(1/Params.m);
% Upsi  = q*(sin(phi)/cos(theta)) + r*(cos(psi)/cos(theta));

%Upsi = 0;
%z        =   R_psi * u(1:3) * (Params.m/-T);

%z        =   R_psi * Upos * (Params.m/-T);
%phi_c    =   asin(-z(2));
%theta_c  =   atan2(z(1),z(3));
%phidt = q;
%r_d      =   Upsi * cos(theta)*cos(phi) - phidt * sin(phi);

%% Input % 
% in       = zeros(4,1);
% in(1,1)  = T; 
% in(2,1)  = phi_c;% roll angle control
% in(3,1)  = theta_c;% pitch angle control
% in(4,1)  = r_d;% yaw angle control



