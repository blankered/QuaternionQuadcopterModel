 function TrajectoryDesired = DesiredTrajectory(u,Params)

global Params

waypoint = [0,0 ; 
            .8522, 1.964; 
            3.25, 6.389; 
            5.894, 9.841; 
            8.206, 12; 
            11.15, 13.83; 
            13.69, 14.74; 
            15, 15;
            17.38, 15.22;
            18.5, 15.16;
            19.41, 14.86;
            19.77, 14.52;
            20.01, 14;
            20.22, 13.09;
            20.41, 12.53;
            20.69, 12.06;
            20.95, 11.76;
            21.62, 11.26;
            21.94, 11.09;
            22.61, 10.82;
            23, 10.69;
            23.6, 10.54;
            24.43, 10.37;
            26.79, 10.01;
            28.31, 10.03;
            30, 10;
            31.06, 10.06;
            32.23, 10.25;
            33.2, 10.53;
            36.07, 12.04;
            37.77, 13.47;
            39.61, 15.58;
            41.13, 17.84;
            43.33, 22.1;
            45.29, 27.21;
            47, 33.06;
            48.11, 37.97;
            50, 50];


x = u(1:16);
% Position
px = x(1);%% 16 quad states
py = x(2);
pz = x(3);
% Angular Position
Phi = x(4);
Theta = x(5);
Psi = x(6);
% Body Velocity
ub = x(7);
vb= x(8);
wb= x(9);
% Angular Velocity
p = x(10);
q = x(11);
r = x(12);
% Coefficients
CT1 = x(13);
CT2 = x(14);
CT3 = x(15);
CT4 = x(16);

if  (px < waypoint(Params.NextWaypoint ,1)+0.1) && (px > waypoint(Params.NextWaypoint ,1)-0.1 )&& (py < waypoint(Params.NextWaypoint ,2)+0.1) && (py > waypoint(Params.NextWaypoint ,2)-0.1)
    desired.x = waypoint(Params.NextWaypoint +1,1);
    desired.y = waypoint(Params.NextWaypoint +1,2);
    if Params.NextWaypoint  <= (size(waypoint,1)-2);
        Params.NextWaypoint = Params.NextWaypoint +1;
    end
else
    desired.x = waypoint(Params.NextWaypoint,1);
    desired.y = waypoint(Params.NextWaypoint,2);
    
end

% if desired.x == waypoint(end)
%     return
% end
% 
% x_velocity = Params.Rbi(Phi,Theta,Psi)*x(7:9);
% vn     = x_velocity(1);% velocity in north
% ve     = x_velocity(2);% velocity in east
% vd     = x_velocity(3);% velocity in down

% Testing Simulation Only
% desired.x = 1;
% desired.y = 1;
desired.z = 1;
% desired.yaw = 0;

% desired.dx = (desired.x-px)/Params.dt
% desired.dy = (desired.y-py)/Params.dt
% desired.dz = (desired.z-pz)/Params.dt
% desired.dyaw = (desired.yaw-Psi)/Params.dt
% 
% desired.ddx = (desired.dx-vn)/Params.dt
% desired.ddy = (desired.dy-ve)/Params.dt
% desired.ddz = (desired.dz-vd)/Params.dt
% desired.ddyaw = (desired.dyaw-r)/Params.dt

desired.dx   = 0;
desired.dy   = 0;
desired.dz   = 0;
desired.dyaw = 0;

desired.ddx   = 0;
desired.ddy   = 0;
desired.ddz   = 0;
desired.ddyaw = 0;

referenceInputs = zeros(4,1);

TrajectoryDesired(1)  = desired.x;
TrajectoryDesired(2)  = desired.y;
TrajectoryDesired(3)  = desired.z;
TrajectoryDesired(4)  = desired.dx;
TrajectoryDesired(5)  = desired.dy;
TrajectoryDesired(6)  = desired.dz;
TrajectoryDesired(7)  = desired.dyaw;
TrajectoryDesired(10) = desired.ddy;
TrajectoryDesired(11) = desired.ddx;
TrajectoryDesired(10) = desired.ddz - Params.g;
TrajectoryDesired(11) = desired.ddyaw;

