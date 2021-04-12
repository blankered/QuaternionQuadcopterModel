function TrajectoryDesired = DesiredWaypoint(u,Params)

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


x = u;
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

if  (state.px < waypoint(Params.NextWaypoint ,1)+0.01) && (state.px > waypoint(Params.NextWaypoint ,1)-0.01 )&& (state.py < waypoint(Params.NextWaypoint ,2)+0.01) && (state.py > waypoint(Params.NextWaypoint ,2)-0.01)
    desired.x = waypoint(Params.NextWaypoint +1,1);
    desired.y = waypoint(Params.NextWaypoint +1,2);
    desired.z = sqrt(desired.x^2 + desired.y^2);
    if Params.NextWaypoint  <= (size(waypoint,1)-2);
        Params.NextWaypoint = Params.NextWaypoint +1;
    end
else
    desired.x = waypoint(Params.NextWaypoint,1);
    desired.y = waypoint(Params.NextWaypoint,2);
    desired.z = sqrt(desired.x^2 + desired.y^2);
end


desired.psi = 0;


TrajectoryDesired(1) = desired.x;
TrajectoryDesired(2) = desired.y;
TrajectoryDesired(3) = desired.z;
TrajectoryDesired(4) = desired.psi;

