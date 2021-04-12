function TrajectoryDesired = TrackingWaypoint(u,Params)

global Params

waypoint = [0 0 0;
            1 0 0;
            0 0 0
            1 0 0];


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
    if Params.NextWaypoint  <= (size(waypoint,1)-2);
        Params.NextWaypoint = Params.NextWaypoint +1;
    end
else
    desired.x = waypoint(Params.NextWaypoint,1);
    desired.y = waypoint(Params.NextWaypoint,2);
    
end


desired.psi = 0;
desired.z = 0;

TrajectoryDesired(1) = desired.x;
TrajectoryDesired(2) = desired.y;
TrajectoryDesired(3) = desired.z;
TrajectoryDesired(4) = desired.psi;

