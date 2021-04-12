function [output] = traj_helix(t, state, r, z_max)
% TRAJ_HELIX generats a helix in the xy plane of r 5m centered about the
%   point (0; 0; 0) starting at the point (r; 0; 0).
%   The z coordinate should start at 0 and end at z_max.

if nargin < 4, z_max = 5; end
if nargin < 3, r = 2; end
T       = 30;       % finishing time 10
rot = 4*pi;

if t >= T
    % hover controller input
    x       = cos(2*pi)*r;
    y       = sin(2*pi)*r;
    z       = z_max;
    pos     = [x; y; z];
    vel     = zeros(3,1);
    acc     = zeros(3,1);
else
    % 3-d position controller input
    % quintic polynomial
    t0      = 0;
    tf      = T;
    
    M       = ...
       [1    t0  t0^2    t0^3    t0^4    t0^5;
        0    1   2*t0    3*t0^2  4*t0^3  5*t0^4;
        0    0   2       6*t0    12*t0^2 20*t0^3;
        1    tf  tf^2    tf^3    tf^4    tf^5;
        0    1   2*tf    3*tf^2  4*tf^3  5*tf^4;
        0    0   2       6*tf    12*tf^2 20*tf^3];
    b       = ...
       [0    0;
        0    0;
        0    0;
        rot z_max;
        0    0;
        0    0];
    a       = M\b;
    out     = a(1,:) + a(2,:)*t + a(3,:)*t^2 + a(4,:)*t^3 + a(5,:)*t^4 + a(6,:)*t^5;
    outd    = a(2,:) + 2*a(3,:)*t + 3*a(4,:)*t^2 + 4*a(5,:)*t^3 + 5*a(6,:)*t^4;
    outdd   = 2*a(3,:) + 6*a(4,:)*t + 12*a(5,:)*t^2 + 20*a(6,:)*t^3;
    
    beta    = out(1,1);
    betad   = outd(1,1);
    betadd  = outdd(1,1);
    z       = out(1,2);
    zd      = outd(1,2);
    zdd     = outdd(1,2);
    % position
    x       = cos(beta)*r;
    y       = sin(beta)*r;
    pos     = [x; y; z];
    % velocity
    xd      = -y*betad;
    yd      =  x*betad;
    vel     = [xd; yd; zd];
    % acceleration
    xdd     = -x*betad^2 - y*betadd;
    ydd     = -y*betad^2 + x*betadd;
    acc     = [xdd; ydd; zdd];
end

% yaw and yawdot
% yaw = [xd; yd];
% yawdot = cross([xd, yd]',[x, y]')/([x,y]')^2;

yaw = [0; 0]
yawdot = [0];
% output desired state
% desired_state.pos = pos(:); %3
% desired_state.vel = vel(:); %3
% desired_state.acc = acc(:); %3
% desired_state.yaw = yaw;    
% desired_state.yawdot = yawdot;

output = [pos(:); vel(:); yaw(:); acc(:); yawdot];
end
