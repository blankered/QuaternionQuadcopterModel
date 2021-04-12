%==========================================================================
%This is the quadrotor drawing code given to ECE 6320 Students
%This is used attitide control and position control (6 times)
%
%09/25/2014: Last modified by Rajikant Sharma
%===============================================================================

function drawQuad(uu,P)

%% States
% x = [px;...     % inertial frame north postion
%      py;...     % inertial frame east position
%      pz;...     % interial frame down position
%      ui;...     % inertial velocity north 
%      vi;...     % inertial velocity east
%      wi;...     % inertial velocity down
%      qw;...
%      qx;...
%      qy;...
%      qz;...
%      p;...
%      q;...
%      r];

% process inputs to function
pn       = uu(1);       % inertial North position
pe       = uu(2);       % inertial East position
pd       = uu(3);
u        = uu(4);
v        = uu(5);
w        = uu(6);
qw       = uu(7)      
qx       = uu(8)     
qy       = uu(9)      
qz       = uu(10)      
p        = uu(11);       
q        = uu(12);      
r        = uu(13);
[phi, theta, psi] = quat2eul([qw qx qy qz],'ZYZ')

NN=13;

%%
t=uu(NN+1); % time



% define persistent variables
persistent fig_quadrotor;
persistent fig_rotors;





% first time function is called, initialize plot and persistent vars
if t<=0.1,
    figure(1), clf
    
    
    %draws quadrotor at initial position
    [fig_quadrotor,fig_rotors] = drawquadrotor(pn,pe,pd,phi,theta,psi, P, [], [], 'normal');
    
    hold on
    %axis([-40,40,-40,40,-40,40]); % see if it is possible to change the axis and not distort the image
    title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        grid on
        view(32,47)  % set the vieew angle for figure  % set the view angle for figure
    
    
    % at every other time step, redraw base and rod
else
    
    drawquadrotor(pn,pe,pd,phi,theta,psi, P, fig_quadrotor, fig_rotors);
    
    drawnow;
end
end


%=======================================================================
% drawquadrotor
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function [handle,handle2] = drawquadrotor(pn,pe,pd,phi,theta,psi, P, handle, handle2, mode)

% define points on spacecraft
[V, F, F_r, patchcolors, rotor_colors] = quadrotorVFC(P);

% rotate spacecraft
V = rotateVert(V, phi, theta, psi);

% translate spacecraft
V = translateVert(V, [pn; pe; pd]);
  % transform vertices from NED to XYZ (for matlab rendering)
R = [0, 1, 0;...
     1, 0, 0;...
     0, 0, -1];...
      
V=(R*V')';

if isempty(handle),
    %   handle = patch('Vertices', V, 'Faces', F,...
    %                  'FaceVertexCData',patchcolors,...
    %                  'FaceColor','flat',...
    %                  'EraseMode', mode);
    handle = patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat');
    handle2 = patch('Vertices', V, 'Faces', F_r,...
        'FaceVertexCData',rotor_colors,...
        'FaceColor','flat');
else
    set(handle,'Vertices',V,'Faces',F);
    set(handle2,'Vertices',V,'Faces',F_r);
    drawnow
end
end

%=======================================================================
% drawpillarsl / draws left pillar
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================

function [V, F, F_r, patchcolors, rotor_colors]=quadrotorVFC(P)

% Define the vertices (physical location of vertices)
Vtemp = [...
    -P.boxlength P.boxwidth 0;... %1  center box
    -P.boxlength -P.boxwidth 0;... %2
    -P.boxlength -P.boxwidth P.boxheight;... %3
    -P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %4
    P.boxlength P.ystart+P.boxwidth P.zstart;... %5
    P.boxlength P.ystart-P.boxwidth P.zstart;... %6
    P.boxlength P.ystart-P.boxwidth P.zstart+P.boxheight;... %7
    P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %8
    -P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %9 arm left
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %10
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %11
    -P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %12
    -P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %13
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %14
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %15
    -P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %16
    P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %17 arm right
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %18
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %19
    P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %20
    P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %21
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %22
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %23
    P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %24
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %25 arm front
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %26
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %27
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %28
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %29
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %30
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %31
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %32
% back front
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %33 
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %34
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %35
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %36
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %37
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %38
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %39
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %40
% perimeter
    0 P.ystart-P.perimeter P.zstart+P.boxheight;... %41 perimeter
    0 P.ystart-P.perimeter P.zstart+P.boxheight-P.periwidth;... %42
    0 P.ystart+P.perimeter P.zstart+P.boxheight;... %43
    0 P.ystart+P.perimeter P.zstart+P.boxheight-P.periwidth;... %44
    P.perimeter P.ystart P.zstart+P.boxheight;... %45
    P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %46
    -P.perimeter P.ystart P.zstart+P.boxheight;... %47
    -P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %48
% camera
    0 0 P.boxheight+(1/12);... %49 
   -1/24 1/12 P.boxheight+(1/12)-(1/32);... %50
    1/24 1/12 P.boxheight+(1/12)-(1/32);... %51
    -1/24 1/12 P.boxheight+(1/12)+(1/32);... %52
    1/24 1/12 P.boxheight+(1/12)+(1/32);... %53
    ];

    c_pi = cos(2*pi/10:2*pi/10:2*pi)';
    
    s_pi = sin(2*pi/10:2*pi/10:2*pi)';
    
    Vtemp = [Vtemp;zeros(40,3)];
    
    Vtemp(54:end,:) = [-P.boxlength-P.armlength+P.rotor_rad*c_pi P.ystart+P.armwidth+P.rotor_rad*s_pi P.zstart+P.boxheight-P.armposition-P.armwidth*ones(10,1);... %13
    P.boxlength+P.armlength+P.rotor_rad*c_pi P.ystart+P.armwidth+P.rotor_rad*s_pi P.zstart+P.boxheight-P.armposition-P.armwidth*ones(10,1);... %14
    0+P.rotor_rad*c_pi P.ystart+P.boxlength+P.armlength+P.rotor_rad*s_pi P.zstart+P.boxheight-P.armposition+P.armwidth*ones(10,1);... %15
    0+P.rotor_rad*c_pi -P.ystart-P.boxlength-P.armlength+P.rotor_rad*s_pi P.zstart+P.boxheight-P.armposition+P.armwidth*ones(10,1);... %16
    ];

V=[Vtemp(:,1:2) -Vtemp(:,3)];

psi=pi/4;

R = [cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
      0, 0, 1];

R = R';

% rotate vertices
V= (R*V')';

% define faces as a list of vertices numbered above
F = [...
    1, 2, 3, 4;... % left
    5, 6, 7, 8;... % right
    2, 6, 7, 3;... % back
    1, 5, 8, 4;... % front
    4, 3, 7, 8;... % top
    1, 2, 6, 5;... % bottom
    16, 15, 11, 12;... %left arm
    15, 14, 10, 11;...
    13, 14, 10, 9;...
    13, 9, 12, 16;...
    19, 23, 22, 18;... %right arm
    20, 19, 23, 24;...
    17, 18, 22, 21;...
    17, 21, 24, 20;...
    31, 27, 28, 32;... %front arm
    29, 32, 28, 25;...
    29, 25, 26, 30;...
    31, 27, 26, 30;...
    35, 39, 38, 34;... %back arm
    35, 36, 40, 39;...
    33, 36, 40, 37;...
    34, 37, 38, 34;...
    41, 42, 46, 45;... %perimeter
    45, 46, 44, 43;...
    43, 44, 48, 47;...
    47, 48, 42, 41;...
    49, 50, 51, 49;... %camera
    49, 50, 52, 49;...
    49, 52, 53, 49;...
    49, 51, 53, 49;...
    ];

F_r = [...
    54:63;...          %rotors
    64:73;...
    74:83;...
    84:93;...
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];

patchcolors = [...
    mygreen;... % left
    mygreen;... % right
    mygreen;... % back
    myred;... % front
    mycyan;... % top
    mycyan;... % bottom
    myblue;... %left arm
    myblue;...
    myblue;...
    myblue;...
    myblue;... %right arm
    myblue;...
    myblue;...
    myblue;...
    myyellow;... %front arm
    myyellow;...
    myyellow;...
    myyellow;...
    myblue;... %back arm
    myblue;...
    myblue;...
    myblue;...
    myred;... %perimeter
    myred;...
    myred;...
    myred;...
    myyellow;...%camera
    myyellow;...
    myyellow;...
    myyellow;...
    ];
rotor_colors = [
    myred;myred;myyellow;mygreen];
end


%%%%%%%%%%%%%%%%%%%%%%%not done
function Vert=rotateVert(Vert,phi,theta,psi)
% 
%   R_roll = [...
%           1, 0, 0;...
%           0, cos(phi), sin(phi);...
%           0, -sin(phi), cos(phi)];
%       
%   R_pitch = [...
%           cos(theta), 0, -sin(theta);...
%           0, 1, 0;...
%           sin(theta), 0, cos(theta)];
%       
%   R_yaw = [...
%           cos(psi), sin(psi), 0;...
%           -sin(psi), cos(psi), 0;...
%           0, 0, 1];
%       
%   R = R_roll*R_pitch*R_yaw;  
  
R = [ cos(theta)*cos(psi)    sin(phi)*cos(psi)*sin(theta)-cos(phi)*sin(psi)     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
      cos(theta)*sin(psi)    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
     -sin(theta)             sin(phi)*cos(theta)                                 cos(phi)*cos(theta)];

% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose

R = R';

% rotate vertices
Vert= (R*Vert')';


end % rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
% not done
function Vert = translateVert(Vert, T)

Vert = Vert + repmat(T', size(Vert,1),1);

end % translateVert