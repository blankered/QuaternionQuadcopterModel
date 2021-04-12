clear all; clc; 
 
global Params Desired integral prev;
run parameters_realtime.m; 
%run importfile.m;
% Set any custom initial conditions here

% quat      = angle2quat(0, 0, 0, 'XYZ');

%LQR Initial Conditions
Params.x0(1) = 0; ... x
Params.x0(2) = 0; ... y
Params.x0(3) = 0; ... z
Params.x0(4) = 0;  
% 
% Params.x0(7) = quat(1);
% Params.x0(8) = quat(2);
% Params.x0(9) = quat(3);
% Params.x0(10) = quat(4);  

% Params.x0(11)= 0.1;
% Params.x0(12)= 0.1;
% Params.x0(13)= 0.1;



%% PID Controls
%% Attitude Controller
options = simset('SrcWorkspace','current');
%sim('AnimationAttControllerVariableQuadDynamicsMdl.slx',[],options);
%sim('models/HeadlessAttRateControllerVariableQuadDynamicsMdl.slx',[],options);
%sim('models/HeadlessAttControllerVariableQuadDynamicsMdl.slx',[],options);
sim('models/HeadlessPositionControllerVariableQuadDynamicsMdl.slx',[],options);
%sim('models/HeadlessAttControllerVariableQuadDynamicsMdl.slx',[],options);
%sim('models/Helical_Traj_HeadlessPositionControllerVariableQuadDynamicsMdl.slx',[],options);
%sim('models/PID_HeadlessWaypointControllerVariableQuadDynamicsMdl.slx',[],options)
%sim('models/PID_HeadlessTrackingControllerVariableQuadDynamicsMdl.slx',[],options)
%sim('models/LQR_3_HeadlessPositionControllerVariableQuadDynamicsMdl.slx',[],options)

run LogPLotter.m
%run LogPLotterAtt.m
visualize_test_dp(States)
visualize_test_dp_vec(States, Control_Command, Vector)
