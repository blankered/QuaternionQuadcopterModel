figure(1);
%% 16 quad States
PlotLog.px = States.Data(:,1)
PlotLog.py = States.Data(:,2);
PlotLog.pz = States.Data(:,3);

PlotLog.ub = States.Data(:,4);
PlotLog.vb = States.Data(:,5);
PlotLog.wb = States.Data(:,6);

PlotLog.qw = States.Data(:,7);
PlotLog.qx = States.Data(:,8);
PlotLog.qy = States.Data(:,9);
PlotLog.qz = States.Data(:,10);

PlotLog.p = States.Data(:,11);
PlotLog.q = States.Data(:,12);
PlotLog.r = States.Data(:,13);
% Coefficients
PlotLog.CT1 = States.Data(:,14);
PlotLog.CT2 = States.Data(:,15);
PlotLog.CT3 = States.Data(:,16);
PlotLog.CT4 = States.Data(:,17);

PlotLog.T1 =  arrayfun(Params.BladeThrust,States.Data(:,14));
PlotLog.T2 =  arrayfun(Params.BladeThrust,States.Data(:,15));
PlotLog.T3 =  arrayfun(Params.BladeThrust,States.Data(:,16));
PlotLog.T4 =  arrayfun(Params.BladeThrust,States.Data(:,17));

%eulZYX = quat2angle([PlotLog.qw PlotLog.qx PlotLog.qy PlotLog.qz ],'ZYX');
%[PlotLog.roll, PlotLog.pitch, PlotLog.yaw] = quat2angle([PlotLog.qw PlotLog.qx PlotLog.qy PlotLog.qz ],'XYZ');
[PlotLog.yaw PlotLog.pitch PlotLog.roll] = quat2angle([PlotLog.qw PlotLog.qx PlotLog.qy PlotLog.qz ])


% subplot(3,2,1)
% plot(States.Time ,PlotLog.px,States.Time ,PlotLog.py,States.Time ,PlotLog.pz);
% legend('x','y','z');
% xlabel('Time (sec)')
% ylabel('Linear Disp(m)')
% grid on;

% subplot(3,2,2)
% plot(States.Time ,PlotLog.ub,States.Time ,PlotLog.vb,States.Time ,PlotLog.wb);
% legend('u','v','w');
% xlabel('Time (sec)');
% ylabel('Linear Vel(m/s)');
% grid on;

subplot(2,2,1)
plot(States.Time, PlotLog.qw, States.Time, PlotLog.qx, States.Time, PlotLog.qy, States.Time, PlotLog.qz);
legend('qw','qx','qy','qz'); 
xlabel('Time (sec)')
ylabel('Quaternion')
grid on;
ylim([-1.5 1.5])

subplot(2,2,2)
plot(States.Time, PlotLog.p, States.Time, PlotLog.q, States.Time, PlotLog.r);
legend('p','q','r');
xlabel('Time (sec)')
ylabel('Angular Vel(rad/s)')
grid on;

subplot(2,2,3)
plot(States.Time,PlotLog.roll*(180/pi()),States.Time, PlotLog.pitch*(180/pi()),States.Time, PlotLog.yaw*(180/pi()));
legend('Roll','Pitch','Yaw');
xlabel('Time (sec)')
ylabel('Angle (degrees)')
grid on;

subplot(2,2,4)
plot(States.Time ,PlotLog.CT1,States.Time ,PlotLog.CT2,States.Time ,PlotLog.CT3,States.Time ,PlotLog.CT4);
legend('CT1','CT2','CT3','CT4');
xlabel('Time (sec)')
ylabel('Thrust Coefficient')
grid on;

% subplot(4,2,6)
% plot(outputs.Time ,PlotLog.ldot,outputs.Time ,PlotLog.mdot,outputs.Time ,PlotLog.ndot);
% legend('ldot','mdot','ndot');
% 
% subplot(4,2,7)
% plot(outputs.Time ,PlotLog.Thrust,outputs.Time ,PlotLog.Tdot);
% legend('Thrust','Tdot');

% figure(2) 
% 
% PlotLog.Desired_Thrust = DesiredOut.Data(:,1);
% PlotLog.Desired_Phi    = DesiredOut.Data(:,2);
% PlotLog.Desired_Theta  = DesiredOut.Data(:,3);
% PlotLog.Desired_Yaw    = DesiredOut.Data(:,4);
% 
% subplot(3,1,1)
% plot(States.Time ,PlotLog.phi,States.Time ,PlotLog.theta,States.Time ,PlotLog.psi);
% legend('phi','theta','psi');
% 
% subplot(3,1,2)
% plot(DesiredOut.Time,PlotLog.Desired_Phi, DesiredOut.Time, PlotLog.Desired_Theta, DesiredOut.Time, PlotLog.Desired_Yaw);
% legend('Desired_Phi','Desired_Theta','Desired_Yaw');
% 
% subplot(3,1,3)
% plot(DesiredOut.Time ,PlotLog.Desired_Thrust);
% legend('Desired_Thrust');
% 

% PlotLog.Desiredu1 = DesiredOut.Data(:,1);
% PlotLog.Desiredu2 = DesiredOut.Data(:,2);
% PlotLog.Desiredu3 = DesiredOut.Data(:,3);
% 
% input = [DesiredOut.Data(:,1), DesiredOut.Data(:,2), DesiredOut.Data(:,3)]
% 
% for i = 1:size(input,1)
%     feedbackacceleration = input(1,:)';
%     forceInertial        = Params.m * (feedbackacceleration + [0; 0; Params.g]);
%     ForceBarInertial     = forceInertial/norm(forceInertial);
%     forceT(i)              = norm(forceInertial);
% end
% PlotLog.DesireduNorm =  forceT;
% T = 1:size(input,1);
% plot(PlotLog.DesireduNorm,T , PlotLog.Desiredu3,T , PlotLog.Desiredu2,T , PlotLog.Desiredu3,T)


figure(3)

PlotLog.Desiredx = Control_Command.Data(:,1);
PlotLog.Desiredy = Control_Command.Data(:,2);
PlotLog.Desiredz = Control_Command.Data(:,3);

hold on
grid on;
title('Aggressive Trajectory following with LQR and Differential Flatness')
plot3(PlotLog.px,PlotLog.py,PlotLog.pz,'--r')
plot3(PlotLog.Desiredx,PlotLog.Desiredy,PlotLog.Desiredz,'b')
legend('Actual','Desired')
xlabel('X position (m)') % x-axis label
ylabel('Y position (m)') % y-axis label
zlabel('Z position (m)') % y-axis label
hold off;
