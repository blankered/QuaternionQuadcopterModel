% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function h = visualize_test_dp_vec(States, Control_Command, Vector)
    % Create a figure with three parts. One part is for a 3D visualization,
    % and the other two are for running graphs of angular velocity and displacement.
    %gif = figure; 
    
    
    plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
    subplot(plots(1));
        
    data.desiredPos = Control_Command.Data(:,1:3)
   
    for i = 1:length(Vector.Data(:,1))
        force.Inertial       = 1.24 * (Vector.Data(i,1:3) + [0, 0, 9.8]);
        ForceBar.Inertial    = force.Inertial/norm(force.Inertial);
        data.vector(i,:) = ForceBar.Inertial ;
     end
     
    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t thrusts] = quadcopter;

    state.px = States.Data(:,1)
    state.py = States.Data(:,2);
    state.pz = States.Data(:,3);
    
    state.ub = States.Data(:,4);
    state.vb = States.Data(:,5);
    state.wb = States.Data(:,6);
    
    state.qw = States.Data(:,7);
    state.qx = States.Data(:,8);
    state.qy = States.Data(:,9);
    state.qz = States.Data(:,10);
    
    state.p = States.Data(:,11);
    state.q = States.Data(:,12);
    state.r = States.Data(:,13);
    % Coefficients
    state.CT1 = States.Data(:,14);
    state.CT2 = States.Data(:,15);
    state.CT3 = States.Data(:,16);
    state.CT4 = States.Data(:,17);
    state.time = States.Time;

    data.x      = [state.px state.py state.pz];
    data.quat   = [-state.qw state.qx state.qy state.qz];
    data.t   = state.time;
    data.input  = [state.CT1 state.CT2 state.CT3 state.CT4];
    data.angvel = [state.p state.q state.r];
    [data.yaw data.pitch data.roll] = quat2angle(data.quat,'ZYX');
    
    % Set axis scale and labels.
    grid on;
    axis([-10 30 -20 20 5 15]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');

    DesiredTraj = animatedline,
    
    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, DesiredTraj, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, DesiredTraj,  plots)
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
    
    
    for t = 1:2:length(data.t)
        % The first, main part, is for the 3D visualization.
        subplot(plots(1));

        % Compute translation to correct linear position coordinates.
        dx = data.x(t, :);
        vec = data.vector(t,:) + data.x(t, :);
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
        quat   = data.quat(t, :);
        rotate = quat2rotm(quat);
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);

        % Compute scaling for the thrust cylinders. The lengths should represent relative
        % strength of the thrust at each propeller, and this is just a heuristic that seems
        % to give a good visual indication of thrusts.
        scales = exp(data.input(t,:) / min(abs(data.input(t,:))) + 5) - exp(6) +  1.5;
        for i = 1:4
            % Scale each cylinder. For negative scales, we need to flip the cylinder
            % using a rotation, because makehgtform does not understand negative scaling.
            s = scales(i);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(thrusts(i), 'Matrix', move * rotate * scalez);
        end

        VectLine = line([dx(1),vec(1)],[dx(2),vec(2)],[dx(3),vec(3)]);
        addpoints(DesiredTraj,data.desiredPos(t,1),data.desiredPos(t,2),data.desiredPos(t,3));   

        % Update the drawing.      
%         xmin = data.x(t,1)-20;
%         xmax = data.x(t,1)+20;
%         ymin = data.x(t,2)-20; 
%         ymax = data.x(t,2)+20;
%         zmin = data.x(t,3)-5;
%         zmax = data.x(t,3)+5;
        xmin = -4;
        xmax = 4;
        ymin = -4; 
        ymax = 4;
        zmin = -2;
        zmax = 8;
        axis([xmin xmax ymin ymax zmin zmax]);
        drawnow;

        % Use the bottom two parts for angular velocity and displacement.
        subplot(plots(2));
        multiplot(data, [data.angvel], t);
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        title('Angular Velocity');

        [yaw pitch roll] = quat2angle(data.quat);

        subplot(plots(3));
        multiplot(data, [data.yaw, data.pitch, data.roll], t);
        xlabel('Time (s)');
        ylabel('Angle (rad)');
        title('Angular Displacement');
        
        save_gif(gcf,t);
        
    end
end

function save_gif(h,n)
      % Capture the plot as an image 
      filename = 'TestAnimation.gif';
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 

      % Write to the GIF File 
      if n == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.t(1:ind,:);
    Values = values(1:ind,:);

    % Plot in RGB, with different markers for different components.
    plot(times, Values(:,1), 'r-', times, Values(:,2), 'g.', times, Values(:,3), 'b-.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    axis([xmin xmax ymin ymax]);
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-.5,     .5,  0, 1.0, 0.15, 0.5);
    h(2) = prism(-.5,    -.5,  0, 1.0, 0.15, 0.5);
	h(3) = prism(-0.05,  -.5,  0, 0.1, 1.0,  0.5);
    
    % Draw bulbs representing propellers at the end of each arm.
    [x y z] = sphere;
    x = .05 * x;
    y = .05 * y;
    z = .05 * z;
    h(4) = surf(.5+x ,.5+y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(.5+x ,y-.5, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x-.5, .5+y , z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(7) = surf(x-.5, y-.5 , z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x y z] = cylinder(.1, 7);
    thrusts(1) = surf(x-.5, y-.5, z, 'EdgeColor', 'none', 'FaceColor', 'y');
    thrusts(2) = surf(x+.5, y-.5, z, 'EdgeColor', 'none', 'FaceColor', 'y');
    thrusts(3) = surf(x-.5, y+.5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(4) = surf(x+.5, y+.5, z, 'EdgeColor', 'none', 'FaceColor', 'm');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X Y Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); 
        hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X Y Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end


% function RotMat = quat2rotm(q)
% RotMat = [(q(1)^2) +(q(2)^2) - (q(3)^2) - (q(4)^2), 2*q(2)*q(3) + 2*q(1)*q(4), 2*q(2)*q(4)-2*q(1)*q(3)]; ...
%           2*q(2)*q(3)-2*q(1)*q(4), (q(1)^2) - (q(2)^2) + (q(3)^2) - (q(4)^2),  2*q(3)*q(4)+2*q(1)*q(2); ...
%           2*q(2)*q(4)+2*q(1)*q(3), 2*q(3)*q(4)-2*q(1)*q(2),  (q(1)^2) - (q(2)^2) - (q(3)^2) + (q(4)^2)]
% end

% function R = quat2rotm(quat)
% x y z w
% quat = quat/norm(quat);
% x = quat(1);
% y = quat(2);
% z = quat(3);
% w = quat(4);
% R = [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w;
%      2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w;
%      2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y];
% end
