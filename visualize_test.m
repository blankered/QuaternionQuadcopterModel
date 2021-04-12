% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function h = visualize_test(uu)
    % Create a figure with three parts. One part is for a 3D visualization,
    % and the other two are for running graphs of angular velocity and displacement.
    figure(1);
    plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
    subplot(plots(1));
    %pause;

    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t thrusts] = quadcopter;

    % Set axis scale and labels.
    axis([-10 30 -20 20 5 15]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');
    
    state.px  = uu(1);
    state.py  = uu(2);
    state.pz  = uu(3);
    % Velocity
    state.ui  = uu(4);
    state.vi  = uu(5);
    state.wi  = uu(6);
    % Quaternion Pose
    state.qw  = uu(7);
    state.qx  = uu(8);
    state.qy  = uu(9);
    state.qz  = uu(10);
    % Angular Velocity
    state.p  = uu(11);
    state.q  = uu(12);
    state.r  = uu(13);
    % Thrust Coefficients
    state.CT1  = uu(14);
    state.CT2  = uu(15);
    state.CT3  = uu(16);
    state.CT4  = uu(17);
    state.time = uu(18);

    data.x      = [state.px state.py state.pz];
    data.quat   = [state.qw state.qx state.qy state.qz];
    data.time   = state.time;
    data.input  = [state.CT1 state.CT2 state.CT3 state.CT4];
    data.angvel = [state.p state.q state.r];
    
    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, plots)
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
    %for t = 1:2:length(data.t)
        % The first, main part, is for the 3D visualization.
        subplot(plots(1));

        % Compute translation to correct linear position coordinates.
        dx = data.x;
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
        quat   = data.quat;
        rotate = quat2rotm(quat);
        %angles = data.theta;
        %rotate = rotation(angles);
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);

        % Compute scaling for the thrust cylinders. The lengths should represent relative
        % strength of the thrust at each propeller, and this is just a heuristic that seems
        % to give a good visual indication of thrusts.
        scales = exp(data.input / min(abs(data.input)) + 5) - exp(6) +  1.5;
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

        % Update the drawing.      
        xmin = data.x(1)-20;
        xmax = data.x(1)+20;
        ymin = data.x(2)-20;
        ymax = data.x(2)+20;
        zmin = data.x(3)-5;
        zmax = data.x(3)+5;
        axis([xmin xmax ymin ymax zmin zmax]);
        drawnow;

        % Use the bottom two parts for angular velocity and displacement.
        subplot(plots(2));
        multiplot(data, data.angvel, data.time);
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        title('Angular Velocity');

        subplot(plots(3));
        multiplot(data, data.quat, data.time);
        xlabel('Time (s)');
        ylabel('Angular Displacement (rad)');
        title('Angular Displacement');
    %end
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.time;

    % Plot in RGB, with different markers for different components.
    if length(values) > 3
        plot(times, values(1), 'r-', times, values(2), 'g-', times, values(3), 'b-', times, values(4), 'm-');
    else
        plot(times, values(1), 'r-', times, values(2), 'g-', times, values(3), 'b-');
    end
    
    % Set axes to remain constant throughout plotting.
    %xmin = min(data.time);
    xmin = 0;
    xmax = max(data.time) + 1;
    ymin = 1.1 * min(min(values)) - 1;
    ymax = 1.1 * max(max(values)) + 1;
    axis([xmin xmax ymin ymax]);
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-5, -0.25, -0.25, 10, 0.5, 0.5);
    h(2) = prism(-0.25, -5, -0.25, 0.5, 10, 0.5);

    % Draw bulbs representing propellers at the end of each arm.
    [x y z] = sphere;
    x = 0.5 * x;
    y = 0.5 * y;
    z = 0.5 * z;
    h(3) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x y z] = cylinder(0.1, 7);
    thrusts(1) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(2) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');
    thrusts(3) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(4) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');

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
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
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
