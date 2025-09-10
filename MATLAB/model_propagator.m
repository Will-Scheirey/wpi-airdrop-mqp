clear; clc; close all;
addpath("Utils");

% ==========================
% --- Initial Conditions ---
% ==========================

initial_angle = pi/6;
quat = quaternion([cos(initial_angle), 0, -sin(initial_angle), 0]);

q0   = compact(quat)';   % Quaternion parts
V_b0 = [0; 0; 0.01];        % Body velocities    [m   s^-1]
w_b0 = [0; 0.5; 0.1]; % Body angular rates [rad s^-1]
P0   = [0; 0; 1000];     % ECEF Position      [m]

x0   = [
    V_b0;

    w_b0;

    q0;

    P0;
    0;
    0
    ];

[t, y] = ode89(@(t, y) dynamic_model_new(t, y, P0), 0:0.1:50, x0);

%% Plotting

plot_pos = true;

figure(1)
clf
numsteps = height(y);


if plot_pos
    patch = poseplot(quaternion(y(1, 7), y(1, 8), y(1, 9), y(1, 10)));

    patch.ScaleFactor = 50;
    xlabel("X")
    ylabel("Y")
    zlabel("Z")

    for i = 2:10:numsteps
        quat = quaternion(y(i, 7), y(i, 8), y(i, 9), y(i, 10));
        pos = [y(i, 11), y(i, 12), y(i, 13)];
    
        set(patch, Orientation=quat, Position=pos); hold on
        plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 10); hold on

        xlim([-200, 400]*2);
        ylim([-400, 200]*2);
        zlim([0, 1200]);

        set(gca,'ZDir','normal')  
        title(sprintf("t = %0.2f", t(i)))
        drawnow
        pause(0.05)
    end

else
    for i = 2:20:numsteps
        quat = quaternion(y(i, 10), y(i, 11), y(i, 12), y(i, 13));
    
        poseplot(quat);
    
        drawnow
        pause(0.1)
    end
end

figure(2)
plot(t, y(:, 1), 'DisplayName', 'X'); hold on;
plot(t, y(:, 2), 'DisplayName', 'Y');
plot(t, y(:, 3), 'DisplayName', 'Z');
plot(t, vecnorm(y(:,1:3), 2, 2), 'LineWidth', 3, 'DisplayName', 'Speed')

legend;
title("Body Frame Velocity vs. Time");
