clear; clc; close all;
addpath("Utils");

% ==========================
% --- Initial Conditions ---
% ==========================

initial_angle = pi/4;
quat = quaternion([cos(initial_angle), 0, -sin(initial_angle), 0]);

q0   = compact(quat)';  % Quaternion parts
V_b0 = [0; 0; 0];       % Body velocities    [m   s^-1]
w_b0 = [0; 0; 0];     % Body angular rates [rad s^-1]
P0   = [0; 0; 1000];    % ECEF Position      [m]

x0   = [
    V_b0;

    w_b0;

    q0;

    P0 + [0; 0; -3];
    ];

[t, y] = ode113(@(t, y) dynamic_model_1(t, y, P0), 0:0.1:20, x0);

%% Plotting

plot_traj(t, y)

function plot_traj(t, y)
%{
figure(1)
clf
numsteps = height(y);

patch = poseplot(quaternion(y(1, 7), y(1, 8), y(1, 9), y(1, 10)));

% patch.ScaleFactor = 50;
xlabel("X")
ylabel("Y")
zlabel("Z")

for i = 2:10:numsteps
    quat = quaternion(y(i, 7), y(i, 8), y(i, 9), y(i, 10));
    pos = [y(i, 11), y(i, 12), y(i, 13)];

    set(patch, Orientation=quat, Position=pos); hold on
    plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 10); hold on

    xlim([-10, 10]*2);
    ylim([-10, 10]*2);
    zlim([990, 1005]);

    set(gca,'ZDir','normal')  
    title(sprintf("t = %0.2f", t(i)))
    drawnow
    pause(0.05)
end
%}
figure(2)
clf
plot(t, y(:, 1), 'DisplayName', 'X'); hold on;
plot(t, y(:, 2), 'DisplayName', 'Y');
plot(t, y(:, 3), 'DisplayName', 'Z');
plot(t, vecnorm(y(:,1:3), 2, 2), 'LineWidth', 3, 'DisplayName', 'Speed')

legend;
title("Body Frame Velocity vs. Time");

figure(3)
clf
plot(t, y(:, 13))
end
