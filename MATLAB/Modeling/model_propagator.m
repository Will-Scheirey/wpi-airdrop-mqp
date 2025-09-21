clear; clc; close all;
addpath("Dynamic_Models", "Kinematics", "Parachute_Utils");

% ==========================
% --- Initial Conditions ---
% ==========================

initial_angle = pi/4;
quat = quaternion([cos(initial_angle), 0, -sin(initial_angle), 0]);

q0   = compact(quat)';  % Quaternion parts
V_b0 = [0; 3; 5];       % Body velocities    [m   s^-1]
w_b0 = [1; 0; 0];     % Body angular rates [rad s^-1]
P0   = [0; 0; 0];    % ECEF Position      [m]

x0   = [
    V_b0;

    w_b0;

    q0;

    P0 + [3; -1; -3];
    ];

[t, y] = ode45(@(t, y) spring_pendulum_dynamic_model(t, y, P0), 0:0.05:100, x0);

%% Plotting

plot_data(t, y, false, false)

function plot_data(t, y, do_animation, save_video)

if do_animation
figure(1)
clf
run_animation(t, y, 3, 1, save_video)
end

figure(2)
clf
plot(t, y(:, 1), 'DisplayName', 'X', 'LineWidth', 1.5); hold on;
plot(t, y(:, 2), 'DisplayName', 'Y', 'LineWidth', 1.5);
plot(t, y(:, 3), 'DisplayName', 'Z', 'LineWidth', 1.5);
plot(t, vecnorm(y(:,1:3), 2, 2), 'LineWidth', 3, 'DisplayName', 'Speed')

legend;
title("Body Frame Velocity vs. Time");
xlabel("Time (s)")
ylabel("Velocity (m/s)");

figure(3)
clf
yyaxis left
plot(t, y(:, 11), 'DisplayName', 'X', 'LineWidth', 1.5); hold on
plot(t, y(:, 12), 'DisplayName', 'Y', 'LineWidth', 1.5)
ylabel("X, Y Position (m)")

yyaxis right
plot(t, y(:,13), 'DisplayName', 'Z', 'LineWidth', 1.5)
ylabel("Z Position (m)")

title("ECEF Position vs. Time")
xlabel("Time (s)")
legend

figure(4)
clf
plot3(y(:, 11), y(:, 12), y(:, 13))

title("ECEF Trajectory")
xlabel("X")
ylabel("Y")
zlabel("Z")

figure(5)
clf
plot(t, y(:, 4), 'DisplayName', 'P', 'LineWidth', 1.5); hold on;
plot(t, y(:, 5), 'DisplayName', 'Q', 'LineWidth', 1.5)
plot(t, y(:, 6), 'DisplayName', 'R', 'LineWidth', 1.5)

xlabel("Time (s)")
ylabel("Angular Rate (rad/s)")
title("Angular Velocities vs. Time")
legend

end

function run_animation(t, y, step, substep, save_video)
numsteps = height(y);

patch = poseplot(quaternion(y(1, 7), y(1, 8), y(1, 9), y(1, 10)));

% patch.ScaleFactor = 50;
xlabel("X")
ylabel("Y")
zlabel("Z")

if save_video
outputVideo = VideoWriter('myVideo.mp4', 'MPEG-4'); % Specify filename and format
open(outputVideo);
end

for i = 2:step:numsteps - step
    for j=i:substep:i+step
        quat = quaternion(y(j, 7), y(j, 8), y(j, 9), y(j, 10));
        pos = [y(j, 11), y(j, 12), y(j, 13)];
    
        set(patch, Orientation=quat, Position=pos); hold on
        plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 1); hold on
    end

    xlim([-1, 1]*10);
    ylim([-1, 1]*10);
    zlim([-5, 1]*5);

    set(gca,'ZDir','normal')  
    title(sprintf("t = %0.2f", t(i)))
    drawnow

    if save_video
    frame = getframe(gcf); % captures the current figure (gcf)
    
    writeVideo(outputVideo, frame);
    else
    % pause(0.05)
    end
end
if save_video
close(outputVideo)
end
end
