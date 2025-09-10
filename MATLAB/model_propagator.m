clear; clc; close all;
addpath("Utils");

q_up = quaternion([cos(pi/4), 0, -sin(pi/4), 0]);

% q_up = quaternion([1, 0, 0, 0]);

[w,i,j,k] = q_up.parts();

x0 = [
    0;
    0;
    1;

    0;
    0.1;
    0;
    
    w;
    i;
    j;
    k;

    0;
    0;
    1000;

    0;
    0;
];

[t, y] = ode45(@(t, y) dynamic_model(t, y), 0:1:50, x0);

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

    for i = 2:1:numsteps
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
    for i = 2:8:numsteps
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

legend;
title("Body Frame Velocity vs. Time");
