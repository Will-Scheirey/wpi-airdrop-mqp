clear; clc; close all;

q_up = quaternion([cos(pi/4), 0, -sin(pi/4), 0]);

x0 = [
    0.001;
    0.001;
    0.001;

    0;
    0;
    0;

    0;
    0;
    0;

    1;
    0;
    0;
    0;

    1000;
    0;
    0;

    0;
    0;
];

[t, y] = ode45(@(t, y) dynamic_model(t, y), 0:0.03:50, x0);

%% Plotting

plot_pos = true;

figure(1)
clf
numsteps = height(y);


if plot_pos
    patch = poseplot(quaternion(y(1, 10), y(1, 11), y(1, 12), y(1, 13)));

    patch.ScaleFactor = 50;
    xlabel("X")
    ylabel("Y")
    zlabel("Z")

    for i = 2:8:numsteps
        quat = quaternion(y(i, 10), y(i, 11), y(i, 12), y(i, 13));
        pos = [y(i, 15), y(i, 16), y(i, 14)];
    
        set(patch, Orientation=quat, Position=pos); hold on
        plot3(y(i, 15), y(i, 16), y(i,14), '.b', 'MarkerSize', 10); hold on

        xlim([-200, 400]);
        ylim([-400, 200]);
        zlim([0, 1200]);

        set(gca,'ZDir','normal')  
        drawnow
        pause(0.1)
    end

else
    for i = 1:numsteps
        quat = quaternion(y(i, 10), y(i, 11), y(i, 12), y(i, 13));
    
        poseplot(quat);
    
        drawnow
        pause(0.1)
    end
end
hold on
%{
plot(t, y(:, 14), 'DisplayName', 'X'); hold on;
plot(t, y(:, 15), 'DisplayName', 'Y');
plot(t, y(:, 16), 'DisplayName', 'Z');

legend;
title("Body Frame Velocity vs. Time");
%}
%{
for i = 1:4:numsteps
    plot3(y(i, 15), y(i, 16), y(i,14), '.b', 'MarkerSize', 10); hold on
    axis square
    axis equal
    drawnow
end
xlabel("X")
ylabel("Y")
zlabel("Z")
%}