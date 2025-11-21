clear; clc; close all
data = load("datagen_dropstage1_particle_1758814770.7211.csv");

timestamp = data(:, 1);
x = data(:, 2);
y = data(:, 3);
speed = data(:, 4);

%{
z = -cumtrapz(speed) .* timestamp;

dx_dt = diff(x) ./ diff(timestamp);
dy_dt = diff(y) ./ diff(timestamp);

v = vecnorm([dx_dt, dy_dt], 2, 2);
%}
% plot(x, y, 'DisplayName', 'Position'); hold on
% plot(timestamp, y, 'DisplayName', 'Position Y')

% plot3(x, y, z)

% 
% plot(timestamp, speed, 'DisplayName', 'Given Speed'); hold on
% plot(timestamp(1:end-1), v, 'DisplayName', 'Calculated Speed')

subplot(3,1,1)
plot(timestamp, x, 'LineWidth', 1.5);
title("State 1");
ylabel("Magnitude")

subplot(3,1,2)
plot(timestamp, y, 'LineWidth', 1.5);
title("State 2");
ylabel("Magnitude")

subplot(3,1,3)
plot(timestamp, speed, 'LineWidth', 1.5);
title("State 3");
xlabel("Time")
ylabel("Magnitude")

sgtitle("States vs. Time")