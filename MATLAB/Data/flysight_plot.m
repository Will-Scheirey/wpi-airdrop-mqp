clear; clc; close all

data = load_flysight_file("SENSOR.csv");

imu = data.IMU;

imu.time = imu.time - imu.time(1);

ax_norm = sqrt(imu.ax .^ 2 + imu.ay .^ 2 + imu.az .^ 2);

figure(1)
plot(imu.time, imu.ax, '-', 'DisplayName', 'A_x', 'LineWidth', 1); hold on
plot(imu.time, imu.ay, '-', 'DisplayName', 'A_y', 'LineWidth', 1); hold on
plot(imu.time, imu.az, '-', 'DisplayName', 'A_z', 'LineWidth', 1); hold on
plot(imu.time, ax_norm, '-', 'DisplayName', 'Norm', 'LineWidth', 2);

legend
xlabel("Time (s)")
ylabel("Acceleration (g)")
title("Flysight Test Data")