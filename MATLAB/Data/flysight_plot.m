clearvars -except data; clc; close all

[~, ~, ~, ~, ~, ~, data, data_gps] = get_flysight_data("Data/Drop1/SENSOR.CSV", "Data/Drop1/TRACK.CSV", false);


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

uif = uifigure;
g = geoglobe(uif);
geoplot3(g,data_gps.GNSS.lat,data_gps.GNSS.lon,data_gps.GNSS.hMSL,"r", "LineWidth", 3)

% plot(imu., vecnorm([data_gps.GNSS.velN, data_gps.GNSS.velE], 2, 2))

