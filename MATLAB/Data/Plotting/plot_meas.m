function fig_idx = plot_meas(data_gps, data_accel, data_gyro, data_mag, data_baro, tspan, fig_idx)

if nargin < 6
    tspan = data_accel.time;
end

if nargin < 7
    fig_idx = 1;
end

gps_time_range = all([data_gps.time > tspan(1), data_gps.time < tspan(end)], 2);

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_gps.time, data_gps.data(:, 1), 'DisplayName', '0'); hold on
plot(data_gps.time, data_gps.data(:, 2), 'DisplayName', '1')
plot(data_gps.time, data_gps.data(:, 3), 'DisplayName', '2')
legend
title("GPS")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot3(data_gps.data(gps_time_range, 1), data_gps.data(gps_time_range, 2), data_gps.data(gps_time_range, 3), 'DisplayName', 'Traj'); hold on
legend
title("GPS")

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_accel.time, data_accel.data(:, 1), 'DisplayName', '0'); hold on
plot(data_accel.time, data_accel.data(:, 2), 'DisplayName', '1')
plot(data_accel.time, data_accel.data(:, 3), 'DisplayName', '2')
legend
title("Accel")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_gyro.time, data_gyro.data(:, 1), 'DisplayName', '0'); hold on
plot(data_gyro.time, data_gyro.data(:, 2), 'DisplayName', '1')
plot(data_gyro.time, data_gyro.data(:, 3), 'DisplayName', '2')
legend
title("Gyro")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_mag.time, data_mag.data(:, 1), 'DisplayName', '0'); hold on
plot(data_mag.time, data_mag.data(:, 2), 'DisplayName', '1')
plot(data_mag.time, data_mag.data(:, 3), 'DisplayName', '2')
legend
title("Mag")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_baro.time, data_baro.data(:, 1), 'DisplayName', 'Baro', 'LineWidth', 1); hold on
plot(data_gps.time, data_gps.data(:, 3), 'DisplayName', 'GPS', 'LineWidth', 1); hold on
legend
xlim([tspan(1), tspan(end)])
xlabel("Time (s)")
ylabel("Altitude Measurement (m)")
title("Baro and GPS")
end