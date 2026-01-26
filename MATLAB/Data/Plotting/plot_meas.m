function fig_idx = plot_meas(data_gps, data_accel, data_gyro, data_mag, data_baro, data_gps_vel, fig_idx, tspan)

if nargin < 8
    tspan = data_accel.time;
end

if nargin < 7
    fig_idx = 1;
end

if ~isempty(data_gps)
gps_time_range = all([data_gps.time > tspan(1), data_gps.time < tspan(end)], 2);

figure(fig_idx); fig_idx = fig_idx + 1;
clf
yyaxis left
plot(data_gps.time, data_gps.data(:, 1), 'DisplayName', '0'); hold on
plot(data_gps.time, data_gps.data(:, 2), 'DisplayName', '1')
ylabel("East, North (m)")
yyaxis right
plot(data_gps.time, data_gps.data(:, 3), 'DisplayName', '2')
ylabel("Up (m)")
legend
title("GPS")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot3(data_gps.data(gps_time_range, 1), data_gps.data(gps_time_range, 2), data_gps.data(gps_time_range, 3), 'DisplayName', 'Traj'); hold on
legend
title("GPS")
axis equal
end

figure(fig_idx); fig_idx = fig_idx + 1;
clf
subplot(2,2,1)
plot(data_accel.time, data_accel.data(:, 1));
title('0')
subplot(2,2,2)
plot(data_accel.time, data_accel.data(:, 2));
title('1')
subplot(2,2,3)
plot(data_accel.time, data_accel.data(:, 3));
title('2')
subplot(2,2,4)
plot(data_accel.time, vecnorm(data_accel.data, 2, 2));
title('Norm')
sgtitle("Accel")
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

data_mag.data = data_mag.data ./ vecnorm(data_mag.data, 2, 2);

plot(data_mag.time, data_mag.data(:, 1), 'DisplayName', '0'); hold on
plot(data_mag.time, data_mag.data(:, 2), 'DisplayName', '1')
plot(data_mag.time, data_mag.data(:, 3), 'DisplayName', '2')
legend
title("Mag")
xlim([tspan(1), tspan(end)])

figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_baro.time, data_baro.data(:, 1), 'DisplayName', 'Baro', 'LineWidth', 1); hold on
if ~isempty(data_gps)
plot(data_gps.time, data_gps.data(:, 3), 'DisplayName', 'GPS', 'LineWidth', 1); hold on
end
legend
xlim([tspan(1), tspan(end)])
xlabel("Time (s)")
ylabel("Altitude Measurement (m)")
title("Baro and GPS")

if ~isempty(data_gps)
figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_gps_vel.time, data_gps_vel.data(:, 1), 'DisplayName', 'Vel X', 'LineWidth', 1); hold on
plot(data_gps_vel.time, data_gps_vel.data(:, 2), 'DisplayName', 'Vel Y', 'LineWidth', 1); hold on
plot(data_gps_vel.time, data_gps_vel.data(:, 3), 'DisplayName', 'Vel Z', 'LineWidth', 1); hold on
legend
xlim([tspan(1), tspan(end)])
xlabel("Time (s)")
ylabel("Velocity Measurement (m/s)")
title("GPS Velocity")


movmean_window = 50;
horizontal_vel = vecnorm(data_gps_vel.data(:, 1:2), 2, 2);
figure(fig_idx); fig_idx = fig_idx + 1;
clf
plot(data_gps_vel.time, movmean(horizontal_vel, movmean_window), 'DisplayName', 'Horizontal Velocity', 'LineWidth', 1); hold on
plot(data_gps_vel.time, movmean(data_gps_vel.data(:, 3), movmean_window), 'DisplayName', 'Vertical Velocity', 'LineWidth', 1); hold on
legend
xlim([tspan(1), tspan(end)])
xlabel("Time (s)")
ylabel("Velocity Measurement (m/s)")
title("GPS Velocity")
end
% plot3(data_mag.data(1,:))
end