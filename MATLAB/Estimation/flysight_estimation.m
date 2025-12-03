clearvars -except data; clc;

%% Load Flysight Data
%{
% if ~exist("data", "var")
    data = load_flysight_file("Data/SENSOR.CSV");
% end

min_t = min(min(data.IMU.time), min(data.MAG.time));

data.IMU.time = data.IMU.time - min_t;
data.MAG.time = data.MAG.time - min_t;

dt_min_imu = min(diff(data.IMU.time));
dt_min_mag = min(diff(data.MAG.time));

dt        = min(dt_min_imu, dt_min_mag);
tspan     = data.IMU.time(1) : dt : data.IMU.time(end);
meas_freq = 1 / dt;

meas_idx_accel = repmat(-1, length(data.IMU .time), 1);
meas_idx_mag   = repmat(2, length(data.MAG .time), 1);
meas_idx_gyro  = repmat(3, length(data.IMU.time), 1);

accel_meas = table([data.IMU.ax, data.IMU.ay, data.IMU.az] * 9.81, data.IMU.time, meas_idx_accel, 'VariableNames', {'data', 'timestamp', 'meas_idx'});
mag_meas   = table([data.MAG.x, data.MAG.y, data.MAG.z],    data.MAG.time, meas_idx_mag,   'VariableNames', {'data', 'timestamp', 'meas_idx'});
gyro_meas  = table(deg2rad([data.IMU.wx, data.IMU.wy, data.IMU.wz]), data.IMU.time, meas_idx_gyro,  'VariableNames', {'data', 'timestamp', 'meas_idx'});

measurements = {mag_meas, gyro_meas};
inputs = accel_meas;
%}
%% Load HPRC Data
clear; clc;
% [data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data();

[data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data_2();
% 
% plot(data_baro.time, data_baro.data)
% return

data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);
data_gps.meas_idx   = repmat(1, length(data_gps.time), 1);
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);
data_gyro.meas_idx  = repmat(3, length(data_gyro.time), 1);
data_baro.meas_idx  = repmat(4, length(data_baro.time), 1);

the_mode = "movmedian";
window = 3;
mag_to_remove = any([isoutlier(data_mag.data(:, 1), the_mode, window), isoutlier(data_mag.data(:, 2), the_mode, window), isoutlier(data_mag.data(:, 3), the_mode, window)], 2);
data_mag = data_mag(~mag_to_remove, :);

the_mode = "movmedian";
window = 100;
accel_to_remove = any([isoutlier(data_accel.data(:, 1), the_mode, window), isoutlier(data_accel.data(:, 2), the_mode, window), isoutlier(data_accel.data(:, 3), the_mode, window)], 2);
data_accel = data_accel(~accel_to_remove, :);

measurements = {data_gps, data_mag, data_gyro, data_baro};
inputs = data_accel;

dt_min_accel = min(diff(data_accel.time));
dt_min_gps   = min(diff(data_gps.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

dt        = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]);
tspan     = data_accel.time(1) : dt : data_accel.time(end);
% tspan = 0 : dt : 25;
meas_freq = 1 / dt;

%% Set up the Kalman Filter
num_steps = numel(tspan);

x0 = [
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
    
    0;
    0;
    0;

    0;
    0;
    0;

    0;
    0;
    0;

    0;
    0;
    0;
];

Rb = 5;

Rp = 5;
R_pos = [
    Rp, 0,   0;
    0,   Rp, 0;
    0,   0,   Rb
    ].^2;

Rq = 1e-1;
R_quat = [
    Rq, 0,  0,  0;
    0,  Rq, 0,  0;
    0,  0,  Rq, 0;
    0,  0,  0,  Rq
    ].^2;

Rw = 1e-1;
R_w = [
    Rw,  0,  0;
    0,   Rw, 0;
    0,   0,  Rw
].^2;

R_baro = Rb^2;

R = blkdiag( ...
    R_pos, ...
    R_quat, ...
    R_w,  ...
    R_baro ...
    );

Q_P = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 1;
] * 1e1;

cross_term = 1e-4;
diag_term = 1e1;

Q_V = [
    diag_term,           cross_term,  cross_term;
    cross_term,    diag_term,        cross_term
    cross_term,    cross_term,  diag_term
];

Qe = 1e-2;
Q_e = [
    Qe,  0,  0,  0;
    0,   Qe, 0,  0;
    0,   0,  Qe, 0;
    0,   0,  0,  Qe
].^2;

Qw = 1e-3;
Q_w = [
    Qw, 0,  0;
    0,  Qw, 0;
    0,  0,  Qw
].^2;

Qwb = 1e-3;
Q_wb = [
 Qwb, 0,   0;
 0,   Qwb, 0;
 0,   0,   Qwb
].^2;

Qab = 1e-3;
Q_ab = [
 Qab, 0,   0;
 0,   Qab, 0;
 0,   0,   Qab
].^2;

Qpb = 1e-8;
Q_pb = [
 Qpb, 0,   0;
 0,   Qpb, 0;
 0,   0,   1e-10
].^2;

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    Q_e, ... % e
    Q_w, ... % w
    Q_wb, ... % wb
    Q_ab, ... % ab
    Q_pb ...
    );

P0 = blkdiag( ...
    1e1 * eye(3), ...
    1e1 * eye(3), ...
    1e1 * eye(4), ...
    1e1 * eye(3), ...
    1e4 * eye(3), ...
    1e4 * eye(3), ...
    1e2 * eye(3) ...
    );

%% Run the Kalman Filter
flysight_box = Box(0.1524, 0.1524, 3.6576, 25);
% flysight_box = Box(0.1524, 0.1524, 0.4, 2);

kf = EKF_Varying_Measurements(R, Q, x0, 0, P0, dt, flysight_box.I());

kf.run_filter(measurements, inputs, tspan);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_est = x_est(:, 1:3);
v_est = x_est(:, 4:6);
e_est = x_est(:, 7:10);
w_est = x_est(:, 11:13);

t_plot = tspan(1:end-1);

%% Plot Values
figure(1)
clf
plot(t_plot, p_est, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Position (m)")
title("Position vs. Time")

hold on;
% plot(data_gps.time, data_gps.data(:, 3), 'LineWidth', 2, 'DisplayName', 'GPS Alt');
% plot(data_baro.time, data_baro.data, 'LineWidth', 2, 'DisplayName', 'Baro Alt')

figure(2)
clf
plot(t_plot, v_est, 'LineWidth', 2)
legend("V_0^E", "V_1^E", "V_2^E")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("ECEF Velocity vs. Time")

figure(3)
clf
plot(t_plot, e_est, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Value")
title("Quaternion vs. Time")

figure(4)
clf
plot(t_plot, w_est, 'LineWidth', 2)
legend("w_0^B", "w_1^B", "w_2^B")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("Body Angular Velocity vs. Time")

figure(6)
clf

subplot(3, 1, 1)
plot(t_plot, x_est(:, 14), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias (rad/s)")
sgtitle("Gyro Axis 0")

subplot(3, 1, 2)
plot(t_plot, x_est(:, 15), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on

legend
xlabel("Time (s)")
ylabel("Bias (rad/s)")
sgtitle("Gyro Axis 1")

subplot(3, 1, 3)
plot(t_plot, x_est(:, 16), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Estimate (rad/s)")
sgtitle("Gyro Axis 2")

sgtitle("Gyro Bias Estimates")

figure(7)
clf
subplot(3,1,1)
plot(t_plot, p_est(:,1), 'LineWidth', 2); hold on;
plot(data_gps.time, data_gps.data(:, 1), '.', 'MarkerSize', 10);
legend("Estimated", "Measurement")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_1^E vs. Time")
xlim([t_plot(1), t_plot(end)])

subplot(3,1,2)
plot(t_plot, p_est(:,2), 'LineWidth', 2); hold on;
plot(data_gps.time, data_gps.data(:, 2), '.', 'MarkerSize', 10);
legend("Estimated", "Measurement")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_2^E vs. Time")
xlim([t_plot(1), t_plot(end)])

subplot(3,1,3)
plot(t_plot, p_est(:,3), 'LineWidth', 2); hold on;
plot(data_gps.time, data_gps.data(:, 3), '.', 'MarkerSize', 10);
legend("Estimated", "Measurement")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_3^E vs. Time")
xlim([t_plot(1), t_plot(end)])

figure(8)
clf
plot(t_plot, kf.accel_calc_all(1:end-1, 1), 'LineWidth', 1.5, 'DisplayName', 'a_0^E'); hold on
plot(t_plot, kf.accel_calc_all(1:end-1, 2), 'LineWidth', 1.5, 'DisplayName', 'a_1^E'); hold on
plot(t_plot, kf.accel_calc_all(1:end-1, 3), 'LineWidth', 1.5, 'DisplayName', 'a_2^E'); hold on
xlim([t_plot(1), t_plot(end)])
legend

figure(9)
clf
idx = 1;
plot_cov(kf.P_hist(idx,idx,:))
xlim([t_plot(1), t_plot(end)])

figure(10)
clf
plot(data_mag.time, data_mag.data(:, 1), 'LineWidth', 2, 'DisplayName', 'Mag X'); hold on
plot(data_mag.time, data_mag.data(:, 2), 'LineWidth', 2, 'DisplayName', 'Mag Y'); hold on
plot(data_mag.time, data_mag.data(:, 3), 'LineWidth', 2, 'DisplayName', 'Mag Z'); hold on
legend
xlim([t_plot(1), t_plot(end)])

figure(11)
clf
plot(data_accel.time, data_accel.data(:, 1), 'LineWidth', 2, 'DisplayName', 'Accel X'); hold on
plot(data_accel.time, data_accel.data(:, 2), 'LineWidth', 2, 'DisplayName', 'Accel Y'); hold on
plot(data_accel.time, data_accel.data(:, 3), 'LineWidth', 2, 'DisplayName', 'Accel Z'); hold on
plot(data_accel.time, vecnorm(data_accel.data, 2, 2), 'LineWidth', 2, 'DisplayName', 'Accel Norm'); hold on
legend
xlim([t_plot(1), t_plot(end)])

figure(12)
clf
plot(data_accel.time, vecnorm(data_accel.data, 2, 2), 'LineWidth', 2, 'DisplayName', 'Measured Accel Norm'); hold on
plot(t_plot, vecnorm(kf.accel_calc_all(1:end-1, :), 2, 2), 'LineWidth', 2, 'DisplayName', 'Estimated Accel Norm'); hold on
legend
xlim([t_plot(1), t_plot(end)])


quats = quaternion(e_est(:, 1), e_est(:, 2), e_est(:, 3), e_est(:, 4));
figure(14)
eul_angles = rad2deg(quat2eul(quats));

clf
plot(t_plot, eul_angles(:, 1), 'LineWidth', 2, 'DisplayName', 'Eul 1'); hold on
plot(t_plot, eul_angles(:, 2), 'LineWidth', 2, 'DisplayName', 'Eul 2'); hold on
plot(t_plot, eul_angles(:, 3), 'LineWidth', 2, 'DisplayName', 'Eul 3'); hold on
legend

figure(15)
clf
plot3(p_est(:,1), p_est(:,2), p_est(:,3), 'MarkerSize', 10, 'DisplayName', 'Estimate'); hold on;
plot3(data_gps.data(:,1), data_gps.data(:,2), data_gps.data(:,3), '.', 'MarkerSize', 10, 'DisplayName', 'Measurement'); hold on
legend
axis equal

figure(16);
clf
subplot(3, 1, 1)
plot(t_plot, x_est(:, 17), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias (m/s^2)")
sgtitle("Accel Axis 0")

subplot(3, 1, 2)
plot(t_plot, x_est(:, 18), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on

legend
xlabel("Time (s)")
ylabel("Bias (m/s^2)")
sgtitle("Accel Axis 1")

subplot(3, 1, 3)
plot(t_plot, x_est(:, 19), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Estimate (m/s^2)")
sgtitle("Accel Axis 2")

sgtitle("Accel Bias Estimates")

figure(17);
clf
subplot(3, 1, 1)
plot(t_plot, x_est(:, 20), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias (m)")
sgtitle("Position Axis 0")

subplot(3, 1, 2)
plot(t_plot, x_est(:, 21), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on

legend
xlabel("Time (s)")
ylabel("Bias (m)")
sgtitle("Position Axis 1")

subplot(3, 1, 3)
plot(t_plot, x_est(:, 22), 'DisplayName', 'Bias Estimate', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Estimate (m)")
sgtitle("Position Axis 2")

sgtitle("Position Bias Estimates")

%% Plot Measurements
figure(18)
clf
plot(data_gps.time, data_gps.data(:, 1), 'DisplayName', '0'); hold on
plot(data_gps.time, data_gps.data(:, 2), 'DisplayName', '1')
plot(data_gps.time, data_gps.data(:, 3), 'DisplayName', '1')
legend
title("GPS")
xlim([tspan(1), tspan(end)])

figure(19)
clf
plot(data_accel.time, data_accel.data(:, 1), 'DisplayName', '0'); hold on
plot(data_accel.time, data_accel.data(:, 2), 'DisplayName', '1')
plot(data_accel.time, data_accel.data(:, 3), 'DisplayName', '1')
legend
title("Accel")
xlim([tspan(1), tspan(end)])

figure(20)
clf
plot(data_gyro.time, data_gyro.data(:, 1), 'DisplayName', '0'); hold on
plot(data_gyro.time, data_gyro.data(:, 2), 'DisplayName', '1')
plot(data_gyro.time, data_gyro.data(:, 3), 'DisplayName', '1')
legend
title("Gyro")
xlim([tspan(1), tspan(end)])

figure(21)
clf
plot(data_mag.time, data_mag.data(:, 1), 'DisplayName', '0'); hold on
plot(data_mag.time, data_mag.data(:, 2), 'DisplayName', '1')
plot(data_mag.time, data_mag.data(:, 3), 'DisplayName', '1')
legend
title("Mag")
xlim([tspan(1), tspan(end)])

figure(22)
clf
plot(data_baro.time, data_baro.data(:, 1), 'DisplayName', '0'); hold on
legend
title("Baro")
xlim([tspan(1), tspan(end)])

%% Plot Innovation
figure(23)
clf
plot(tspan, kf.inno_hist(1:3, :), '.-', 'MarkerSize', 10);
legend("0", "1", "2")
title("Position Innovation")
xlabel("Time (s)")
ylabel("Position Innovation (m)")
xlim([dt*10, tspan(end)])

figure(24)
clf
plot(tspan, kf.inno_hist(4:7, :), '.-', 'MarkerSize', 10);
legend("0", "1", "2", "3")
title("Quaternion Innovation")
xlabel("Time (s)")
ylabel("Quaternion Innovation")
xlim([dt*10, tspan(end)])

figure(25)
clf
plot(tspan, rad2deg(kf.inno_hist(8:10, :)), '.-', 'MarkerSize', 10);
legend("0", "1", "2")
title("Angular Velocity Innovation")
xlabel("Time (s)")
ylabel("Angular Velocity Innovation (rad/s)")
xlim([dt*10, tspan(end)])

return
%% ANIMATION
figure(16)
clf
run_animation(t_plot, p_est, e_est, 5, 10);

figure(16)
clf
for n = 1:3:num_steps-1
    poseplot(quaternion(e_est(n, 1), e_est(n, 2), e_est(n, 3), e_est(n, 4)))
    title(sprintf("t=%0.2f sec", t_plot(n)));
    drawnow
end

function plot_cov(variance)
    variance = squeeze(variance);
    plot(sqrt(variance), '--r', 'DisplayName', 'Covariance'); hold on;
    plot(-sqrt(variance), '--r', 'HandleVisibility', 'off');
end


function run_animation(t, position, orientation, step, substep)
numsteps = height(position);

quat = quaternion(orientation(1, :));
patch = poseplot(quat); hold on

patch.ScaleFactor = 100;
xlabel("X")
ylabel("Y")
zlabel("Z")

for i = 2:step:numsteps - step
    for j=i:substep:i+step
        quat = quaternion(orientation(j, :));
        pos = position(j, :);
    
        set(patch, Orientation=quat, Position=pos); hold on

        plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 1); hold on
    end

    set(gca,'ZDir','normal')  
    legend("Payload", "Trajectory")
    title(sprintf("t = %0.2f", t(i)))

    lim = 1000;
    xlim(position(j,1) + [-1, 1]*lim);
    ylim(position(j,2) + [-1, 1]*lim);
    zlim(position(j,3) + [-1, 1]*lim);

    drawnow

end
end