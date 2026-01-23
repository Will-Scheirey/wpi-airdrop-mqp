clearvars -except data_accel_all data_gyro_all data_mag_all data_gps_all data_baro_all data_gps_vel_all data_sensors_all data_gpsTrack_all old_dir; clc;
%% Load HPRC Data
% [data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data_2();

try
    addpath(genpath("Python"));
    addpath(genpath("MATLAB"));
    addpath(genpath("haars_data"));

catch exception
    fprintf("Exception: %s", exception.message);
    error("Please run code from the root directory!")
end

parent_dir = "haars_data";
drop_dir = "DN167_Lt3_n07_08072025_side_2";
full_dir = fullfile(parent_dir, drop_dir);

load_data = false;

if ~exist("data_accel_all", "var")
    load_data = true;
end

if ~exist("old_dir", "var")
    load_data = true;
elseif old_dir ~= full_dir
    load_data = true;
end

if load_data
    sensor_filename = fullfile(full_dir, "SENSOR.CSV");
    gps_filename = fullfile(full_dir, "TRACK.CSV");

    [data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all,...
        ] = get_and_trim_flysight(sensor_filename, gps_filename);

    mean_window = 10;
    data_gyro_all.data = movmean(data_gyro_all.data, mean_window, 1);
    data_baro_all.data = movmean(data_baro_all.data, mean_window);
    data_baro_all.data = movmean(data_baro_all.data, mean_window, 1);
    if ~isempty(data_gps_all)
        data_gps_all.data = movmean(data_gps_all.data, mean_window, 1);
    end

    old_dir = full_dir;
end

drop_info = get_drop_info(data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all);

if isempty(drop_info)
    drop_time = 4; % data_accel_all.time(1);
    land_time = data_accel_all.time(end);
    t_start = drop_time;
    t_dur = land_time - drop_time;
else
    drop_time = drop_info.time_drop;
    land_time = drop_info.time_land;
    t_start = data_accel_all.time(1);
    t_dur   = data_accel_all.time(end) - data_accel_all.time(1);
end

zero_alt_mean_window = 10;

data_baro = data_baro_all;
data_baro.data = data_baro.data - mean(data_baro.data(1:zero_alt_mean_window));

data_accel = data_accel_all;
data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);

if ~isempty(data_gps_all)
    data_gps = data_gps_all;
    data_gps.meas_idx   = ones(length(data_gps.time), 1);
    data_gps.data(:, 3) = data_gps.data(:, 3) - mean(data_gps.data(1:zero_alt_mean_window, 3));
end

data_mag = data_mag_all;
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);

data_gyro = data_gyro_all;
data_gyro.meas_idx  = repmat(3, length(data_gyro.time), 1);
data_baro.meas_idx  = repmat(4, length(data_baro.time), 1);

t_end   = t_start + t_dur;

if isempty(data_gps_all)
    acc_gps = 0;
else
    acc_gps = [data_gpsTrack_all.GNSS.hAcc, data_gpsTrack_all.GNSS.vAcc];
end

if ~isempty(data_gps_all)
    acc_gps = acc_gps(data_gps.time > t_start, :);
    data_gps = data_gps(data_gps.time > t_start, :);
    data_gps_vel = data_gps_vel_all(data_gps_vel_all.time > t_start, :);
end
data_accel = data_accel(data_accel.time > t_start, :);
data_mag = data_mag(data_mag.time > t_start, :);
data_gyro = data_gyro(data_gyro.time > t_start, :);
data_baro = data_baro(data_baro.time > t_start, :);

if ~isempty(data_gps_all)
    acc_gps = acc_gps(data_gps.time < t_end, :);
    data_gps = data_gps(data_gps.time < t_end, :);
    data_gps_vel = data_gps_vel(data_gps_vel_all.time < t_end, :);
end
data_accel = data_accel(data_accel.time < t_end, :);
data_mag = data_mag(data_mag.time < t_end, :);
data_gyro = data_gyro(data_gyro.time < t_end, :);
data_baro = data_baro(data_baro.time < t_end, :);

dt_min_accel = min(diff(data_accel.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

if ~isempty(data_gps_all)
    measurements = {data_gps, data_baro, data_mag, data_gyro};
    dt_min_gps   = min(diff(data_gps.time));
    dt          = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]) / 4;

else
    measurements = {data_mag, data_gyro};
    dt          = min([dt_min_accel, dt_min_mag, dt_min_gyro, dt_min_baro]) / 4;
end
inputs = data_accel;

tspan = t_start : dt : t_start + t_dur;

meas_freq = 4 / dt;

sensor = Sensor_FlySight(meas_freq);

%% Set up the Kalman Filter
num_steps = numel(tspan);

Rb = 0.1743;

% Rp = 0.25;
Rp = 10;
% Rp = sensor.gps_std_dev;
R_pos = [
    Rp, 0,   0;
    0,   Rp, 0;
    0,   0,   Rp
    ].^2;

% Rm = sensor.mag_std_dev * 1e2;
Rm = 1e-1;
R_mag = (Rm^2) * eye(3);

Rw = 1e-2;
% Rw = sensor.gyro_std_dev;
R_w = [
    Rw,  0,  0;
    0,   Rw, 0;
    0,   0,  Rw
].^2;

Rb = 1e2;
R_baro = Rb^2;

R = blkdiag( ...
    R_pos, ...
    R_mag, ...
    R_w,  ...
    R_baro ...
    );

Qp = 1e-3;
Q_P = [
    Qp, 0, 0;
    0, Qp, 0;
    0, 0, Qp;
] .^2;

cross_term = 1e-3;
diag_term = 1e-2;

Q_V = [
    diag_term,     cross_term,  cross_term;
    cross_term,    diag_term,   cross_term
    cross_term,    cross_term,  diag_term
];

Qe = 1e-3;
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

Qwb = 1e-12;
Q_wb = [
 Qwb, 0,   0;
 0,   Qwb, 0;
 0,   0,   Qwb
].^2;

Qab = 1e-12;
Q_ab = [
 Qab, 0,   0;
 0,   Qab, 0;
 0,   0,   Qab
].^2;

Qpb = 1e-12;
Q_pb = [
 Qpb, 0,   0;
 0,   Qpb, 0;
 0,   0,   Qpb
].^2;

Qmb = 1e-1;
Q_mb = Qmb * eye(3,3);

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    Q_e, ... % e
    Q_w, ... % w
    Q_wb, ... % wb
    Q_ab, ... % ab
    Q_pb, ...
    Q_mb ...
    );

P0 = blkdiag( ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(4), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2* eye(3), ...
    1e-2 * eye(3) ...
    );

Q = (Q + Q.')/2;                     % enforce symmetry
q_floor = 1e-12;                     % pick a floor in variance units
Q = Q + q_floor * eye(size(Q));      % jitter to make invertible

%% Run the Kalman Filter
payload = get_a22();

kf = Airdrop_EKF(R, Q, 0, P0, dt, payload.I());

if isempty(data_gps_all)
    data_gps.data = [0, 0, 0];
end

kf.initialize(true, data_accel.data(1, :)', data_gyro.data(1, :)', data_mag.data(1, :)', data_gps.data(1, :)', data_baro.data(1, :)');
kf.run_filter(measurements, inputs, tspan, acc_gps, drop_time, 1, true);

%{
%% Run the Smoother

sm = Forward_Backward_Smoother(R, Q, 0, P0, dt, payload.I());  % match your kf ctor
sm.is_initialized = true;
sm.set_forward_results(kf.x_hist, kf.P_hist, kf.F_hist, kf.Q_hist);
sm.smooth(measurements, inputs, tspan, acc_gps, true);

%% Fuse Data
[x_s, P_s] = sm.fuse();
%}
%% Exract Values

% x_est = x_s(:, 2:end)';
x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

p_est = x_est(:, kf.x_inds.P_E);
v_est = x_est(:, kf.x_inds.V_E);
e_est = x_est(:, kf.x_inds.e);
w_est = x_est(:, kf.x_inds.w_b);

w_b_est = x_est(:, kf.x_inds.b_g);
a_b_est = x_est(:, kf.x_inds.b_a);
p_b_est = x_est(:, kf.x_inds.b_p);
m_b_est = x_est(:, kf.x_inds.b_m);

t_plot = tspan(1:end-1);
drop_times     = 9050 : dt : 9500;
% t_plot_drop = [drop_times(1), drop_times(end)];
% t_plot_drop = [t_plot(1), t_plot(end)];
t_plot_drop = [drop_time, land_time];


%% Plot Values
fig_idx = new_fig(1);
clf
plot(t_plot, p_est, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Position (m)")
title("Position vs. Time")
xlim(t_plot_drop)

if ~isempty(data_gps_all)
fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(data_gps.time, data_gps.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_1^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps.time, data_gps.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_2^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps.time, data_gps.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_3^E vs. Time")
xlim(t_plot_drop)
end

fig_idx = new_fig(fig_idx);
clf
plot(tspan, kf.inno_hist(kf.measurement_ranges{1}(1:2), :), '.-', 'MarkerSize', 10); hold on
legend("X", "Y")
title("Position Innovation")
xlabel("Time (s)")
ylabel("Position Innovation (m)")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, v_est, 'LineWidth', 2); hold on
plot(t_plot, vecnorm(v_est, 2, 2), 'LineWidth', 1.5)
legend("V_0^E", "V_1^E", "V_2^E", "Velocity Norm")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("ECEF Velocity vs. Time")
xlim(t_plot_drop)

if ~isempty(data_gps_all)
fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(data_gps_vel.time, data_gps_vel.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_1^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps_vel.time, data_gps_vel.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_2^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps_vel.time, data_gps_vel.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m)")
title("V_3^E vs. Time")
xlim(t_plot_drop)
end

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, e_est, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Value")
title("Quaternion vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(tspan, kf.inno_hist(kf.measurement_ranges{2}, :), '.-', 'MarkerSize', 10);
legend("m_x", "m_y", "m_z")
title("Magnetometer Innovation")
xlabel("Time (s)")
ylabel("Mag Innovation")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, m_b_est(:, 1), 'DisplayName', 'X', 'LineWidth', 1.5); hold on
plot(t_plot, m_b_est(:, 2), 'DisplayName', 'Y', 'LineWidth', 1.5);
plot(t_plot, m_b_est(:, 3), 'DisplayName', 'Z', 'LineWidth', 1.5);
xlabel("Time (s)")
ylabel("Bias Estimate (gauss)")
title("Magnetometer Bias Estimate vs Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, w_est, 'LineWidth', 2)
legend("w_0^B", "w_1^B", "w_2^B")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("Body Angular Velocity vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(data_gyro.time, data_gyro.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, w_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("\omega_1^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gyro.time, data_gyro.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, w_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("\omega_2^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gyro.time, data_gyro.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, w_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("\omega_3^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf

inno = kf.inno_hist(kf.measurement_ranges{3}(1), :);
good = ~isnan(inno);

% subplot(3,1,1)
plot(tspan(good), inno(good), 'DisplayName', 'Innovation'); hold on
plot_cov(kf.P_hist(kf.x_inds.w_b(1),kf.x_inds.w_b(1),:));
lim = max(kf.P_hist(kf.x_inds.w_b(1),kf.x_inds.w_b(1),:));
ylim([-lim, lim])

legend
title("Gyro Innovation")
xlabel("Time (s)")
ylabel("Gyro Innovation (rad/s)")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(tspan, kf.accel_calc_all, 'LineWidth', 2); hold on
legend("a_1", "a_2", "a_3")
xlim([t_plot(1)+1, t_plot(end)])
% xlim(t_plot_drop)
title("Estimated Acceleration Components")

fig_idx = new_fig(fig_idx);
clf
plot(data_accel.time, abs(vecnorm(data_accel.data, 2, 2) - 9.81), 'LineWidth', 2, 'DisplayName', 'Measured Accel Norm'); hold on
plot(t_plot, vecnorm(kf.accel_calc_all(1:end-1, :), 2, 2), 'LineWidth', 2, 'DisplayName', 'Estimated Accel Norm'); hold on
legend
xlim([t_plot(1)+1, t_plot(end)])
title("Measured and Estimated Acceleration Norm")
ylim([-1, 12])

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, w_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (rad/s)")
xlim([t_plot(1)+1, t_plot(end)])

title("Gyro Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, a_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m/s^2)")
xlim([t_plot(1)+1, t_plot(end)])

title("Acceleration Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, p_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m)")
xlim([t_plot(1)+1, t_plot(end)])

title("GPS Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
pos_inno = kf.inno_hist(1:3, :)';
good_pos_inno = ~isnan(pos_inno(:, 1));
pos_inno_tstep = 1:length(pos_inno);

x_pos_inno = pos_inno(good_pos_inno, 1);

idx = 1;
plot_cov(kf.P_hist(idx,idx,:)); hold on
plot(pos_inno_tstep(good_pos_inno), x_pos_inno)
xlabel("Timestep")
ylabel("Variance (rm/s)")
title("X Position Variance and Innovation")
% xlim([0, 26313])
ylim([-7, 7])
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
y_pos_inno = pos_inno(good_pos_inno, 2);
idx = 2;
plot_cov(kf.P_hist(idx,idx,:)); hold on
plot(pos_inno_tstep(good_pos_inno), y_pos_inno)
xlabel("Timestep")
ylabel("Variance (rm/s)")
title("X_1 Position Variance")

fig_idx = new_fig(fig_idx);
clf

idx = 7;
plot_cov(kf.P_hist(idx,idx,:)); hold on

e_inno = kf.inno_hist(4:6, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_0_inno = e_inno(good_e_inno, 1);

plot(e_inno_tstep(good_e_inno), e_0_inno)

xlabel("Timestep")
ylabel("Variance ")
title("e_0 Quaternion Part Variance")
% xlim([0, 26313])
ylim([-1,1]*0.15)
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
idx = 8;
plot_cov(kf.P_hist(idx,idx,:)); hold on
e_inno = kf.inno_hist(4:6, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_1_inno = e_inno(good_e_inno, 2);

plot(e_inno_tstep(good_e_inno), e_1_inno)

xlabel("Timestep")
ylabel("Variance ")
title("e_1 Quaternion Part Variance")
ylim([-1,1]*0.15)
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
plot3(p_est(:,1), p_est(:,2), p_est(:,3), '.-', 'MarkerSize', 10, 'DisplayName', 'Estimate', 'LineWidth', 0.5); hold on;
plot3(p_est(1,1), p_est(1,2), p_est(1,3), '.r', 'MarkerSize', 30, 'DisplayName', 'Start', 'LineWidth', 1); hold on;
plot3(p_est(end,1), p_est(end,2), p_est(end,3), '.g', 'MarkerSize', 30, 'DisplayName', 'End', 'LineWidth', 1); hold on;
legend
axis equal
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
title("Trajectory")

fig_idx = new_fig(fig_idx);
clf
plot3(p_est(:,1), p_est(:,2), p_est(:,3), '.-', 'MarkerSize', 10, 'DisplayName', 'Estimate', 'LineWidth', 0.5); hold on;
plot3(p_est(1,1), p_est(1,2), p_est(1,3), '.r', 'MarkerSize', 30, 'DisplayName', 'Start', 'LineWidth', 1); hold on;
plot3(p_est(end,1), p_est(end,2), p_est(end,3), '.g', 'MarkerSize', 30, 'DisplayName', 'End', 'LineWidth', 1); hold on;
plot3(data_gps.data(:, 1), data_gps.data(:, 2), data_gps.data(:, 3), "--k", "LineWidth", 0.5, 'DisplayName', 'GPS'); hold on
legend
axis equal
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
title("Trajectory")

fig_idx = new_fig(fig_idx);
clf

drop_idx = t_plot > drop_time;
x_pos = p_est(drop_idx, 1) - p_est(find(drop_idx, 1), 1);
y_pos = p_est(drop_idx, 2) - p_est(find(drop_idx, 1), 2);
z_pos = p_est(drop_idx, 3);

yyaxis left
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), x_pos, 'LineWidth', 1.5, 'DisplayName', 'X'); hold on;
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), y_pos, 'LineWidth', 1.5, 'DisplayName', 'Y'); hold on;
ylabel("X, Y Position (m)")

yyaxis right
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), z_pos, 'LineWidth', 1.5, 'DisplayName', 'z'); hold on;
ylabel("Altitude (m)")

xlabel("Time (s)")
legend
title("Position vs. Time")

%% Plot Innovation

baro_values = ~ (kf.inno_hist(10, 2:end) == 0);

fig_idx = new_fig(fig_idx);
clf
plot(tspan(baro_values), kf.inno_hist(10, baro_values), '.-', 'MarkerSize', 10., 'LineWidth', 2);
title("Baro Innovation")
xlabel("Time (s)")
ylabel("Innovation (m)")
xlim(t_plot_drop)

%% Plot Cross-Covariance

%{
fig_idx = new_fig(fig_idx);
clf
idx1 = kf.x_inds.P_E(1);
idx2 = kf.x_inds.e(1);

fields = fieldnames(kf.x_inds);

for i = 1:length(fields)
    idxs = kf.x_inds.(fields{i});
    tick_idxs(i) = idxs(1);
end

% plot(squeeze(kf.P_hist(idx1, idx2, :)))

for n=1:100:num_steps
    clf
    data = sqrt(max(squeeze(kf.P_hist(:,:,n)), 1e-25));
    contourf(log10(data), 'LineStyle', 'none')
    title(sprintf("Time = %0.2fs", tspan(n)))

    xticks(tick_idxs);
    xticklabels(fields)

    yticks(tick_idxs);
    yticklabels(fields)

    colorbar
    drawnow
end
%}

%{
%% Plot Smoothed and Unsmoothed
fig_idx = new_fig(fig_idx);
clf


drop_time = get_drop_info(data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all).time_drop;



kf_pos = kf.x_hist(1:3, tspan > drop_time);
sm_pos = p_est(tspan(2:end) > drop_time, :)';

plot3(kf_pos(1,:), kf_pos(2,:), kf_pos(3,:), '.r', 'DisplayName', 'kf', 'Clipping', 'off'); hold on
plot3(sm_pos(1,:), sm_pos(2,:), sm_pos(3,:), '-b', 'DisplayName', 'sm', 'Clipping', 'off'); hold on
legend

return
%}

%% HEADING
fig_idx = new_fig(fig_idx);
clf
heading = atan2d(v_est(:,1), v_est(:,2));
plot(heading, 'DisplayName', 'Vel'); hold on

doit = false;

if doit
    quat_data = [e_est(:, 4), e_est(:, 1), e_est(:, 2), e_est(:, 3)];
else
    quat_data = [e_est(:, 1), e_est(:, 2), e_est(:, 3), e_est(:, 4)];
end

euler_angles = rad2deg(quat2eul(quat_data));
plot(euler_angles(:, 1), 'DisplayName', 'Quat')

%% MEASUREMENTS
fig_idx = plot_meas(data_gps_all, data_accel_all, data_gyro_all, data_mag_all, data_baro_all, data_gps_vel_all, fig_idx);

return
%% Down Vector
figure(16)
down_vec_all = kf.down_vec_all;
num_vec = height(down_vec_all);
lim = [-1,1];
for n = 1:10:num_vec
    clf
    time = t_plot(n);

    accel = data_accel.data(find(data_accel.time > time, 1), :);
    accel = accel / norm(accel);

    quiver3(0, 0, 0, down_vec_all(n, 1), down_vec_all(n, 2), down_vec_all(n, 3), 'LineWidth', 5, 'DisplayName', 'Down'); hold on
    quiver3(0, 0, 0, accel(1), accel(2), accel(3), 'LineWidth', 5, 'DisplayName', 'Accel'); hold on

    xlim(lim);
    ylim(lim);
    zlim(lim);
    title("t=" + time);
    drawnow
    pause(0.01)
end

%% Acceleration Vector
figure(16)
accel_all = kf.accel_calc_all;
num_vec = height(accel_all);
lim = [-1,1] * 10;
for n = 1:num_vec
    clf
    time = t_plot(n);

    quiver3(0, 0, 0, accel_all(n, 1), accel_all(n, 2), accel_all(n, 3), 'LineWidth', 1, 'DisplayName', 'Down'); hold on

    xlim(lim);
    ylim(lim);
    zlim(lim);
    title("t=" + time);
    drawnow
    pause(0.01)
end

%% Acceleration Vector
figure(16)
accel_all = kf.accel_calc_all;
num_vec = height(accel_all);
lim = [-1,1] * 2;
n_start = find(tspan > 4650, 1);
for n = n_start:num_vec
    clf
    time = t_plot(n);

    quiver3(0, 0, 0, accel_all(n, 1), accel_all(n, 2), accel_all(n, 3), 'LineWidth', 2, 'DisplayName', 'Accel'); hold on
    quiver3(0, 0, 0, v_est(n, 1), v_est(n, 2), v_est(n, 3), 'LineWidth', 2, 'DisplayName', 'Vel'); hold on

    xlim(lim);
    ylim(lim);
    zlim(lim);
    title("t=" + time);
    legend
    drawnow
    pause(0.01)
end

%% MAG
figure(1002)
clf
lim = 1.5;
for i=1:10:height(data_mag.data)
    plot3(data_mag.data(i,1 ), data_mag.data(i,2), data_mag.data(i,3), '.b'); hold on
    xlim([-1,1]*lim)
    ylim([-1,1]*lim)
    zlim([-1,1]*lim)
    title("t=" + data_mag.time(i));
    drawnow
end

%% ANIMATION
figure(16)
clf

animation_start_time = drop_time;
animation_start_time = 0;
start_idx = find(t_plot > animation_start_time, 1);
run_animation(t_plot, p_est, e_est, 500, 100, start_idx, false);

function plot_cov(variance)
    variance = squeeze(variance);
    plot(sqrt(variance), '.r', 'DisplayName', 'Covariance', 'LineWidth', 1.5); hold on;
    plot(-sqrt(variance), '.r', 'HandleVisibility', 'off', 'LineWidth', 1.5);
end


function run_animation(t, position, orientation, step, substep, start_idx, save_video)

if nargin < 7
    save_video = false;
end

numsteps = height(position);

quat = quaternion(orientation(1, :));
patch = poseplot(quat); hold on

lim = 1000;

patch.ScaleFactor = lim / 10;
xlabel("X")
ylabel("Y")
zlabel("Z")

if save_video
outputVideo = VideoWriter('myVideo.mp4', 'MPEG-4'); % Specify filename and format
open(outputVideo);
end

for i = start_idx:step:numsteps - step
    for j=i:substep:i+step
        quat = quaternion(orientation(j, 1), orientation(j, 2), orientation(j, 3), orientation(j, 4));
        pos = position(j, :);
    
        set(patch, Orientation=quat, Position=pos); hold on

        plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 1); hold on
    end

    set(gca,'ZDir','normal')  
    legend("Payload", "Trajectory")
    title(sprintf("t = %0.2f", t(i)))

    xlim(position(j,1) + [-1, 1]*lim);
    ylim(position(j,2) + [-1, 1]*lim);
    zlim(position(j,3) + [-1, 1]*lim);

    drawnow

    if save_video
        frame = getframe(gcf); % captures the current figure (gcf)
    
        writeVideo(outputVideo, frame);
    end

end
if save_video
close(outputVideo)
end
end