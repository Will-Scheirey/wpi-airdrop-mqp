clearvars -except data_accel_all data_gyro_all data_mag_all data_gps_all data_baro_all data_gps_vel_all data_sensors_all data_gpsTrack_all old_folder; clc;
%% Load HPRC Data
% [data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data_2();

folder = "Drop2";
load_data = false;

if ~exist("data_accel_all", "var")
    load_data = true;
end

if ~exist("old_folder", "var")
    load_data = true;
elseif old_folder ~= folder
     load_data = true;
end

if load_data
    [data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all,...
        ] = get_flysight_data("MATLAB/Data/" + folder + "/SENSOR.CSV", "MATLAB/Data/" + folder + "/TRACK.CSV", true);

[data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all,...
        ] = trim_flysight(data_accel_all,...
        data_gyro_all,... 
        data_mag_all,...
        data_gps_all,...
        data_baro_all,...
        data_gps_vel_all,...
        data_sensors_all,...
        data_gpsTrack_all ...
        );

    old_folder = folder;
end

zero_alt_mean_window = 1000;

data_baro = data_baro_all;
data_baro.data = data_baro.data - mean(data_baro.data(1:zero_alt_mean_window));

data_accel = data_accel_all;
data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);

data_gps = data_gps_all;
data_gps.meas_idx   = ones(length(data_gps.time), 1);
data_gps.data(:, 3) = data_gps.data(:, 3) - mean(data_gps.data(1:zero_alt_mean_window, 3));

data_mag = data_mag_all;
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);

data_gyro = data_gyro_all;
data_gyro.meas_idx  = repmat(3, length(data_gyro.time), 1);
data_baro.meas_idx  = repmat(4, length(data_baro.time), 1);

% t_start = 5000;
% t_dur   = 2000;
t_start = data_accel.time(1);
t_dur   = data_accel.time(end) - data_accel.time(1);
t_end   = t_start + t_dur;

acc_gps = [data_gpsTrack_all.GNSS.hAcc, data_gpsTrack_all.GNSS.vAcc];

acc_gps = acc_gps(data_gps.time > t_start, :);
data_accel = data_accel(data_accel.time > t_start, :);
data_gps = data_gps(data_gps.time > t_start, :);
data_mag = data_mag(data_mag.time > t_start, :);
data_gyro = data_gyro(data_gyro.time > t_start, :);
data_baro = data_baro(data_baro.time > t_start, :);
data_gps_vel = data_gps_vel_all(data_gps_vel_all.time > t_start, :);

acc_gps = acc_gps(data_gps.time < t_end, :);
data_accel = data_accel(data_accel.time < t_end, :);
data_gps = data_gps(data_gps.time < t_end, :);
data_mag = data_mag(data_mag.time < t_end, :);
data_gyro = data_gyro(data_gyro.time < t_end, :);
data_baro = data_baro(data_baro.time < t_end, :);
data_gps_vel = data_gps_vel(data_gps_vel_all.time < t_end, :);

data_accel.data = movmean(data_accel.data, 10, 1);

measurements = {data_gps, data_baro, data_mag, data_gyro};
inputs = data_accel;

dt_min_accel = min(diff(data_accel.time));
dt_min_gps   = min(diff(data_gps.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

dt        = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]) / 4;
% tspan = 0 : dt : data_accel.time(end);
tspan = t_start : dt : t_start + t_dur;
% tspan = 9000 : dt: 9500;

% tspan = 0 : dt : 25;
meas_freq = 1 / dt;

%% Set up the Kalman Filter
num_steps = numel(tspan);

Rb = 0.1743;

Rp = 0.25;
Rp = 10;
R_pos = [
    Rp, 0,   0;
    0,   Rp, 0;
    0,   0,   Rp
    ].^2;

Rq = 1e-2;
R_quat = [
    Rq, 0,  0,  0;
    0,  Rq, 0,  0;
    0,  0,  Rq, 0;
    0,  0,  0,  Rq
    ].^2;

Rw = 1e-4;
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

Qp = 1e-1;
Q_P = [
    Qp, 0, 0;
    0, Qp, 0;
    0, 0, Qp;
] .^2;

cross_term = 1e-7;
diag_term = 1e-1;

Q_V = [
    diag_term,     cross_term,  cross_term;
    cross_term,    diag_term,   cross_term
    cross_term,    cross_term,  diag_term
];

Qe = 1e-1;
Q_e = [
    Qe,  0,  0,  0;
    0,   Qe, 0,  0;
    0,   0,  Qe, 0;
    0,   0,  0,  Qe
].^2;

Qw = 1e-1;
Q_w = [
    Qw, 0,  0;
    0,  Qw, 0;
    0,  0,  Qw
].^2;

Qwb = 1e-8;
Q_wb = [
 Qwb, 0,   0;
 0,   Qwb, 0;
 0,   0,   Qwb
].^2;

Qab = 1e-9;
Q_ab = [
 Qab, 0,   0;
 0,   Qab, 0;
 0,   0,   Qab
].^2;

Qpb = 1e-6;
Q_pb = [
 Qpb, 0,   0;
 0,   Qpb, 0;
 0,   0,   Qpb
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
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(4), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2* eye(3) ...
    );

%% Run the Kalman Filter
payload = get_a22();
kf = EKF_Varying_Measurements(R, Q, 0, P0, dt, payload.I());

kf.initialize(true, data_accel.data(1, :)', data_gyro.data(1, :)', data_mag.data(1, :)', data_gps.data(1, :)', data_baro.data(1, :)');

kf.run_filter(measurements, inputs, tspan, acc_gps);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_est = x_est(:, kf.x_inds.P_E);
v_est = x_est(:, kf.x_inds.V_E);
e_est = x_est(:, kf.x_inds.e);
w_est = x_est(:, kf.x_inds.w_b);

w_b_est = x_est(:, kf.x_inds.b_g);
a_b_est = x_est(:, kf.x_inds.b_a);
p_b_est = x_est(:, kf.x_inds.b_p);

t_plot = tspan(1:end-1);
drop_times     = 9050 : dt : 9500;
% t_plot_drop = [drop_times(1), drop_times(end)];
t_plot_drop = [t_plot(1), t_plot(end)];


%% Plot Values
figure(1)
clf
plot(t_plot, p_est, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Position (m)")
title("Position vs. Time")
xlim(t_plot_drop)

figure(2)
clf
subplot(3,1,1)
plot(data_gps.time, data_gps.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_1^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps.time, data_gps.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_2^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps.time, data_gps.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_3^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

figure(3)
clf
plot(tspan, kf.inno_hist(1:2, :), '.-', 'MarkerSize', 10); hold on
legend("X", "Y")
title("Position Innovation")
xlabel("Time (s)")
ylabel("Position Innovation (m)")
xlim(t_plot_drop)

figure(4)
clf
plot(t_plot, v_est, 'LineWidth', 2); hold on
plot(t_plot, vecnorm(v_est, 2, 2), 'LineWidth', 1.5)
legend("V_0^E", "V_1^E", "V_2^E", "Velocity Norm")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("ECEF Velocity vs. Time")
xlim(t_plot_drop)

figure(5)
clf
subplot(3,1,1)
plot(data_gps_vel.time, data_gps_vel.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_1^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps_vel.time, data_gps_vel.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_2^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps_vel.time, data_gps_vel.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m)")
title("V_3^E vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

figure(6)
clf
plot(t_plot, e_est, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Value")
title("Quaternion vs. Time")
xlim(t_plot_drop)

figure(7)
clf
subplot(4,1,1)
plot(tspan, kf.quat_meas_all(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, e_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Quaternion Part")
title("e_1 vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(4,1,2)
plot(tspan, kf.quat_meas_all(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, e_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Quaternion Part")
title("e_2 vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(4,1,3)
plot(tspan, kf.quat_meas_all(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, e_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Quaternion Part")
title("e_3 vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

subplot(4,1,4)
plot(tspan, kf.quat_meas_all(:, 4), '.', 'MarkerSize', 10); hold on
plot(t_plot, e_est(:,4), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Quaternion Part")
title("e_4 vs. Time")
xlim([t_plot(1), t_plot(end)])
xlim(t_plot_drop)

figure(8)
clf
plot(tspan, kf.inno_hist(4:7, :), '.-', 'MarkerSize', 10);
legend("0", "1", "2", "3")
title("Quaternion Innovation")
xlabel("Time (s)")
ylabel("Quaternion Innovation")
xlim(t_plot_drop)

%{
figure(7)
quats = quaternion(e_est(:, 1), e_est(:, 2), e_est(:, 3), e_est(:, 4));
eul_angles = rad2deg(quat2eul(quats));

clf
plot(t_plot, eul_angles(:, 1), 'LineWidth', 2, 'DisplayName', 'Eul 1'); hold on
plot(t_plot, eul_angles(:, 2), 'LineWidth', 2, 'DisplayName', 'Eul 2'); hold on
plot(t_plot, eul_angles(:, 3), 'LineWidth', 2, 'DisplayName', 'Eul 3'); hold on
legend
xlim(t_plot_drop)
%}

figure(9)
clf
plot(t_plot, w_est, 'LineWidth', 2)
legend("w_0^B", "w_1^B", "w_2^B")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
title("Body Angular Velocity vs. Time")
xlim(t_plot_drop)

figure(10)
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

figure(11)
clf
plot(tspan, kf.inno_hist(8:10, :), '.-', 'MarkerSize', 10);
legend("0", "1", "2")
title("Angular Velocity Innovation")
xlabel("Time (s)")
ylabel("Angular Velocity Innovation (rad/s)")
xlim(t_plot_drop)

figure(12)
clf
plot(tspan, kf.accel_calc_all, 'LineWidth', 2); hold on
legend("a_1", "a_2", "a_3")
xlim([t_plot(1)+1, t_plot(end)])
% xlim(t_plot_drop)
title("Estimated Acceleration Components")

figure(13)
clf
plot(data_accel.time, abs(vecnorm(data_accel.data, 2, 2) - 9.81), 'LineWidth', 2, 'DisplayName', 'Measured Accel Norm'); hold on
plot(t_plot, vecnorm(kf.accel_calc_all(1:end-1, :), 2, 2), 'LineWidth', 2, 'DisplayName', 'Estimated Accel Norm'); hold on
legend
xlim([t_plot(1)+1, t_plot(end)])
title("Measured and Estimated Acceleration Norm")
xlim([6350, 6650])
ylim([-1, 12])

figure(14)
clf
plot(t_plot, w_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (rad/s)")
xlim([t_plot(1)+1, t_plot(end)])

title("Gyro Bias Estimates")

figure(15)
clf
plot(t_plot, a_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m/s^2)")
xlim([t_plot(1)+1, t_plot(end)])

title("Acceleration Bias Estimates")

figure(16)
clf
plot(t_plot, p_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m)")
xlim([t_plot(1)+1, t_plot(end)])

title("GPS Bias Estimates")

figure(17)
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

figure(171)
clf

y_pos_inno = pos_inno(good_pos_inno, 2);

idx = 2;
plot_cov(kf.P_hist(idx,idx,:)); hold on
plot(pos_inno_tstep(good_pos_inno), y_pos_inno)
xlabel("Timestep")
ylabel("Variance (rm/s)")
title("\X_0 Position Variance")
% xlim([0, 26313])

var(y_pos_inno)
mean(y_pos_inno)


figure(1001)
clf

idx = 7;
plot_cov(kf.P_hist(idx,idx,:)); hold on

e_inno = kf.inno_hist(4:7, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_0_inno = e_inno(good_e_inno, 1);

plot(e_inno_tstep(good_e_inno), e_0_inno)
%{
pos_inno = kf.inno_hist(1:3, :)';
good_pos_inno = ~isnan(pos_inno(:, 1));

plot(tspan(good_pos_inno), pos_inno(good_pos_inno, 1))
%}
xlabel("Timestep")
ylabel("Variance ")
title("e_0 Quaternion Part Variance")
% xlim([0, 26313])
ylim([-1,1]*0.15)
legend("Variance", "Innovation")


figure(1002)
clf

idx = 8;
plot_cov(kf.P_hist(idx,idx,:)); hold on

e_inno = kf.inno_hist(4:7, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_1_inno = e_inno(good_e_inno, 2);

plot(e_inno_tstep(good_e_inno), e_1_inno)
%{
pos_inno = kf.inno_hist(1:3, :)';
good_pos_inno = ~isnan(pos_inno(:, 1));

plot(tspan(good_pos_inno), pos_inno(good_pos_inno, 1))
%}
xlabel("Timestep")
ylabel("Variance ")
title("e_1 Quaternion Part Variance")
% xlim([0, 26313])
ylim([-1,1]*0.15)
legend("Variance", "Innovation")

% 
% subplot(3,1,2)
% idx = 2;
% plot_cov(kf.P_hist(idx,idx,:))
% xlabel("Timestep")
% ylabel("Variance (m)")
% title("Y Position Variance")
% 
% subplot(3,1,3)
% idx = 3;
% plot_cov(kf.P_hist(idx,idx,:))
% xlabel("Timestep")
% ylabel("Variance (m)")
% title("Z Position Variance")

figure(18)
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

figure(19)
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

figure(1000)
clf
yyaxis left
drop_time = 6365;
drop_idx = t_plot > drop_time;
x_pos = p_est(drop_idx, 1) - p_est(find(drop_idx, 1), 1);
y_pos = p_est(drop_idx, 2) - p_est(find(drop_idx, 1), 2);
z_pos = p_est(drop_idx, 3);

plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), x_pos, 'LineWidth', 1.5, 'DisplayName', 'X'); hold on;
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), y_pos, 'LineWidth', 1.5, 'DisplayName', 'Y'); hold on;
ylabel("X, Y Position (m)")
yyaxis right
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), z_pos, 'LineWidth', 1.5, 'DisplayName', 'z'); hold on;
xlabel("Time (s)")
ylabel("Altitude (m)")

legend

title("Position vs. Time")

%% Plot Innovation

baro_values = ~ (kf.inno_hist(11, 2:end) == 0);

figure(20)
clf
plot(tspan(baro_values), kf.inno_hist(11, baro_values), '.-', 'MarkerSize', 10., 'LineWidth', 2);
title("Baro Innovation")
xlabel("Time (s)")
ylabel("Innovation (m)")
xlim(t_plot_drop)

figure(21)
clf
pos_inno = kf.inno_hist(1:2, :)';
good_pos_inno = ~isnan(pos_inno(:, 1));
gps_var = [data_gpsTrack_all.GNSS.hAcc, data_gpsTrack_all.GNSS.hAcc] * 2;

plot(data_gpsTrack_all.GNSS.time, vecnorm(gps_var, 2, 2), 'LineWidth', 2); hold on
legend("Estimated GPS Accuracy")
xlim(t_plot_drop)
return

%% Down Vector
figure(16)
num_vec = height(down_vec_all);
lim = [-1,1];
for n = 1:num_vec
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

%% ANIMATION
figure(16)
clf
animation_start_time = 4650;
start_idx = find(t_plot > animation_start_time, 1);
run_animation(t_plot, p_est, e_est, 200, 50, start_idx, false);

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
        quat = quaternion(orientation(j, 4), orientation(j, 1), orientation(j, 2), orientation(j, 3));
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