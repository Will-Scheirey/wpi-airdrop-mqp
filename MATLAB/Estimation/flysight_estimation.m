clearvars -except data_accel_all data_gyro_all data_mag_all data_gps_all data_baro_all data_gps_vel_all data_sensors_all data_gpsTrack_all old_dir data_stationary data_moving drop_info; clc;
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
drop_dir = "DN171_Lt3_n11_08072025_side_2";
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

    
[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_flysight_data(sensor_filename, gps_filename, false);

[data_stationary, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);

    data_accel_all = data_moving.data_accel;
    data_gyro_all = data_moving.data_gyro;
    data_mag_all = data_moving.data_mag;
    data_gps_all = data_moving.data_gps;
    data_baro_all = data_moving.data_baro;
    data_gps_vel_all = data_moving.data_gps_vel;
    data_sensors_all = data_moving.data_flysight_sensor;
    data_gpsTrack_all = data_moving.data_flysight_gps;

    
    drop_info = get_drop_info(data_accel_all,...
            data_gyro_all,... 
            data_mag_all,...
            data_gps_all,...
            data_baro_all,...
            data_gps_vel_all,...
            data_sensors_all,...
            data_gpsTrack_all);

    mean_window = 20;
    data_gyro_all.data  = movmean(data_gyro_all.data, mean_window, 1);
    data_accel_all.data = movmean(data_accel_all.data, mean_window, 1);
    data_gps_all.data = movmean(data_gps_all.data, mean_window, 1);
    old_dir = full_dir;
end

[var_accel, var_gyro, var_gps, var_mag, var_baro] = calibrate_sensors(data_stationary);

% fig_idx = plot_meas(data_stationary.data_gps, data_stationary.data_accel, data_stationary.data_gyro, data_stationary.data_mag, data_stationary.data_mag, data_stationary.data_gps_vel);

% return

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
data_gyro.meas_idx  = repmat(-1, length(data_gyro.time), 1);
data_baro.meas_idx  = repmat(3, length(data_baro.time), 1);

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
    measurements = {data_gps, data_mag(data_mag.time < drop_time, :), data_baro};
    dt_min_gps   = min(diff(data_gps.time));
    dt          = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]) / 4;

else
    measurements = {data_mag, data_gyro};
    dt          = min([dt_min_accel, dt_min_mag, dt_min_gyro, dt_min_baro]) / 4;
end
inputs = table(data_accel.time, data_accel.data, data_gyro.data, ...
'VariableNames', {'time', 'accel', 'gyro'});

tspan = t_start : dt : t_start + t_dur;

meas_freq = 4 / dt;

sensor = Sensor_FlySight(meas_freq);

%% Set up the Kalman Filter
num_steps = numel(tspan);

Rp = 10;
% Rp = sensor.gps_std_dev;
R_pos = eye(3) * Rp^2;

% Rm = sensor.mag_std_dev * 1e2;
R_mag = blkdiag(var_mag(1), var_mag(2), var_mag(3)).^2;

R_baro = var_baro^2;

R = blkdiag( ...
    R_pos, ...
    R_mag, ...
    R_baro ...
    );

sigma_a = eye(3) * sqrt(norm(var_accel)) * 1e1;

Q_V = (dt*sigma_a).^2;
Q_P = (0.5*dt^2*sigma_a).^2;

sigma_w = sqrt(norm(var_gyro));
Q_e = eye(4) * (dt*sigma_w)^2;

Qwb = 1e-5;
Q_wb = eye(3) * Qwb^2;

Qab = 1e-5;
Q_ab = eye(3) * Qab^2;

Qpb = 1e-4;
Q_pb = eye(3) * Qpb^2;

sigma_bm = 1e-2;
Q_mb = eye(3) * (sigma_bm^2);

sigma_bb = 1e-2;
Q_bb = sigma_bb^2;

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    Q_e, ... % e
    Q_wb, ... % w
    Q_ab, ... % ab
    Q_pb, ...
    Q_mb, ...
    Q_bb ...
    );

P0 = blkdiag( ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(4), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2* eye(3), ...
    1e-2 * eye(3), ...
    1e-2 ...
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
% w_est = x_est(:, kf.x_inds.w_b);

w_b_est = x_est(:, kf.x_inds.b_g);
a_b_est = x_est(:, kf.x_inds.b_a);
p_b_est = x_est(:, kf.x_inds.b_p);
m_b_est = x_est(:, kf.x_inds.b_m);
b_b_est = x_est(:, kf.x_inds.b_b);

t_plot = tspan(1:end-1);
drop_times     = 9050 : dt : 9500;
% t_plot_drop = [drop_times(1), drop_times(end)];
% t_plot_drop = [t_plot(1), t_plot(end)];
t_plot_drop = [drop_time, land_time];



%{
%% Plot Innovation

baro_values = ~ (kf.inno_hist(kf.x_in, 2:end) == 0);

fig_idx = new_fig(fig_idx);
clf
plot(tspan(baro_values), kf.inno_hist(10, baro_values), '.-', 'MarkerSize', 10., 'LineWidth', 2);
title("Baro Innovation")
xlabel("Time (s)")
ylabel("Innovation (m)")
xlim(t_plot_drop)
%}

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
%{
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
%}
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
run_animation(t_plot, p_est, e_est, 500, 10, start_idx, false);


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