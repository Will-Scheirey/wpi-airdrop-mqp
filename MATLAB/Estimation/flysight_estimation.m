clearvars -except data; clc; close all

%% Load Flysight Data
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

%% Load HPRC Data
clear; clc; close all
[data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data();

data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);
data_gps.meas_idx   = repmat(1, length(data_gps.time), 1);
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);
data_gyro.meas_idx  = repmat(3, length(data_gyro.time), 1);
data_baro.meas_idx  = repmat(4, length(data_baro.time), 1);


measurements = {data_gps, data_mag, data_gyro, data_baro};
% measurements = {data_mag, data_gyro};
inputs = data_accel;

dt_min_accel = min(diff(data_accel.time));
dt_min_gps   = min(diff(data_gps.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

dt        = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]) / 3;
tspan     = data_accel.time(1) : dt : data_accel.time(end) / 10;
meas_freq = 1 / dt;

plot(data_gps.time, data_gps.data(:, 1)); hold on
plot(data_gps.time, data_gps.data(:, 2))
plot(data_gps.time, data_gps.data(:, 3))

%% Create Sensor Info
sensor = Sensor_FlySight(1000);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
e_std_dev = sensor.mag_std_dev;
w_bias_std_dev = 20;

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
];

R = blkdiag( ...
    [25, 0, 0; ...
    0,  25, 0; ...
    0,  0, 100].^2, ...
    [1, 0, 0, 0; ...
    0,    1, 0, 0; ...
    0,    0,   1, 0; ...
    0,    0,   0,   1].^2, ...
    (w_std_dev^2) * eye(3),  ...
    100 ...
    );

Q_P = [
    1, 0,  0;
    0,    1, 0;
    0,    0,    1;
] * 1e0;

cross_term = 1e-1;
diag_term = 1e0;

Q_V = [
    diag_term,           cross_term,  cross_term;
    cross_term,    diag_term,        cross_term
    cross_term,    cross_term,  diag_term
];

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    1e0 * eye(4), ... % e
    1e0 * eye(3), ... % w
    1e-8 * eye(3) ...
    );

P0 = blkdiag( ...
    1e1 * eye(3), ...
    1e2 * eye(3), ...
    1e-3 * eye(4), ...
    1e-3 * eye(3), ...
    w_bias_std_dev^2 * eye(3) ...
    );

%% Run the Kalman Filter
flysight_box = Box(0.1524, 0.1524, 3.6576, 20);

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

%% Plot IMU

figure(1)
clf
plot(data_mag.time, data_mag.data(:, 1), 'LineWidth', 2, 'DisplayName', 'Mag X'); hold on
plot(data_mag.time, data_mag.data(:, 2), 'LineWidth', 2, 'DisplayName', 'Mag X'); hold on
plot(data_mag.time, data_mag.data(:, 3), 'LineWidth', 2, 'DisplayName', 'Mag X'); hold on

legend

%% Plot Values
figure(1)
clf
plot(t_plot, p_est, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Error (m)")
title("Position vs. Time")

hold on;
% plot(data_gps.time, data_gps.data(:, 3), 'LineWidth', 2, 'DisplayName', 'GPS Alt');
% plot(data_baro.time, data_baro.data, 'LineWidth', 2, 'DisplayName', 'Baro Alt')

figure(2)
clf
plot(t_plot, v_est, 'LineWidth', 2)
legend("V_0^E", "V_1^E", "V_2^E")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("ECEF Velocity vs. Time")

figure(3)
clf
plot(t_plot, e_est, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Error")
title("Quaternion vs. Time")

figure(4)
clf
plot(t_plot, w_est, 'LineWidth', 2)
legend("w_0^B", "w_1^B", "w_2^B")
xlabel("Time (s)")
ylabel("Error (rad/s)")
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
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 2")

sgtitle("Gyro Bias Estimate Errors")

figure(7)
clf
plot(t_plot, v_est);

%% ANIMATION

figure(8)
for n = 1:10:num_steps
    poseplot(quaternion(e_est(n, 4), e_est(n, 1), e_est(n, 2), e_est(n, 3)))
    drawnow
end