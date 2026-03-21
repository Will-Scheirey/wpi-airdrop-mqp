clearvars -except t y model x_actual tspan; clear; clc;
clear; clc

% Basic sim timing
num_sec = 50;
meas_freq = 20; % measurements per second
num_steps = num_sec * meas_freq + 1;

% Only rerun if something changed or doesnt exist
run_sim = false;
if ~exist("t", "var")
    run_sim = true;
elseif numel(t) ~= num_steps || t(end) ~= num_sec
    run_sim = true;
end

if run_sim
    disp("Running sim")
    tspan = linspace(0, num_sec, num_steps);
    [t, y, model] = propagate_model('tspan', tspan, 'riser', true, 'model', @Parachute_Model_Simple, 'use_drag', true);
    x_actual = y(1:end-1, :);
end

% Use sim time everywhere
tspan = t';
num_steps = length(tspan);

% Truth states from model
[p_actual, v_actual, a_actual, e_actual, w_actual, alpha_actual] = get_model_property(t, y, model, 'P_p', 'V_p', 'a_p_curr', 'e_p', 'w_p', 'alpha_p_curr');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);

% Noise levels
p_std_dev = sensor.gps_std_dev;
v_std_dev = 3;
baro_std_dev = sensor.baro_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
mag_std_dev = sensor.mag_std_dev;
w_bias_std_dev = 0;

% Accel gets corrected first then noise
a_corr = correct_meas_accel(a_actual, v_actual, w_actual, e_actual, alpha_actual);
a_meas = sensor_noise_white(a_corr, a_std_dev);

% Constant gyro bias + noise
w_bias = w_bias_std_dev * randn(3, 1);
w_meas = sensor_noise_white(w_actual, w_std_dev);

w_meas(:, 1) = w_meas(:, 1) + w_bias(1);
w_meas(:, 2) = w_meas(:, 2) + w_bias(2);
w_meas(:, 3) = w_meas(:, 3) + w_bias(3);

% GPS position
p_meas = sensor_noise_white(p_actual, p_std_dev);

% Just keeping this around if needed
lla_worc = [42.2741, -71.8080, 100];
pos_lla = enu2lla(p_meas, lla_worc, "flat");

mag_XYZ  = zeros(num_steps, 3);
mag_actual = zeros(num_steps, 3);
v_e_actual = zeros(num_steps, 3);

% Simple reference mag field
m_enu = [1; 0; 0];

for n = 1:num_steps
    mag_XYZ(n, :) = m_enu;

    C_EB = body2enu_rotm(e_actual(n, :));

    % Rotate into body frame
    mag_actual(n, :) = (C_EB' * m_enu);

    % Velocity in ENU
    v_e_actual(n, :) = (C_EB * v_actual(n, :)');
end

m_meas = sensor_noise_white(mag_actual, mag_std_dev);
v_meas = sensor_noise_white(v_e_actual, v_std_dev);

% Pack into tables for filter
data_accel = table(tspan', a_meas, repmat(-1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_gyro  = table(tspan', w_meas, repmat(-1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});

data_gps   = table(tspan', p_meas, repmat(1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_mag   = table(tspan', m_meas, repmat(2, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_vel   = table(tspan', v_meas, repmat(4, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});

% Only using gps + mag for now
measurements = {data_gps, data_mag};

% Inputs are imu streams
inputs = table(tspan', data_accel.data, data_gyro.data, ...
    'VariableNames', {'time', 'accel', 'gyro'});

%% Set up the Kalman Filter
num_steps = numel(t);

%% Run the Kalman Filter

% Run filter faster than sim
kf_dt_div = 3;

dt = t(2) - t(1);
dt_kf = dt / kf_dt_div;

% Variances grouped
sensor_var = struct( ...
    'accel', repmat(a_std_dev.^2, 1, 3), ...
    'gyro',  repmat(w_std_dev.^2, 1, 3), ...
    'gps',   repmat(p_std_dev.^2, 1, 3), ...
    'mag',   repmat(mag_std_dev.^2, 1, 3), ...
    'vel',   repmat(v_std_dev.^2, 1, 3), ...
    'baro',  baro_std_dev.^2 ...
);

[R, Q, P0] = get_noise_params_sim(sensor_var, dt_kf);

kf = Airdrop_EKF(R, Q, P0, dt_kf);

% Initialize from truth
kf.initialize(true, ...
     a_actual(1,:), ...
    [0, 0, 0]', ...
    data_mag.data(1, :)', ...
    data_gps.data(1, :)', ...
    data_gps.data(1, 3)', ...
    e_actual(1, :), ... 
    v_e_actual(1, :) ...
    )

% GPS accuracy placeholder
acc_gps = repmat(p_std_dev, num_steps, 2);

t_kf = 0:dt_kf:t(end);

kf.run_filter(measurements, inputs, t_kf, acc_gps, 100, 1, true);

% Downsample back to sim grid
x_est_temp = kf.x_hist(:, 1:kf_dt_div:end);
x_est = x_est_temp';
covariances = kf.P_hist;

%% Extract Values

% Trim truth
p_truth = p_actual(1:end-1, :);

v_truth_b = v_actual(1:end-1, :);

e_truth = e_actual(1:end-1, :);
w_truth = w_actual(1:end-1, :);

% Convert velocity to ENU
for i = 1:height(v_truth_b)
    v_truth(i, :) = body2enu_rotm(e_truth(i, :)) * v_truth_b(i, :)';
end

x_truth = [p_truth, v_truth, e_truth, w_truth];

% Pull estimates
p_est = x_est(:,  kf.x_inds.P_E);
v_est = x_est(:,  kf.x_inds.V_E);
e_est = x_est(:,  kf.x_inds.e);

% Errors
p_err = p_est - p_truth;
v_err = v_est - v_truth;
e_err = e_est - e_truth;

% Gyro bias error
w_bias_err = w_bias' - x_est(:, kf.x_inds.b_g);

x_err = [p_err, v_err, e_err, w_bias_err];

t_plot = t(1:end-1);

%% Plot Values
fig_idx = new_fig(1);
clf
plot(t_plot, p_err, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Error (m)")
title("Position Error vs Time")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, v_err, 'LineWidth', 2)
legend("V_0^E", "V_1^E", "V_2^E")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("ECEF Velocity Error vs Time")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, e_err, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Error")
title("Quaternion Error vs Time")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, x_est(:, kf.x_inds.b_m), 'LineWidth', 2)
title("Mag Bias Estimate")

fig_idx = new_fig(fig_idx);
clf
plot(tspan, data_mag.data, 'LineWidth', 2)
title("Mag Measurement")

fig_idx = new_fig(fig_idx);
clf

subplot(3, 1, 1)
plot(t_plot, w_bias_err(:, 1), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Bias Error")
title("Gyro Axis 0")

subplot(3, 1, 2)
plot(t_plot, w_bias_err(:, 2), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Bias Error")
title("Gyro Axis 1")

subplot(3, 1, 3)
plot(t_plot, w_bias_err(:, 3), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Bias Error")
title("Gyro Axis 2")

sgtitle("Gyro Bias Errors")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, v_est)
title("Velocity Estimates")
legend("X", "Y", "Z")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, e_truth)
title("Actual Quaternion")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, e_est)
title("Estimated Quaternion")

fig_idx = new_fig(fig_idx);
clf
plot(tspan, data_accel.data)
title("Acceleration Measurement")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, x_est(:, kf.x_inds.b_a))
title("Accel Bias Estimate")

fig_idx = new_fig(fig_idx);
clf
plot(tspan, data_gyro.data)
title("Gyro Measurement")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, x_est(:, kf.x_inds.b_g))
title("Gyro Bias Estimate")

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(tspan, data_gps.data(:, 1), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.P_E(1)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("X Pos")

subplot(3,1,2)
plot(tspan, data_gps.data(:, 2), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.P_E(2)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("Y Pos")

subplot(3,1,3)
plot(tspan, data_gps.data(:, 3), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.P_E(3)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("Z Pos")

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(tspan, data_vel.data(:, 1), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.V_E(1)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("X Vel")

subplot(3,1,2)
plot(tspan, data_vel.data(:, 2), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.V_E(2)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("Y Vel")

subplot(3,1,3)
plot(tspan, data_vel.data(:, 3), '.-r', 'MarkerSize', 10); hold on
plot(t_plot, x_est(:, kf.x_inds.V_E(3)), '-b', 'LineWidth', 1)
legend("Measurement", "Estimate")
title("Z Vel")

fig_idx = new_fig(fig_idx);
plot(t_plot, x_est(:, kf.x_inds.b_v), 'LineWidth', 1)
title("Velocity Bias Estimate")

idx = kf.x_inds.e(1);

fig_idx = new_fig(fig_idx);
clf
p = plot_cov(e_err(:, 1), squeeze(covariances(idx, idx, 1:kf_dt_div:end)), t_plot);
p.MarkerSize = 10;
p.LineWidth = 1;
legend
ylabel("Error")
xlabel("Time (s)")
xlim([t_plot(1), t_plot(end)])
title("Quaternion Part 0 Error and Covariance")