clearvars -except t y model x_actual tspan; clc; close all

num_sec = 10;
meas_freq = 100; % Number of measurements per second
num_steps = num_sec * meas_freq + 1;

% Only re-run the simulation if we need to
run_sim = false;
if ~exist("t", "var")
    run_sim = true;
elseif numel(t) ~= num_steps || t(end) ~= num_sec
    run_sim = true;
end

if run_sim
    disp("Running sim")
    tspan = linspace(0, num_sec, num_steps);
    [t, y, model] = propagate_model('tspan', tspan, 'riser', true, 'model', @Parachute_Model_Simple);
    x_actual = y(1:end-1, :);
end

[p_actual, v_actual, a_actual, e_actual, w_actual, alpha_actual] = get_model_property(t, y, model, 'P_p', 'V_p', 'a_p_curr', 'e_p', 'w_p', 'alpha_p_curr');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
mag_std_dev = sensor.mag_std_dev;
w_bias_std_dev = 0;

a_corr = correct_meas_accel(a_actual, v_actual, w_actual, e_actual, alpha_actual);

a_meas = sensor_noise_white(a_corr, a_std_dev);

w_bias = w_bias_std_dev * randn(3, 1);

w_meas = sensor_noise_white(w_actual, w_std_dev);

w_meas(:, 1) = w_meas(:, 1) + w_bias(1);
w_meas(:, 2) = w_meas(:, 2) + w_bias(2);
w_meas(:, 3) = w_meas(:, 3) + w_bias(3);

p_meas = sensor_noise_white(p_actual, p_std_dev);
lla_worc = [42.2741, -71.8080, 100];
pos_lla = enu2lla(p_meas, lla_worc, "flat");

mag_XYZ  = zeros(num_steps, 3);
mag_actual = zeros(num_steps, 3);
v_e_actual = zeros(num_steps, 3);
for n = 1:num_steps
    XYZ = wrldmagm(pos_lla(n, 3), pos_lla(n, 1), pos_lla(n, 2), 2025);
    XYZ = XYZ / norm(XYZ);

    mag_XYZ(n, :) = XYZ;
    C_EB = ecef2body_rotm(e_actual(n, :)); % Body -> inertial

    mag_actual(n, :) = (C_EB' * XYZ);
    v_e_actual(n, :) = (C_EB * v_actual(n, :)');
end

m_meas = sensor_noise_white(mag_actual, mag_std_dev);

v_meas = sensor_noise_white(v_e_actual, p_std_dev);

data_accel = table(tspan', a_meas, repmat(-1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_gyro  = table(tspan', w_meas, repmat(-1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});

data_gps   = table(tspan', p_meas, repmat(1, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_mag   = table(tspan', m_meas, repmat(2, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});
data_vel   = table(tspan', v_meas, repmat(4, num_steps, 1), 'VariableNames', {'time', 'data', 'meas_idx'});

measurements = {data_gps, data_mag, data_vel};

inputs = table(tspan', data_accel.data, data_gyro.data, ...
    'VariableNames', {'time', 'accel', 'gyro'});

%% Set up the Kalman Filter
num_steps = numel(t);

%% Run the Kalman Filter
% The Kalman Filter
dt = t(2) - t(1);

sensor_var = struct( ...
    'accel', repmat(a_std_dev, 1, 3), ...
    'gyro',  repmat(w_std_dev, 1, 3), ...
    'gps',   repmat(p_std_dev, 1, 3), ...
    'mag',   repmat(mag_std_dev, 1, 3), ...
    'baro',  p_std_dev);

[R, Q, P0] = get_noise_params(sensor_var, dt);

kf = Airdrop_EKF(R, Q, 0, P0, dt);

kf.initialize(true, ...
    [9.81, 0, 0]', ...
    [0, 0, 0]', ...
    data_mag.data(1, :)', ...
    data_gps.data(1, :)', ...
    data_gps.data(1, 3)', ...
    e_actual(1, :), ... 
    v_e_actual(1, :) ...
    )

acc_gps = repmat(p_std_dev, num_steps, 2);

kf.run_filter(measurements, inputs, t, acc_gps, 0, 1, true);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_truth = p_actual(1:end-1, :);

v_truth_b = v_actual(1:end-1, :);

e_truth = e_actual(1:end-1, :);
w_truth = w_actual(1:end-1, :);

for i = 1:height(v_truth_b)
    v_truth(i, :) = ecef2body_rotm(e_truth(i, :)) * v_truth_b(i, :)';
end

x_truth = [p_truth, v_truth, e_truth, w_truth];

p_est = x_est(:,  kf.x_inds.P_E);
v_est = x_est(:,  kf.x_inds.V_E);
e_est = x_est(:,  kf.x_inds.e);

p_err = p_est - p_truth;
v_err = v_est - v_truth;
e_err = e_est - e_truth;

w_bias_err = w_bias' - x_est(:, kf.x_inds.b_g);

x_err = [p_err, v_err, e_err, w_bias_err];

t_plot = t(1:end-1);

%% Plot Values
figure(1)
clf
plot(t_plot, p_err, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Error (m)")
title("Position Error vs. Time")

figure(2)
clf
plot(t_plot, v_err, 'LineWidth', 2)
legend("V_0^E", "V_1^E", "V_2^E")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("ECEF Velocity Error vs. Time")

figure(3)
clf
plot(t_plot, e_err, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Error")
title("Quaternion Error vs. Time")

figure(4)
clf
plot(t_plot, x_est(:, kf.x_inds.b_m), 'LineWidth', 2)
title("Mag Bias Estimate")

figure(5)
clf
plot(tspan, mag_actual, 'LineWidth', 2)
title("Mag Measurement")

figure(6)
clf

subplot(3, 1, 1)
plot(t_plot, w_bias_err(:, 1), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 0")

subplot(3, 1, 2)
plot(t_plot, w_bias_err(:, 2), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on

legend
xlabel("Time (s)")
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 1")

subplot(3, 1, 3)
plot(t_plot, w_bias_err(:, 3), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 2")

sgtitle("Gyro Bias Estimate Errors")

figure(7)
clf
plot(t_plot, v_est);

figure(8)
clf
plot(t_plot, e_truth);
title("Actual Quaternion Parts")

figure(9)
clf
plot(t_plot, e_est);
title("Estimated Quaternion Parts")



function plot_cov(err, cov, t)
    plot(t, err, '-b', 'LineWidth', 1.5, 'DisplayName', 'Error'); hold on
    plot(t, sqrt(cov), '--r', 'LineWidth', 1, 'DisplayName', 'Covariance');
    plot(t, -sqrt(cov), '--r', 'LineWidth', 1, 'HandleVisibility', 'off');
end