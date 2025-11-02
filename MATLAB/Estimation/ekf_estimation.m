clear; clc; close all

num_sec = 100;
meas_freq = 20; % Number of measurements per second
tspan = linspace(0, num_sec, num_sec * meas_freq + 1);
[t, y, model] = propagate_model('tspan', tspan, 'riser', true);
x_actual = y(1:end-1, :);

%% Extract Model Properties

[p_actual, v_g_actual, a_actual, e_actual, w_actual, alpha_actual] = get_model_property(t, y, model, 'P_p', 'V_p', 'a_p_curr', 'e_p', 'w_p', 'alpha_p_curr');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
e_std_dev = sensor.mag_std_dev;
v_g_std_dev = sensor.grnd_vel_std_dev;

a_corr = correct_meas_accel(a_actual, v_g_actual, w_actual, e_actual, alpha_actual);
a_meas = sensor_noise_white(a_corr, a_std_dev);

w_meas = sensor_noise_white(w_actual, w_std_dev);

p_meas = sensor_noise_white(p_actual, p_std_dev);

e_meas = sensor_noise_white(e_actual, e_std_dev);

v_g_mag_actual = vecnorm(v_g_actual(:, 1:2), 2, 2);
v_g_mag_meas = sensor_noise_white(v_g_mag_actual, v_g_std_dev);

measurements = [p_meas, e_meas, w_meas, v_g_mag_meas]';

state_idx = 4;

%% Set up the Kalman Filter
num_steps = numel(t);

x0 = [
    y(1, 1:13)';
    zeros(3, 1);
    0;
    0
    ];

tau_d   = [7e0; 1e1; 1e1];
sigma_d = 5e1;

R = blkdiag( ...
    (p_std_dev^2) * eye(3), ...
    (e_std_dev^2) * eye(4), ...
    (w_std_dev^2) * eye(3),  ...
    (v_g_std_dev^2) ...
    );

Q_P = [
    1e-5, 0,  0;
    0,    1e-5, 0;
    0,    0,    1e-5;
];

cross_term = 1e-3;

Q_V = [
    1e-1,           cross_term,  cross_term;
    cross_term,    1e-1,        cross_term
    cross_term,    cross_term,  1e-1
];

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    1e-3 * eye(4), ... % e
    1e-1 * eye(3),... % w
    sigma_d^2 * eye(3), ...
    0*10^2 * eye(2)...
    );

P0 = blkdiag( ...
    1e1 * eye(3), ...
    1e2 * eye(3), ...
    1e-3 * eye(4), ...
    1e-3 * eye(3), ...
    sigma_d^2 * eye(3), ...
    0*10^2 * eye(2)...
    );

%% Run the Kalman Filter
% The Kalman Filter
dt = t(2) - t(1);
kf = EKF_Wind(R, Q, x0, 0, P0, dt, model.payload.I(), tau_d);
% kf = EKF_No_Dynamics(R, Q, x0, 0, P0, t(2) - t(1));

kf.run_filter(measurements, a_meas', num_steps);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_truth = p_actual(1:end-1, :);
v_g_truth = v_g_actual(1:end-1, :);
e_truth = e_actual(1:end-1, :);
w_truth = w_actual(1:end-1, :);
x_truth = [p_truth, v_g_truth, e_truth, w_truth];

p_est = x_est(:, 1:3);
v_g_est = x_est(:, 4:6);
e_est = x_est(:, 7:10);
w_est = x_est(:, 11:13);
wind_est = x_est(:, 17:18);

p_err = p_est - p_truth;
v_g_err = v_g_est - v_g_truth;
e_err = e_est - e_truth;
w_err = w_est - w_truth;

x_err = [p_err, v_g_err, e_err, w_err];

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
plot(t_plot, v_g_err, 'LineWidth', 2)
legend("V_0^B", "V_1^B", "V_2^B")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("Body Velocity Error vs. Time")

V_e_est = zeros(num_steps-1, 3);
V_e_truth = zeros(num_steps-1, 3);

for i = 1:num_steps-1
    e_e = e_est(i, :);
    V_e_est(i, :) = ecef2body_rotm(e_e)' * v_g_est(i, :)';

    e_r = e_truth(i, :);
    V_e_truth(i, :) = ecef2body_rotm(e_r)' * v_g_truth(i, :)';
end

V_e_err = V_e_est - V_e_truth;

figure(3)
clf
plot(t_plot, e_err, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Error")
title("Quaternion Error vs. Time")

figure(4)
clf
plot(t_plot, w_err, 'LineWidth', 2)
legend("w_0^B", "w_1^B", "w_2^B")
xlabel("Time (s)")
ylabel("Error (rad/s)")
title("Body Angular Velocity Error vs. Time")

figure(5)
clf
plot_cov(x_err(:, state_idx), squeeze(covariances(state_idx, state_idx, 1:end-1)), t_plot)

function plot_cov(err, cov, t)
    plot(t, err, '-b', 'LineWidth', 1.5, 'DisplayName', 'Error'); hold on
    plot(t, sqrt(cov), '--r', 'LineWidth', 1, 'DisplayName', 'Covariance');
    plot(t, -sqrt(cov), '--r', 'LineWidth', 1, 'HandleVisibility', 'off');
end

figure(6)
clf
plot(t_plot, wind_est)

return

