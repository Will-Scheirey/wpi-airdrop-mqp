clear; clc; close all

num_sec = 100;
meas_freq = 20; % Number of measurements per second
tspan = linspace(0, num_sec, num_sec * meas_freq + 1);
[t, y, model] = propagate_model('tspan', tspan, 'riser', true);
x_actual = y(1:end-1, :);

%% Extract Model Properties

[p_actual, v_actual, a_actual, e_actual, w_actual, alpha_actual] = get_model_property(t, y, model, 'P_p', 'V_p', 'a_p_curr', 'e_p', 'w_p', 'alpha_p_curr');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
e_std_dev = sensor.mag_std_dev;

a_corr = correct_meas_accel(a_actual, v_actual, w_actual, e_actual, alpha_actual);
a_meas = sensor_noise_white(a_corr, a_std_dev);

w_meas = sensor_noise_white(w_actual, w_std_dev);

p_meas = sensor_noise_white(p_actual, p_std_dev);

e_meas = sensor_noise_white(e_actual, e_std_dev);

measurements = [p_meas, e_meas, w_meas]';

%% Set up the Kalman Filter
num_steps = numel(t);

x0 = [
    y(1, 1:13)';
    zeros(3, 1)
    ];

tau_d   = 0.7e1;
sigma_d = 5e1;

R = blkdiag( ...
    (p_std_dev^2) * eye(3), ...
    (e_std_dev^2) * eye(4), ...
    (w_std_dev^2) * eye(3)  ...
    );

Q = blkdiag(...
    1e0 * eye(3),... % P
    6e-2 * eye(3),... % V
    1e-3 * eye(4), ... % e
    1e-1 * eye(3),... % w
    sigma_d^2 * eye(3) ...
    );

P0 = blkdiag( ...
    1e-3 * eye(3), ...
    1e-3 * eye(3), ...
    1e-3 * eye(4), ...
    1e-3 * eye(3), ...
    sigma_d^2 * eye(3) ...
    );

%% Run the Kalman Filter
% The Kalman Filter
dt = t(2) - t(1);
kf = EKF_Basic_Kinematics(R, Q, x0, 0, P0, dt, model.payload.I(), tau_d);
% kf = EKF_No_Dynamics(R, Q, x0, 0, P0, t(2) - t(1));

kf.run_filter(measurements, a_meas', num_steps);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_truth = p_actual(1:end-1, :);
v_truth = v_actual(1:end-1, :);
e_truth = e_actual(1:end-1, :);
w_truth = w_actual(1:end-1, :);
x_truth = [p_truth, v_truth, e_truth, w_truth];

p_est = x_est(:, 1:3);
v_est = x_est(:, 4:6);
e_est = x_est(:, 7:10);
w_est = x_est(:, 11:13);

p_err = p_est - p_truth;
v_err = v_est - v_truth;
e_err = e_est - e_truth;
w_err = w_est - w_truth;

x_err = [p_err, v_err, e_err, w_err];

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
legend("V_0^B", "V_1^B", "V_2^B")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("Body Velocity Error vs. Time")

V_e_est = zeros(num_steps-1, 3);
V_e_truth = zeros(num_steps-1, 3);

for i = 1:num_steps-1
    e_e = e_est(i, :);
    V_e_est(i, :) = ecef2body_rotm(e_e)' * v_est(i, :)';

    e_r = e_truth(i, :);
    V_e_truth(i, :) = ecef2body_rotm(e_r)' * v_truth(i, :)';
end

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

state_idx = 6;

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
plot(t_plot, p_est(:, 3), 'DisplayName', 'Estimate'); hold on
plot(t_plot, p_truth(:, 3), 'DisplayName', 'Actual')
legend

return
figure(6)
clf
plot3(p_est(:, 1), p_est(:, 2), p_est(:, 3), '.b', 'DisplayName', 'Estimated', 'LineWidth', 1.5); hold on
plot3(p_actual(:, 1), p_actual(:, 2), p_actual(:, 3), '.r', 'DisplayName', 'Actual', 'LineWidth', 1.5);


legend
return;
figure(1)
clf
subplot(3, 1, 1)
plot(t, p_est(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, p_actual(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_0")
legend

subplot(3, 1, 2)
plot(t, p_est(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, p_actual(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_1")
legend

subplot(3, 1, 3)
plot(t, p_est(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, p_actual(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_2")

legend

figure(2)
clf
subplot(3, 1, 1)
plot(t, v_est(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, v_actual(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_0")
legend

subplot(3, 1, 2)
plot(t, v_est(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, v_actual(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_1")
legend

subplot(3, 1, 3)
plot(t, v_est(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, v_actual(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_2")

legend

return
figure(3)

clf
subplot(2, 2, 1)
plot(t, e_est(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, e_actual(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_0")
legend

subplot(2, 2, 2)
plot(t, e_est(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, e_actual(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_1")
legend

subplot(2, 2, 3)
plot(t, e_est(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, e_actual(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_2")

subplot(2, 2, 4)
plot(t, e_est(4, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, e_actual(:, 4), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_3")

legend

figure(4)
clf
subplot(3, 1, 1)
plot(t, V_e_e(:, 1), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, V_e_r(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Ve_0")
legend

subplot(3, 1, 2)
plot(t, V_e_e(:, 2), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, V_e_r(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Ve_1")
legend

subplot(3, 1, 3)
plot(t, V_e_e(:, 3), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, V_e_r(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Ve_2")

legend

figure(5)
clf
plot(t, vecnorm(v_est(:, :), 2, 1), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, vecnorm(v_actual(:, :), 2, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Velocity Norm")

figure(7)
clf
subplot(3, 1, 1)
plot(t, w_est(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 1), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_actual(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_0")
legend

subplot(3, 1, 2)
plot(t, w_est(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 2), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_actual(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_1")
legend

subplot(3, 1, 3)
plot(t, w_est(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 3), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_actual(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_2")

legend
