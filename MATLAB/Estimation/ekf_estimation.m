clear; clc; close all

num_sec = 100;
meas_freq = 20; % Number of measurements per second
tspan = linspace(0, num_sec, num_sec * meas_freq + 1);
[t, y, model] = propagate_model('tspan', tspan);

model_inds = struct(...
    'p', 1:3,...
    'v', 4:6,...
    'e', 7:10 ...
    );

%% Extract Model Properties

[p_p, a_p, alpha_p, w_p, e_p] = get_model_property(t, y, model, 'P_p', 'a_p_curr', 'alpha_p_curr', 'w_p', 'e_p');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
e_std_dev = sensor.mag_std_dev;

% a_meas = correct_meas_accel(a_p, w_p, e_p, alpha_p);
% a_meas = sensor_noise_white(a_meas, a_std_dev);

a_meas = sensor_noise_white(a_p, a_std_dev);

w_meas = sensor_noise_white(w_p, w_std_dev);

p_meas = sensor_noise_white(p_p, p_std_dev);

e_meas = sensor_noise_white(e_p, e_std_dev);

measurements = [p_meas, e_meas, w_meas]';

%% Set up the Kalman Filter
num_steps = numel(t);

x0 = [
    y(1, 1:6)';
    y(1, 7:13)';
];

R = blkdiag( ...
    (p_std_dev^2) * eye(3), ...
    (e_std_dev^2) * eye(4), ...
    (w_std_dev^2) * eye(3)  ...
    );

Q = blkdiag(...
    1e-1 * eye(3),... % P
    1e0 * eye(3),... % V
    1e-3 * eye(3),... % w
    1e-1 * eye(4) ... % e
    );

P0 = blkdiag( ...
    (p_std_dev^2) * eye(3), ...
    Q(4:6, 4:6), ...
    (e_std_dev^2) * eye(4), ...
    (w_std_dev^2) * eye(3)  ...
    );

%% Run the Kalman Filter
% The Kalman Filter
kf = EKF_No_Dynamics(R, Q, x0, 0, P0, t(2) - t(1));

kf.run_filter(measurements, a_meas', num_steps);

x_estimates = kf.x_hist;
covariances = kf.P_hist;

p = x_estimates(1:3, :);
v = x_estimates(4:6, :);
e = x_estimates(7:10, :);
w = x_estimates(11:13, :);

% y = y';

% errors = x_estimates - y;

%% Plot

figure(1)
clf
plot3(p(1, :), p(2, :), p(3, :), 'DisplayName', 'Estimated', 'LineWidth', 1.5); hold on
plot3(y(:, 1), y(:, 2), y(:, 3), 'DisplayName', 'Actual', 'LineWidth', 1.5);

legend

figure(1)
clf
subplot(3, 1, 1)
plot(t, p(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_0")
legend

subplot(3, 1, 2)
plot(t, p(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_1")
legend

subplot(3, 1, 3)
plot(t, p(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_2")

legend

figure(2)
clf
subplot(3, 1, 1)
plot(t, v(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 4), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_0")
legend

subplot(3, 1, 2)
plot(t, v(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 5), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_1")
legend

subplot(3, 1, 3)
plot(t, v(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 6), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_2")

legend

figure(3)

clf
subplot(2, 2, 1)
plot(t, e(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 7), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_0")
legend

subplot(2, 2, 2)
plot(t, e(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 8), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_1")
legend

subplot(2, 2, 3)
plot(t, e(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 9), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_2")

subplot(2, 2, 4)
plot(t, e(4, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, y(:, 10), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_3")

legend

V_e_e = zeros(num_steps, 3);
V_e_r = zeros(num_steps, 3);

for i = 1:num_steps
    e_e = x_estimates(7:10, i);
    V_e_e(i, :) = ecef2body_rotm(e_e)' * x_estimates(4:6, i);

    e_r = y(i, 7:10);
    V_e_r(i, :) = ecef2body_rotm(e_r)' * y(i, 4:6)';
end

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
plot(t, vecnorm(v(:, :), 2, 1), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, vecnorm(y(:, 4:6), 2, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Velocity Norm")

figure(7)
clf
subplot(3, 1, 1)
plot(t, w(1, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 1), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_p(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_0")
legend

subplot(3, 1, 2)
plot(t, w(2, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 2), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_p(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_1")
legend

subplot(3, 1, 3)
plot(t, w(3, :), 'DisplayName', 'Estimated', 'LineWidth', 3); hold on
plot(t, w_meas(:, 3), 'DisplayName', 'Measurement', 'LineWidth', 3);
plot(t, w_p(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("wb_2")

legend
