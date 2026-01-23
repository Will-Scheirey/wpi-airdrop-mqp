clear; clc; close all

num_sec = 10;
meas_freq = 100; % Number of measurements per second
tspan = linspace(0, num_sec, num_sec * meas_freq);
[t, y, model] = propagate_model('tspan', tspan);

%% Extract Model Properties

a_p     = get_model_property(t, y, model, 'a_p_curr');
alpha_p = get_model_property(t, y, model, 'alpha_p_curr');
w_p     = get_model_property(t, y, model, 'w_p');
e_p     = get_model_property(t, y, model, 'e_p');

%% Create Sensor Measurements

accel_std_dev = 1;
gyro_std_dev = 0.1;

a_meas = correct_meas_accel(a_p, w_p, e_p, alpha_p);
a_meas = sensor_noise_white(a_meas, accel_std_dev);

w_meas = sensor_noise_white(w_p, gyro_std_dev);

measurements = [a_meas, w_meas]';

num_steps = numel(t);

%% Integrate Sensor Measurements

e = zeros(4, num_steps);
v   = zeros(3, num_steps);
p   = zeros(3, num_steps);

e(:,1)  = y(1,7:10)';
p(:, 1) = y(1,1:3)';
v(:, 1) = y(1,4:6)';

g_vec_e = [0; 0; -9.8];

for i = 2:num_steps
    dt = t(i) - t(i-1);
    
    w = w_meas(i, :);
    a = a_meas(i, :);

    e_dot   = -1/2 * quat_kinematic_matrix(w) * e(:, i-1); % Quaternion rates
    e(:,i) = e(:,i-1) + e_dot*dt;
    e(:,i) = e(:, i) / norm(e(:, i));

    C_EB = ecef2body_rotm(e(:, i));

    v_dot   = a' + C_EB * g_vec_e;
    p_dot   = C_EB' * v(:, i-1);

    v(:, i)   = v  (:, i-1) + v_dot   * dt; % integrate raw accel
    p(:, i)   = p  (:, i-1) + p_dot   * dt; % integrate vel
end

%% Plot

figure(1)
clf
plot3(p(1, :), p(2, :), p(3, :), 'DisplayName', 'Integrated', 'LineWidth', 1.5); hold on
plot3(y(:, 1), y(:, 2), y(:, 3), 'DisplayName', 'Actual', 'LineWidth', 1.5);

legend

figure(1)
clf
subplot(3, 1, 1)
plot(t, p(1, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 1), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_0")
legend

subplot(3, 1, 2)
plot(t, p(2, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 2), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_1")
legend

subplot(3, 1, 3)
plot(t, p(3, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 3), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("P_2")

legend

figure(2)
clf
subplot(3, 1, 1)
plot(t, v(1, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 4), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_0")
legend

subplot(3, 1, 2)
plot(t, v(2, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 5), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_1")
legend

subplot(3, 1, 3)
plot(t, v(3, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 6), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("Vb_2")

legend

figure(3)

clf
subplot(2, 2, 1)
plot(t, e(1, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 7), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_0")
legend

subplot(2, 2, 2)
plot(t, e(2, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 8), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_1")
legend

subplot(2, 2, 3)
plot(t, e(3, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 9), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_2")

subplot(2, 2, 4)
plot(t, e(4, :), 'DisplayName', 'Integrated', 'LineWidth', 3); hold on
plot(t, y(:, 10), 'DisplayName', 'Actual', 'LineWidth', 3, 'LineStyle', ':');
title("e_3")

legend
