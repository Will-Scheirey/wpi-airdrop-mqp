clearvars -except t y model x_actual; clc; close all

num_sec = 100;
meas_freq = 20; % Number of measurements per second
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
    [t, y, model] = propagate_model('tspan', tspan, 'riser', true);
    x_actual = y(1:end-1, :);
end

[p_actual, v_actual, a_actual, e_actual, w_actual, alpha_actual] = get_model_property(t, y, model, 'P_p', 'V_p', 'a_p_curr', 'e_p', 'w_p', 'alpha_p_curr');

%% Create Sensor Measurements
sensor = Sensor_FlySight(meas_freq);
p_std_dev = sensor.gps_std_dev;
a_std_dev = sensor.accel_std_dev;
w_std_dev = sensor.gyro_std_dev;
e_std_dev = sensor.mag_std_dev;

a_corr = correct_meas_accel(a_actual, v_actual, w_actual, e_actual, alpha_actual);

a_meas = sensor_noise_white(a_corr, a_std_dev);

w_bias = 5 * randn(3, 1);

w_meas = sensor_noise_white(w_actual, w_std_dev);

w_meas(:, 1) = w_meas(:, 1) + w_bias(1);
w_meas(:, 2) = w_meas(:, 2) + w_bias(2);
w_meas(:, 3) = w_meas(:, 3) + w_bias(3);

p_meas = sensor_noise_white(p_actual, p_std_dev);

e_meas = sensor_noise_white(e_actual, e_std_dev);

measurements = [p_meas, e_meas, w_meas]';

state_idx = 13;

%% Set up the Kalman Filter
num_steps = numel(t);

x0 = [
    y(1, 1:3)';

    ecef2body_rotm(y(1, 7:10)')' * y(1, 4:6)'

    y(7:13)';
    zeros(3,1);
    ];

R = blkdiag( ...
    (p_std_dev^2) * eye(3), ...
    (e_std_dev^2) * eye(4), ...
    (w_std_dev^2) * eye(3)  ...
    );

Q_P = [
    1e-5, 0,  0;
    0,    1e-5, 0;
    0,    0,    1e-5;
];

cross_term = 1e-10;
diag_term = 1e-1;

Q_V = [
    diag_term,           cross_term,  cross_term;
    cross_term,    diag_term,        cross_term
    cross_term,    cross_term,  diag_term
];

Q = blkdiag(...
    Q_P,... % P
    Q_V,... % V
    1e-3 * eye(4), ... % e
    1e-3 * eye(3), ... % w
    1e-10 * eye(3) ...
    );

P0 = blkdiag( ...
    1e1 * eye(3), ...
    1e2 * eye(3), ...
    1e-3 * eye(4), ...
    1e-3 * eye(3), ...
    1e3 * eye(3) ...
    );

%% Run the Kalman Filter
% The Kalman Filter
dt = t(2) - t(1);
kf = EKF_V_E(R, Q, x0, 0, P0, dt, model.payload.I());
% kf = EKF_No_Dynamics(R, Q, x0, 0, P0, t(2) - t(1));

kf.run_filter(measurements, a_meas', num_steps);

x_est = kf.x_hist(:, 2:end)';
covariances = kf.P_hist;

%% Exract Values

p_truth = p_actual(1:end-1, :);

v_truth_b = v_actual(1:end-1, :);

e_truth = e_actual(1:end-1, :);
w_truth = w_actual(1:end-1, :);


for i = 1:height(v_truth_b)
    v_truth(i, :) = ecef2body_rotm(e_truth(i, :))' * v_truth_b(i, :)';
end

% v_truth = v_actual(1:end-1, :);

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
legend("V_0^E", "V_1^E", "V_2^E")
xlabel("Time (s)")
ylabel("Error (m/s)")
title("ECEF Velocity Error vs. Time")

return

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

figure(6)
clf

subplot(3, 1, 1)
plot(t_plot, x_est(:, 14) - w_bias(1), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on
legend
xlabel("Time (s)")
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 0")

subplot(3, 1, 2)
plot(t_plot, x_est(:, 15) - w_bias(2), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on

legend
xlabel("Time (s)")
ylabel("Bias Error (rad/s)")
sgtitle("Gyro Axis 1")

subplot(3, 1, 3)
plot(t_plot, x_est(:, 16) - w_bias(3), 'DisplayName', 'Estimate Error', 'LineWidth', 1.5); hold on
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
plot(t_plot, v_truth)

%% O
% 1) Build "model-predicted" ECEF acceleration using your filter's formula
n = size(a_corr,1);
a_model_e = zeros(n,3);

g_e = [0;0;-9.81];

for i = 1:n
    e   = e_actual(i,:)';
    a_b = a_corr(i,:)';             % what you feed to the EKF

    C_EB = ecef2body_rotm(e)';      % body -> ECEF
    a_model_e(i,:) = (C_EB * a_b + g_e)';   % EKF's view of dV_E/dt
end

% 2) Compute numerical derivative of the true ECEF velocity
vE_truth = zeros(n,3);
for i = 1:n
    vE_truth(i,:) = (ecef2body_rotm(e_actual(i,:)')' * v_actual(i,:)')';
end

dt = t(2) - t(1);
dv_dt_num = diff(vE_truth) / dt;        % size (n-1,3)

% 3) Compare them (truncate a_model_e to same length)
a_model_e_trunc = a_model_e(1:end-1,:);

figure;
subplot(3,1,1)
plot(t(1:end-1), dv_dt_num(:,1), 'k', t(1:end-1), a_model_e_trunc(:,1), '--r');
legend('dVx/dt truth','model'); ylabel('ax_E')

subplot(3,1,2)
plot(t(1:end-1), dv_dt_num(:,2), 'k', t(1:end-1), a_model_e_trunc(:,2), '--r');
legend('dVy/dt truth','model'); ylabel('ay_E')

subplot(3,1,3)
plot(t(1:end-1), dv_dt_num(:,3), 'k', t(1:end-1), a_model_e_trunc(:,3), '--r');
legend('dVz/dt truth','model'); ylabel('az_E'); xlabel('t (s)')


function plot_cov(err, cov, t)
    plot(t, err, '-b', 'LineWidth', 1.5, 'DisplayName', 'Error'); hold on
    plot(t, sqrt(cov), '--r', 'LineWidth', 1, 'DisplayName', 'Covariance');
    plot(t, -sqrt(cov), '--r', 'LineWidth', 1, 'HandleVisibility', 'off');
end