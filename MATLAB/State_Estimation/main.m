clear; clc; close all;
% Standard deviations for emulated sensor noise
std_dev_accel = 1e-3;
std_dev_pos = 1;

dt = 0.1; % Timestep [s]
num_steps = 100;

% Process Noise
Q = [
    0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    ];

% Measurement error covariance
R = [
    std_dev_accel^2, 0;
    0,               std_dev_pos^2
    ];

% State transition matrix
F = [
    1, dt, dt^2/2;
    0, 1,  dt;
    0, 0,  1
    ];

% Initial conditions
x0 = [0; 10; 1];

% Model propagation
values = zeros(3, num_steps);
values(:, 1) = x0;

for i = 2:num_steps
    values(:, i) = F * values(:, i-1);
end

% Sensitivity matrix (maps measurements to states)
H = [
    0, 0, 1; % First measurement maps to acceleration
    1, 0, 0 % Second measurement maps to position
    ];

% Initial covariance matrix
P0 = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 1
    ] * 10;

% Generated sensor noise
noise_accel = std_dev_accel * randn(1, num_steps);
noise_pos   = std_dev_pos * randn(1, num_steps);

% Emulated sensor measurements
measurements = zeros(2, num_steps);
measurements(1, :) = values(3, :) + noise_accel;
measurements(2, :) = values(1, :) + noise_pos;

% The Kalman Filter
kf = KalmanFilter(R, Q, H, F, x0, P0);

% For historical dat
x_estimates = zeros(3, num_steps);
covariances = zeros(1, num_steps);

% The state we want to track
state_idx = 1;

% Kalman Filter propagation
for i=1:num_steps
    x_estimates(:, i) = kf.x_curr;
    covariances(i) = kf.P_curr(state_idx,state_idx);

    kf.stepFilter(measurements(:, i));
end

%{
% Estimate and plot errors and covariance
state_err = x_estimates(state_idx, 2:end) - values(state_idx, 1:end-1);

plot(state_err, '-r', 'DisplayName', 'Error', 'LineWidth', 1.5); hold on;
plot(sqrt(covariances), '-b', 'DisplayName', 'Covariance', 'LineWidth', 1.5);
plot(-sqrt(covariances), '-b', 'HandleVisibility', 'off', 'LineWidth', 1.5);

legend;
%}

plot(x_estimates(1, 2:end)); hold on
plot(values(1, 1:end-1));