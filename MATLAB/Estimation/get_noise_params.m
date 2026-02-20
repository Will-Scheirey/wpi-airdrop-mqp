function [R, Q, P0] = get_noise_params(sensor_var, dt)

%% Noise Parameters
Rp = 10;
R_pos = eye(3) * Rp^2;

% sensor_var.mag = sensor_var.mag * 1e1;
% sensor_var.baro = sensor_var.baro * 1e1;
% sensor_var.gyro = sensor_var.gyro * 1e1;
% sensor_var.accel = sensor_var.accel * 1e1;
% % sensor_var.mag = sensor_var.mag * 1e1;

R_mag = blkdiag(sensor_var.mag(1), sensor_var.mag(2), sensor_var.mag(3)).^2;

R_baro = (sensor_var.baro * 1e2)^2;

Rv = 2;
R_vel = eye(3) * Rv^2;

R = blkdiag( ...
    R_pos, ...
    R_mag, ...
    R_baro, ...
    R_vel ...
    );

% accel noise covariance
sigma_a = sqrt(sensor_var.accel(:)) * 1e3;
Sa = diag(sigma_a.^2);

Q_PV = [ (dt^4/4)*Sa, (dt^3/2)*Sa;
         (dt^3/2)*Sa, (dt^2)*Sa ];

sigma_w = sqrt(norm(sensor_var.gyro));
Q_e = eye(4) * (dt*sigma_w)^2;

Qwb = 1e-5;
Q_wb = eye(3) * Qwb^2;

Qab = 1e-5;
Q_ab = eye(3) * Qab^2;

Qpb = 1e-5;
Q_pb = eye(3) * Qpb^2;

sigma_bm = 1e-2;
Q_mb = eye(3) * (sigma_bm^2);

sigma_bb = 1e-2;
Q_bb = sigma_bb^2;

sigma_vv = 1e-5;
Q_vv = sigma_vv^2 * eye(3);

Q = blkdiag(...
    Q_PV, ... % P
    ... % Q_V,... % V
    Q_e, ... % e
    Q_wb, ... % w
    Q_ab, ... % ab
    Q_pb, ...
    Q_mb, ...
    Q_bb, ...
    Q_vv ...
    );

P0 = blkdiag( ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2 * eye(4), ...
    1e-2 * eye(3), ...
    1e-2 * eye(3), ...
    1e-2* eye(3), ...
    1e-2 * eye(3), ...
    1e-2, ...
    1e-2 * eye(3) ...
    );

Q = (Q + Q.')/2;                     % enforce symmetry
q_floor = 1e-12;                     % pick a floor in variance units
Q = Q + q_floor * eye(size(Q));      % jitter to make invertible

end