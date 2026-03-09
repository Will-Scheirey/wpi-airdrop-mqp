function [R, Q, P0] = get_noise_params_sim(var, dt)
%GET_NOISE_PARAMS_SIM  Build EKF R, Q, P0 from sensor variances.
%
% Inputs (all are variances, not sigmas):
%   var.accel : 1x3 accel variance in (m/s^2)^2, body frame
%   var.gyro  : 1x3 gyro variance  in (rad/s)^2
%   var.gps   : 1x3 GPS position variance in m^2
%   var.vel   : 1x3 GPS/ECEF velocity variance in (m/s)^2
%   var.mag   : 1x3 mag variance (whatever units you're using)^2
%   var.baro  : 1x1 baro altitude variance in m^2
%
% dt is the sample interval in seconds.

    % --------------------------
    % Measurement noise (R)
    % --------------------------
    R_pos  = diag(var.gps(:));     % 3x3
    R_mag  = diag(var.mag(:));     % 3x3
    R_baro = var.baro;             % 1x1
    R_vel  = diag(var.vel(:));     % 3x3

    R = blkdiag(R_pos, R_mag, R_baro, R_vel);

    % --------------------------
    % Process noise (Q)
    % --------------------------
    % P/V driven by continuous white acceleration noise:
    Sa = diag(var.accel(:));
    
    % model mismatch floor
    sigma_a_floor = 1;
    Sa = Sa + (sigma_a_floor^2)*eye(3);
    
    Q_PV = [ (dt^4/4)*Sa, (dt^3/2)*Sa;
             (dt^3/2)*Sa, (dt^2)*Sa ];

    Q_PV(1:3, 1:3) = Q_PV(4:6, 4:6) * 1e3;
    Q_PV(4:6, 4:6) = Q_PV(4:6, 4:6) * 1e3;

    % Quaternion noise: keep it simple and small.
    % Use an effective gyro variance (average axis variance).
    w_var_eff = mean(var.gyro(:)) * 5e4;              % (rad/s)^2
    Q_e = eye(4) * (dt^2) * w_var_eff;          % crude but consistent

    % Bias random-walk strengths (variance per step).
    % Tune these. Starting values are intentionally not microscopic.
    q_bg = (1e-6)^2 * dt;    % gyro bias RW (rad/s)^2 per step
    q_ba = (1e-6)^2 * dt;    % accel bias RW (m/s^2)^2 per step
    q_bp = (1e-6)^2 * dt;    % GPS position bias RW (m)^2 per step
    q_bm = (1e-6)^2 * dt;    % mag bias RW (mag units)^2 per step
    q_bb = (1e-6)^2 * dt;    % baro bias RW (m)^2 per step
    q_bv = (1e-6)^2 * dt;    % GPS velocity bias RW ((m/s)^2) per step

    Q_bg = eye(3) * q_bg;
    Q_ba = eye(3) * q_ba;
    Q_bp = eye(3) * q_bp;
    Q_bm = eye(3) * q_bm;
    Q_bb = q_bb;
    Q_bv = eye(3) * q_bv;

    % State order must match your init_x_inds:
    % [P(3) V(3) e(4) b_g(3) b_a(3) b_p(3) b_m(3) b_b(1) b_v(3)]
    Q = blkdiag(Q_PV, Q_e, Q_bg, Q_ba, Q_bp, Q_bm, Q_bb, Q_bv);

    Q = (Q + Q.')/2;
    Q = Q + 1e-12 * eye(size(Q));

    % --------------------------
    % Initial covariance (P0)
    % --------------------------
    P0 = blkdiag( ...
        1e-2 * eye(3), ...   % P
        1e-2 * eye(3), ...   % V
        1e-2 * eye(4), ...   % q
        1e-6 * eye(3), ...   % b_g
        1e-6 * eye(3), ...   % b_a
        1e-6 * eye(3), ...   % b_p
        1e-6 * eye(3), ...   % b_m
        1e-6,         ...    % b_b
        1e-6 * eye(3) ...    % b_v
    );
end