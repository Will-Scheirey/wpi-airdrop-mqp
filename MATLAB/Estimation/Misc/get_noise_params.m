function [R, Q, P0] = get_noise_params(sensor_var, dt)
% GET_NOISE_PARAMS Generates noise parameters for given sensor variations
%
% INPUTS:
%   sensor_var : Struct of sensor variance
%       .mag
%       .baro
%       .gyro
%       .accel
%   dt         : Propagation timestep

% Set this to be constant for now
% GPS Position
Rp = 5;
R_pos = [
    Rp, 0, 0;
    0, Rp, 0;
    0, 0, Rp*5;
].^2;

% Magnetometer
R_mag = blkdiag(sensor_var.mag(1), sensor_var.mag(2), sensor_var.mag(3)).^2;

% Had to hand-tune this (probably can be improved)
% Barometer
R_baro = (sensor_var.baro * 1e3)^2;

% Set this to be constant for now
% GPS velocity
Rv = 2;
R_vel = eye(3) * Rv^2;

% Generate the noise covariance matrix
R = blkdiag( ...
    R_pos, ...
    R_mag, ...
    R_baro, ...
    R_vel ...
    );

% Acceleration noise - hand-tuned
sigma_a = sqrt(sensor_var.accel(:)) * 1e3;
Sa = diag(sigma_a.^2);

% Standard position and velocity process noise matrix
Q_PV = [ (dt^4/4)*Sa, (dt^3/2)*Sa;
         (dt^3/2)*Sa, (dt^2)*Sa ];

% Quaternion process noise
sigma_w = sqrt(norm(sensor_var.gyro));
Q_e = eye(4) * (dt*sigma_w)^2;

% Hard to tune these

% Gyro bias 
Qwb = 1e-5;
Q_wb = eye(3) * Qwb^2;

% Accel bias
Qab = 1e-5;
Q_ab = eye(3) * Qab^2;

% GPS position bias
Qpb = 1e-5;
Q_pb = eye(3) * Qpb^2;

% Magnetometer bias - had to make this large for while in plane
sigma_bm = 1e-2;
Q_mb = eye(3) * (sigma_bm^2);

% Barometer altitude bias
sigma_bb = 1e-2;
Q_bb = sigma_bb^2;

% GPS velocity bias
sigma_bv = 1e-5;
Q_bv = sigma_bv^2 * eye(3);

Q = blkdiag(...
    Q_PV, ...
    Q_e,  ...
    Q_wb, ...
    Q_ab, ...
    Q_pb, ...
    Q_mb, ...
    Q_bb, ...
    Q_bv ...
    );

% Initial state covariance estimate - make these all relatively small
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

% Add some amount of noise floor
Q = (Q + Q.')/2;                     % enforce symmetry
q_floor = 1e-12;                     % pick a floor in variance units
Q = Q + q_floor * eye(size(Q));      % jitter to make invertible

end