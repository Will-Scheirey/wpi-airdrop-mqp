function drop_info = get_drop_info(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps)

% GET_DROP_INFO Gets information relating to an airdrop from flysight data
%   This function uses sensor and GPS measurements to identify when a drop
%   occurs, when the payload lands, and the GPS position reading on drop
%   and landing.
%
%   The drop is identified when an acceleration of over 15 m/s^2 is
%   sustained for 10 acceleration measurements. This can definitely be
%   improved. Additionally, using uncorrected barometer data as the
%   altitude for when the drop occurs is likely not ideal.
%
% INPUTS:
%   data_accel           : Trimmed and aligned accelerometer data
%   data_gyro            : Trimmed and aligned gyroscope data
%   data_mag             : Trimmed and aligned magnetometer data
%   data_gps             : Trimmed and aligned GPS position data
%   data_baro            : Trimmed and aligned barometer data
%   data_gps_vel         : Trimmed and aligned GPS velocity data
%   data_flysight_sensor : Trimmed and aligned raw sensor data
%   data_flysight_gps    : Trimmed and aligned raw GPS data
%
% OUTPUTS:
%   drop_info : A struct of information relating to the drop
%       .gps_drop  : GPS position when the drop occurs
%       .vel_drop  : GPS velocity when the drop occurs
%       .gps_land  : GPS position when the drop ends (landing)
%       .time_drop : Timestamp when the drop occurs
%       .time_land : Timestamp when the drop ends (landing)

% Set up parameters
% Average 10 points
accel_mean_window = 10;
accel_norm = movmean(vecnorm(data_accel.data, 2, 2), accel_mean_window, 1);

% Assume drop is when accel > 12 m/s^2
accel_drop = 12;
% Assuming landing is when vel < 2 m/s
vel_land = 2;

% Take 2 seconds before and 2 seconds after identified drop and landing,
% just to be safe.
step_back = 2;
step_forward = 2;

% Find when the drop occurred
drop_idx = find(accel_norm > accel_drop, 1);

if isempty(drop_idx)
    drop_info = [];
    return
end

% Calculate the time
drop_time = data_accel.time(drop_idx) - step_back;

% Trim the data for identification of landing
trim_fcn = @(data) data(find(data.time > drop_time, 1):end, :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

% Find when landing occurred
vel_norm = movmean(vecnorm(data_gps_vel.data, 2, 2), accel_mean_window, 1);
land_idx = find(vel_norm < vel_land, 1);

if isempty(land_idx)
    drop_info = [];
    return
end

% Calculate the time
land_time = data_gps_vel.time(land_idx) + step_forward;

% Trim the data
trim_fcn = @(data) data(1:find(data.time > land_time, 1), :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

% Generate the output struct
drop_info = struct( ...
    'gps_drop', [data_gps.data(1,1:2), data_baro.data(1)], ...
    'vel_drop', data_gps_vel.data(1,:),...
    'gps_land', [data_gps.data(end,1:2), data_baro.data(end)], ...
    'time_drop', drop_time, ...
    'time_land', land_time ...
    );

end