function [data_stationary, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps)

% TRIM_FLYSIGHT Trims flysight data to just flight%
%   Flight is identified when the GPS velocity measurement is over 30 m/s.
%   This can probably be improved but works well enough.
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
%   data_stationary : A struct of the data while stationary, shortly before
%       flight. Note that this is not all of the stationary data, just that
%       which will be used for sensor calibration
%   data_moving     : A struct of the data while moving (in flight)

% We currently can't identify drops without GPS data
if isempty(data_gps)
    data_stationary = [];
    data_moving     = [];
    return
end

% Align the GPS times just in case
gps_shift = data_baro.time(end) - data_gps.time(end);
data_gps.time = data_gps.time + gps_shift;
data_gps_vel.time = data_gps_vel.time + gps_shift;
data_flysight_gps.GNSS.time = data_flysight_gps.GNSS.time + gps_shift;

% Assume flight is > 30 m/s and landing is < 0.5 m/s
vel_start = 30;
vel_end = 0.5;

% Start the data 250 seconds before we identify flight and end 60 seconds
% after landing, just to be safe
% calibration
dur_back = 250;
dur_forward = 60;

% Calculate the GPS velocity norm. Maybe this should take a moving average
% for better robustness
gps_vel_norm = vecnorm(data_gps_vel.data, 2, 2);

% Find when flight starts
time_moving = data_gps_vel.time(find(gps_vel_norm > vel_start, 1));
start_time = time_moving - dur_back;

% Find when flight ends
time_stopped = gps_vel_norm < vel_end;
% Here we need to make sure we only check for valid matches after flight
% starts, since before flight starts we also see very little GPS velocity
stop_idx = find(all([time_stopped, data_gps_vel.time > time_moving], 2), 1);
stop_time = data_gps_vel.time(stop_idx) + dur_forward;

% This is an anonymous helper function that will trim any of the data 
% to flight
trim_fcn_moving = @(data) data(find(data.time > start_time, 1):find(data.time > stop_time, 1), :);

% Take an 180 seconds before flight for sensor calibration purposes
stationary_dur = 180; % [sec]

% This is an anonymous helper function that will trim any of the data 
% to before flight
trim_fcn_stationary = @(data) data(find(data.time > start_time - stationary_dur, 1):find(data.time > start_time - 5, 1), :);

% Generate the structs using the previously-defined helper functions
% Note we don't trim the raw sensor data
data_moving = struct( ...
    'data_accel',           trim_fcn_moving(data_accel), ...
    'data_gyro',            trim_fcn_moving(data_gyro), ...
    'data_mag',             trim_fcn_moving(data_mag), ...
    'data_gps',             trim_fcn_moving(data_gps), ...
    'data_baro',            trim_fcn_moving(data_baro), ...
    'data_gps_vel',         trim_fcn_moving(data_gps_vel), ...
    'data_flysight_sensor', data_flysight_sensor, ...
    'data_flysight_gps', struct('GNSS', trim_fcn_moving(data_flysight_gps.GNSS)) ...
);

data_stationary = struct( ...
    'data_accel',           trim_fcn_stationary(data_accel), ...
    'data_gyro',            trim_fcn_stationary(data_gyro), ...
    'data_mag',             trim_fcn_stationary(data_mag), ...
    'data_gps',             trim_fcn_stationary(data_gps), ...
    'data_baro',            trim_fcn_stationary(data_baro), ...
    'data_gps_vel',         trim_fcn_stationary(data_gps_vel), ...
    'data_flysight_sensor', data_flysight_sensor, ...
    'data_flysight_gps', struct('GNSS', trim_fcn_stationary(data_flysight_gps.GNSS)) ...
);
end