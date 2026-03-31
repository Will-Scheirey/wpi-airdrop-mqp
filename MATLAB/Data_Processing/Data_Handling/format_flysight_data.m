function [dt, ...
    tspan, ...
    all_measurements, ...
    flight_measurements, ...
    ekf_measurements, ...
    ekf_inputs, ...
    sensor_var, ...
    drop_info, ...
    data_stationary] = format_flysight_data(full_dir, smooth_window, alt_mean_window)

% FORMAT_FLYSIGHT_DATA Loads and formats flysight data for the EKF
%   This function formats flysight data for the EKF and calculates other
%   relevant information, like when flight occurs, sensor variance, when
%   the drop occurs, and information relating to the drop.
%
%   This function could probably be named something else, as it does far
%   more than its name suggests.
%
% INPUTS:
%   full_dir        : Directory of flysight data
%   smooth_window   : Window to use for moving average of IMU data
%   alt_mean_window : Window to use for zeroing the barometer
%
% OUTPUTS:
%   dt                  : Timestep for propagation
%   tspan               : Array of time entries for estimation
%   all_measurements    : Table of all measurements
%   flight_measurements : Table of measurements, trimmed to fligh
%   ekf_measurements    : Cell array of formatted measurements for the EKF
%   ekf_inputs          : Cell array of formatted IMU inputs for the EKF
%   sensor_var          : Struct of estimated variance for each sensor
%   drop_info           : Struct of relevant drop information
%   data_stationary     : Table of measurements, trimmed to before flight
%       Primarily used for sensor calibration.
%
% See also LOAD_FLYSIGHT_DATA, TRIM_FLYSIGHT, GET_SENSOR_VAR, GET_DROP_INFO

%% Generate Filenames
sensor_filename = fullfile(full_dir, "SENSOR.CSV");
gps_filename = fullfile(full_dir, "TRACK.CSV");

%% Load Data and Trim
[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = load_flysight_data(sensor_filename, gps_filename, false);

[data_stationary, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);

% Calculate and store sensor variance
[var_accel, var_gyro, var_gps, var_mag, var_baro] = get_sensor_var(data_stationary);

sensor_var = struct( ...
    'accel', var_accel, ...
    'gyro',  var_gyro, ...
    'gps',   var_gps, ...
    'mag',   var_mag, ...
    'baro',  var_baro);

data_accel_all    = data_moving.data_accel;
data_gyro_all     = data_moving.data_gyro;
data_mag_all      = data_moving.data_mag;
data_gps_all      = data_moving.data_gps;
data_baro_all     = data_moving.data_baro;
data_gps_vel_all  = data_moving.data_gps_vel;
data_sensors_all  = data_moving.data_flysight_sensor;
data_gpsTrack_all = data_moving.data_flysight_gps;

all_measurements = struct( ...
    'accel',      data_accel, ...
    'gyro',       data_gyro, ...
    'mag',        data_mag, ...
    'gps',        data_gps, ...
    'baro',       data_baro, ...
    'gps_vel',    data_gps_vel, ...
    'sensor_all', data_flysight_sensor, ...
    'gps_all',    data_flysight_gps);

% NOTE: drop_info will be empty if a problem ocurred while attempting to
% identify the drop
drop_info = get_drop_info(data_accel_all,...
    data_gyro_all,...
    data_mag_all,...
    data_gps_all,...
    data_baro_all,...
    data_gps_vel_all,...
    data_sensors_all,...
    data_gpsTrack_all);

%%  Smooth Data

mean_window = smooth_window;
data_gyro_all.data  = movmean(data_gyro_all.data, mean_window, 1);
data_accel_all.data = movmean(data_accel_all.data, mean_window, 1);

%% Calculate Time Span

% Here we use the accelerometer; any sensor will work
t_start = data_accel_all.time(1);

% Calculate duration this way so if this calculation changes in the
% future, the rest of the code will still work
t_dur   = data_accel_all.time(end) - t_start;

t_end   = t_start + t_dur;

%% Zero the Barometer
zero_alt_mean_window = alt_mean_window;

% Zero the barometer by taking an average of the altitude measurements
% before takeoff. This does mean that altitude is now relative to initial.
data_baro      = data_baro_all;
zero_alt = mean(data_baro.data(1:zero_alt_mean_window));

data_baro.data = data_baro.data - zero_alt;

%% Format Measurements
% For the EKF, every measurement table row needs an entry with the 
% corresponding measurement index. For consistency, inputs are formatted
% the same, but index is set to -1

% Sometimes we may have data that didn't get GPS lock, so no GPS data
if ~isempty(data_gps_all)
    data_gps = data_gps_all;
    data_gps.meas_idx   = ones(length(data_gps.time), 1);
    data_gps.data(:, 3) = data_gps.data(:, 3) - mean(data_gps.data(1:zero_alt_mean_window, 3));
end

data_accel = data_accel_all;
data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);

data_gyro = data_gyro_all;
data_gyro.meas_idx  = repmat(-1, length(data_gyro.time), 1);

data_mag = data_mag_all;
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);

data_baro.meas_idx  = repmat(3, length(data_baro.time), 1);

data_vel = data_gps_vel_all;
data_vel.meas_idx  = repmat(4, length(data_vel.time), 1);

%% Trim Data Again
if isempty(data_gps_all)
    acc_gps = 0;
else
    % Extract the estimated accuracy for each GPS measurement calculated by
    % the GPS receiver
    acc_gps = [data_gpsTrack_all.GNSS.hAcc, data_gpsTrack_all.GNSS.vAcc];
end

% Trim data from the beginning
if ~isempty(data_gps_all)
    acc_gps      = acc_gps(data_gps.time > t_start, :);
    data_gps     = data_gps(data_gps.time > t_start, :);
    data_gps_vel = data_gps_vel_all(data_gps_vel_all.time > t_start, :);
end

data_accel = data_accel(data_accel.time > t_start, :);
data_mag   = data_mag  (data_mag  .time > t_start, :);
data_gyro  = data_gyro (data_gyro .time > t_start, :);
data_baro  = data_baro (data_baro .time > t_start, :);

% Trim data to the end
if ~isempty(data_gps_all)
    acc_gps = acc_gps(data_gps.time < t_end, :);
    data_gps = data_gps(data_gps.time < t_end, :);
    data_gps_vel = data_gps_vel(data_gps_vel_all.time < t_end, :);
end

data_accel = data_accel(data_accel.time < t_end, :);
data_mag   = data_mag  (data_mag  .time < t_end, :);
data_gyro  = data_gyro (data_gyro .time < t_end, :);
data_baro  = data_baro (data_baro .time < t_end, :);

flight_measurements = struct( ...
    'accel',      data_accel, ...
    'gyro',       data_gyro, ...
    'mag',        data_mag, ...
    'gps',        data_gps, ...
    'baro',       data_baro, ...
    'gps_vel',    data_gps_vel, ...
    'acc_gps',    acc_gps, ...
    'sensor_all', data_flysight_sensor, ...
    'gps_all',    data_flysight_gps);

%% Calculate Timestep

% Find the minimum timestep for any of the sensor measurements
dt_min_accel = min(diff(data_accel.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

dt_mult = 1; % In case we want to use this in the future

% Make sure we don't try to use GPS data if we don't have it
if ~isempty(data_gps_all)
    ekf_measurements = {data_gps, data_mag, data_baro, data_vel};
    dt_min_gps       = min(diff(data_gps.time));

    dt_arr = [dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro];
else
    ekf_measurements = {data_mag, data_gyro};
    dt_arr = [dt_min_accel, dt_min_mag, dt_min_gyro, dt_min_baro];
end

% Choose dt as the minimum required for any of the measurements
dt = min(dt_arr);
dt = dt * dt_mult;

% Generate the array of time entries for estimation
tspan = t_start : dt : t_start + t_dur;

% Interpolate IMU measurements if required
accel_interp = interp1(data_accel.time, data_accel.data, tspan);
gyro_interp  = interp1(data_gyro.time, data_gyro.data, tspan);

% Sometimes interpolation can act a bit weird, so make sure we dont have
% any nans or infs
good         = isfinite(accel_interp(:, 1));
accel_interp = accel_interp(good, :);
gyro_interp  = gyro_interp(good, :);

% Generate the table for ekf inputs
ekf_inputs = table(tspan(good)', accel_interp, gyro_interp, ...
    'VariableNames', {'time', 'accel', 'gyro'});

end