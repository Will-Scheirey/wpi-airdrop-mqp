function [ ...
    dt, ...
    tspan, ...
    all_measurements, ...
    flight_measurements, ...
    measurements, ...
    inputs, ...
    sensor_var, ...
    drop_info] = format_flysight_data(full_dir, smooth_window, alt_mean_window)

%% Load Data
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
    data_flysight_gps] = get_flysight_data(sensor_filename, gps_filename, false);

[data_stationary, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);
[var_accel, var_gyro, var_gps, var_mag, var_baro] = calibrate_sensors(data_stationary);

sensor_var = struct( ...
    'accel', var_accel, ...
    'gyro',  var_gyro, ...
    'gps',   var_gps, ...
    'mag',   var_mag, ...
    'baro',  var_baro);

data_accel_all = data_moving.data_accel;
data_gyro_all = data_moving.data_gyro;
data_mag_all = data_moving.data_mag;
data_gps_all = data_moving.data_gps;
data_baro_all = data_moving.data_baro;
data_gps_vel_all = data_moving.data_gps_vel;
data_sensors_all = data_moving.data_flysight_sensor;
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

%{
flight_measurements = struct( ...
    'accel', data_accel_all, ...
    'gyro', data_gyro_all, ...
    'mag', data_mag_all, ...
    'gps', data_gps_all, ...
    'baro', data_baro_all, ...
    'gps_vel', data_gps_vel_all, ...
    'sensor_all', data_sensors_all, ...
    'gps_all', data_gpsTrack_all);
%}

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
data_gps_all.data = movmean(data_gps_all.data, mean_window, 1);


%% Calculate Time Span
if isempty(drop_info)
    t_start = 4;
else
    t_start = data_accel_all.time(1);
end

t_dur = data_accel_all.time(end) - t_start;

t_end   = t_start + t_dur;

%% Zero the Barometer
zero_alt_mean_window = alt_mean_window;

data_baro = data_baro_all;
data_baro.data = data_baro.data - mean(data_baro.data(1:zero_alt_mean_window));

%% Format Measurements
data_accel = data_accel_all;
data_accel.meas_idx = repmat(-1, length(data_accel.time), 1);

if ~isempty(data_gps_all)
    data_gps = data_gps_all;
    data_gps.meas_idx   = ones(length(data_gps.time), 1);
    data_gps.data(:, 3) = data_gps.data(:, 3) - mean(data_gps.data(1:zero_alt_mean_window, 3));
end

data_mag = data_mag_all;
data_mag.meas_idx   = repmat(2, length(data_mag.time), 1);

data_gyro = data_gyro_all;
data_gyro.meas_idx  = repmat(-1, length(data_gyro.time), 1);

data_baro.meas_idx  = repmat(3, length(data_baro.time), 1);

data_vel = data_gps_vel_all;
data_vel.meas_idx  = repmat(4, length(data_vel.time), 1);

%% Trim Data Again
if isempty(data_gps_all)
    acc_gps = 0;
else
    acc_gps = [data_gpsTrack_all.GNSS.hAcc, data_gpsTrack_all.GNSS.vAcc];
end

if ~isempty(data_gps_all)
    acc_gps = acc_gps(data_gps.time > t_start, :);
    data_gps = data_gps(data_gps.time > t_start, :);
    data_gps_vel = data_gps_vel_all(data_gps_vel_all.time > t_start, :);
end
data_accel = data_accel(data_accel.time > t_start, :);
data_mag = data_mag(data_mag.time > t_start, :);
data_gyro = data_gyro(data_gyro.time > t_start, :);
data_baro = data_baro(data_baro.time > t_start, :);

if ~isempty(data_gps_all)
    acc_gps = acc_gps(data_gps.time < t_end, :);
    data_gps = data_gps(data_gps.time < t_end, :);
    data_gps_vel = data_gps_vel(data_gps_vel_all.time < t_end, :);
end

data_accel = data_accel(data_accel.time < t_end, :);
data_mag = data_mag(data_mag.time < t_end, :);
data_gyro = data_gyro(data_gyro.time < t_end, :);
data_baro = data_baro(data_baro.time < t_end, :);

flight_measurements = struct( ...
    'accel', data_accel, ...
    'gyro', data_gyro, ...
    'mag', data_mag, ...
    'gps', data_gps, ...
    'baro', data_baro, ...
    'gps_vel', data_gps_vel, ...
    'acc_gps', acc_gps, ...
    'sensor_all', data_flysight_sensor, ...
    'gps_all', data_flysight_gps);

%% Calculate Timestep

dt_min_accel = min(diff(data_accel.time));
dt_min_mag   = min(diff(data_mag.time));
dt_min_gyro  = min(diff(data_gyro.time));
dt_min_baro  = min(diff(data_baro.time));

data_vel.data = data_vel.data;

dt_mult = 1;

if ~isempty(data_gps_all)
    measurements = {data_gps, data_mag, data_baro, data_vel};
    % measurements = {data_gps, data_mag(data_mag.time < drop_info.time_drop, :), data_baro};
    dt_min_gps   = min(diff(data_gps.time));
    dt          = min([dt_min_accel, dt_min_gps, dt_min_mag, dt_min_gyro, dt_min_baro]) * dt_mult;

else
    measurements = {data_mag, data_gyro};
    dt          = min([dt_min_accel, dt_min_mag, dt_min_gyro, dt_min_baro]) * dt_mult;
end
inputs = table(data_accel.time, data_accel.data, data_gyro.data, ...
    'VariableNames', {'time', 'accel', 'gyro'});

tspan = t_start : dt : t_start + t_dur;

end