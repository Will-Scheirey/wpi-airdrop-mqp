function [data_accel, ...
    data_gyro, ...
    data_mag, ...
    data_gps, ...
    data_baro, ...
    data_gps_vel, ...
    data_flysight_sensor, ...
    data_flysight_gps] = get_and_trim_flysight(sensor_filename, gps_filename)

% GET_AND_TRIM_FLYSIGHT Loads and trims flysight data
%   This is a function that wraps load_flysight_data and trim_flysight,
%   returning just the data trimmed to the flight
%
% INPUTS:
%   sensor_filename : Filename of the sensor CSV file
%   smooth_window   : Filename of the GPS CSV file
%
% OUTPUTS:
%   data_accel           : Trimmed and aligned accelerometer data
%   data_gyro            : Trimmed and aligned gyroscope data
%   data_mag             : Trimmed and aligned magnetometer data
%   data_gps             : Trimmed and aligned GPS position data
%   data_baro            : Trimmed and aligned barometer data
%   data_gps_vel         : Trimmed and aligned GPS velocity data
%   data_flysight_sensor : Trimmed and aligned raw sensor data
%   data_flysight_gps    : Trimmed and aligned raw GPS data
%
% See also LOAD_FLYSIGHT_DATA, TRIM_FLYSIGHT


[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = load_flysight_data(sensor_filename, gps_filename, false);

[~, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);

data_accel = data_moving.data_accel;
data_gyro = data_moving.data_gyro;
data_mag = data_moving.data_mag;
data_gps = data_moving.data_gps;
data_baro = data_moving.data_baro;
data_gps_vel = data_moving.data_gps_vel;
data_flysight_sensor = data_moving.data_flysight_sensor;
data_flysight_gps = data_moving.data_flysight_gps;

end