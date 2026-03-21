function [data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps] = get_trim_align_flysight(sensor_filename, gps_filename)

% GET_TRIM_ALIGN_FLYSIGHT Loads, trims, and aligns flysight data
%   This is a function that wraps get_and_trim_flysight and trim_flysight,
%   returning the sensor and GPS data aligned and trimmed to flight
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
% See also GET_AND_TRIM_FLYSIGHT, ALIGN_FLYSIGHT_TIMES

[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_and_trim_flysight(sensor_filename, gps_filename);

% We only need to modify the GPS data, so if none exists, just return
if isempty(data_gps)
    return;
end

[data_gps,...
    data_gps_vel,...
    data_flysight_gps] = align_flysight_times(data_gps, data_baro, data_gps_vel, data_flysight_gps);
end