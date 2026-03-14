function [data_gps,...
    data_gps_vel,...
    data_flysight_gps] = align_flysight_times(data_gps, data_baro, data_gps_vel, data_flysight_gps)
% ALIGN_FLYSIGHT_TIMES Align GPS and sensor timestamps for flysight data.
%   This function attempts to fix misalignments in timestamps between
%   sensor data and GPS data by shifting the GPS data to align runtime
%   duration. The barometer data is used as the reference duration,
%   although any sensor measurement should work.
%
%   While this way of synchronization seems to work, it may induce slight
%   offsets depending on the exact timestamps used to calculate runtime
%   duration. The best way to avoid this would be for GPS data to also
%   include runtime data instead of just UTC time.
% 
% INPUTS:
%   data_gps         : Table of formatted GPS position data
%   data_baro        : Table of formatted barometer data
%   data_gps_vel     : Table of formated GPS velocity data
%   data_flysight_gps: Table of unformatted GPS data
%
% OUTPUTS:
%   data_gps         : Table of aligned GPS position data
%   data_gps_vel     : Table of aligned GPS velocity data
%   data_flysight_gps: Table of raw, aligned GPS velocity data  
%
% See also LOAD_FLYSIGHT_DATA

    % Calculate the difference in runtime
    gps_shift = data_baro.time(end) - data_gps.time(end);

    % Shift the timestamps
    data_gps.time = data_gps.time + gps_shift;
    data_gps_vel.time = data_gps_vel.time + gps_shift;
    data_flysight_gps.GNSS.time = data_flysight_gps.GNSS.time + gps_shift;
end