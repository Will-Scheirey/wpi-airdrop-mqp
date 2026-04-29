function [data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps] = load_flysight_data(sensor_filename, gps_filename, cv_filename, verbose)
% LOAD_FLYSIGHT_DATA Loads flysight data from a directory
%   This function loads sensor and GPS data from a HAARS data directory.
%   Parsing the raw CSV can take quite a while due to the formatting, so we
%   attempt to load from a .MAT file if it exists. If it does not, we save
%   the output to a .MAT file for future use.
%
%   Output data (except for raw) are tables with columns "time" and "data",
%   where "time" is a column vectory of seconds relative to initial, and 
%   "data" is a matrix with columns corresponding to axes and rows 
%   corresponding to the entries in the "time" column
%
%   It is assumed that the units in the CSV file are:
%       Acceleration : G's
%       Gyroscope    : Degrees / sec
%       Magnetometer : Gauss
%       GPS Position : Degrees lat/lon, Meters MSL
%       Barometer    : Pa
%       GPS Velocity : Meters / sec NED
%
% INPUTS:
%   sensor_filename : Filename of the sensor CSV file
%   gps_filename    : Filename of the GPS CSV file
%   cv_filename     : Filename of the computer vision GPS CSV file
%   verbose         : A boolean specifying verbose output
%
% OUTPUTS:
%   data_accel           : Accelerometer data 
%       [ m / s^2 ]
%   data_gyro            : Gyroscope data
%       [ rad / s ]
%   data_mag             : Magnetometer data
%       [ gauss ]
%   data_gps             : GPS position data in ENU from initial
%       [ m ]
%   data_baro            : Barometer altitude data in MSL
%       [ m ]
%   data_gps_vel         : GPS velocity data in ENU
%       [ m / s ]
%   data_flysight_sensor : Raw sensor data
%   data_flysight_gps    : Raw GPS data
%
% See also PARSE_FLYSIGHT_RAW

% Make verbose an optional input
if nargin < 3
    verbose = false;
end

% Loading from the CSV can take multiple minutes. So, check if we have
% a .mat file already saved. If we do, use it. Otherwise, load the
% data and save a .mat file for next time.
sensor_mat = replace(sensor_filename, "CSV", "mat");
if isfile(sensor_mat)
    if verbose
        disp("Loading Flysight Sensor from .mat...")
    end
    data_flysight_sensor = load(sensor_mat).data_flysight_sensor;
else
    if verbose
        disp("Loading Flysight Sensor from .csv...")
    end
    data_flysight_sensor = parse_flysight_raw(sensor_filename);
    save(sensor_mat, "data_flysight_sensor");
end

% Do the same with GPS data
gps_mat = replace(gps_filename, "CSV", "mat");
if isfile(gps_mat)
    if verbose
        disp("Loading Flysight GPS from .mat...")
    end
    data_flysight_gps = load(gps_mat).data_flysight_gps;
else
    if verbose
        disp("Loading Flysight GPS from .csv...")
    end
    data_flysight_gps = parse_flysight_raw(gps_filename);
    save(gps_mat, "data_flysight_gps");
end

if verbose
    disp("Processing...")
end

% Load GPS coordinates differently depending on whether computer vision
% inputs are provided
if isfile(cv_filename)
    cv_data = readtable(cv_filename);
    cv_lat = cv_data.latitude;
    cv_lon = cv_data.longitude;
    
    % Check that we have GPS data before extracting it
    if ~isfield(data_flysight_gps, "GNSS")
        data_gps = table();
        data_gps_vel = table();
        return
    end
    
    % Store true GPS data
    gps_time = data_flysight_gps.GNSS.time;
    gps_lat  = data_flysight_gps.GNSS.lat;
    gps_lon  = data_flysight_gps.GNSS.lon;
    gps_h    = data_flysight_gps.GNSS.hMSL;
    
    % Corroborate CV timestamps given window length of 15 and stride of 5,
    % find time at which to switch to CV GPS estimates
    idx0 = numel(gps_time) - 3500 + 7;
    t0 = gps_time(idx0);
    
    % Sample CV timestamps at 1 Hz
    M = numel(cv_lat);
    t_cv = t0 + (0:M-1)';
    h_cv = interp1(gps_time, gps_h, t_cv, 'linear', 'extrap');
    
    % Split GPS data such that FlySight GPS is retained before CV begins,
    % then EKF switches to CV GPS estimates

    % Extract LLA
    gps_keep = gps_time < t0;
    lla_gps_before = [gps_lat(gps_keep), gps_lon(gps_keep), gps_h(gps_keep)];
   
    lla_cv = [cv_lat(:), cv_lon(:), h_cv(:)];
    ref_lla = [gps_lat(1), gps_lon(1), gps_h(1)];
    
    % Convert LLA to ENU from the initial position
    enu_gps_before = lla2enu(lla_gps_before, ref_lla, 'flat');
    enu_cv         = lla2enu(lla_cv,         ref_lla, 'flat');
    
    time_combined = [gps_time(gps_keep); t_cv];
    enu_combined  = [enu_gps_before; enu_cv];
    
    data_gps = table(time_combined, enu_combined, ...
        'VariableNames', {'time', 'data'});
else
    % Generate IMU timestamps (accel and mag are the same)
    imu_time = (data_flysight_sensor.IMU.time - data_flysight_sensor.IMU.time(1));
    % Generate accel table
    data_accel = table(imu_time, [data_flysight_sensor.IMU.ax, data_flysight_sensor.IMU.ay, data_flysight_sensor.IMU.az] * 9.81, 'VariableNames', {'time', 'data'});
    
    % Generate gyro table
    gyro_meas = [data_flysight_sensor.IMU.wx, data_flysight_sensor.IMU.wy, data_flysight_sensor.IMU.wz];
    gyro_meas = deg2rad(gyro_meas);
    data_gyro = table(imu_time, gyro_meas, 'VariableNames', {'time', 'data'});
    
    % Generate mag timestamps and table
    mag_time = (data_flysight_sensor.MAG.time - data_flysight_sensor.MAG.time(1));
    data_mag = table(mag_time, [data_flysight_sensor.MAG.x, data_flysight_sensor.MAG.y, data_flysight_sensor.MAG.z], 'VariableNames', {'time', 'data'});
    
    % Calculated atmosphere using MATLAB's pressure altitude function
    % `atmospalt'. This is definitely not the ideal way to be feeding
    % measurements to the EKF
    baro_alt = atmospalt(data_flysight_sensor.BARO.pressure);
    baro_time = (data_flysight_sensor.BARO.time - data_flysight_sensor.BARO.time(1));
    data_baro = table(baro_time, baro_alt, 'VariableNames', {'time', 'data'});
    
    % Check we have GPS data before extracting it
    if ~isfield(data_flysight_gps, "GNSS")
        data_gps = table();
        data_gps_vel = table();
        return
    end
    
    % Extract LLA
    data_gps_lla = [data_flysight_gps.GNSS.lat, data_flysight_gps.GNSS.lon, data_flysight_gps.GNSS.hMSL];
    % Convert LLA to ENU from the initial position
    data_gps_enu = lla2enu(data_gps_lla(:, 1:3), data_gps_lla(1, 1:3), 'flat');
    
    data_gps = table(data_flysight_gps.GNSS.time, data_gps_enu, 'VariableNames', {'time', 'data'});
end

% Generate the GPS velocity table in ENU
data_gps_vel = table(data_flysight_gps.GNSS.time, [data_flysight_gps.GNSS.velE, data_flysight_gps.GNSS.velN, -data_flysight_gps.GNSS.velD], 'VariableNames', {'time', 'data'});

end