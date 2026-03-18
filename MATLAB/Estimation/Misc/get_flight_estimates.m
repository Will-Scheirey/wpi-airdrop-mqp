function data_out = get_flight_estimates(full_dir, verbose)
% GET_FLIGHT_ESTIMATES Estimates the drop trajectory and relevant info
% 
% INPUTS:
%   full_dir : The directory containing the drop .CSVs
%   verbose  : (Optional) Whether to output debug information
%
% OUTPUTS:
%   data_out : The flight estimates
%       .dt             : The timestep used for estimation
%       .t_plot         : The time series trimmed to the flight
%       .t_plot_drop    : The beginning and end timestamps of the drop
%       .drop_t_plot    : Time series for the drop, starting at 0s
%       .tspan          : (This is t_plot with one extra timestamp on the end)
%       .estimates      : A struct of the trajectory estimates
%           .all        : All of the states in one matrix
%           .pos        : Position in ENU from initial
%               [ m ]
%           .vel        : Velocity in ENU
%               [ m / s ]
%           .quat       : Quaternion
%           .eul        : Euler angles
%               [ rad ]
%           .gyro_bias  : Gyroscope sensor bias
%               [ rad / s]
%           .accel_bias : Accelerometer sensor bias
%               [ m / s^2 ]
%           .pos_bias   : GPS receiver bias in ENU
%               [ m ]
%           .mag_bias   : Magnetometer bias 
%               [ guass ]
%           .baro_bias  : Barometer altitude bias
%               [ m ]
%       .inputs         : A table of all the inputs (IMU measurements)
%       .drop_estimates : The estimates, trimmed to just the drop
%       .cov            : The covariance state estimate history
%       .kf             : The Kalman filter object
%       .measurements   : A table of all the measurements
%       .stationary_measurements : Measurements trimmed to before flight
%       .drop_info      : The struct of estimated drop parameters
%       .time_utc       : The datetime object for when the drop occurs
%       .weather        : Weather for the time of flight
%       .system_data    : Drop system data information
%       .carp           : Estimated carp/harp inputs for the drop
%       .winds          : Winds for the time of drop
%
% See Also: FORMAT_FLYSIGHT_DATA, LOAD_WEATHER_DATA, GET_CARP_PARAMS

if nargin < 2
    verbose = false;
end

smooth_window = 3;
alt_mean_window = 100;

[dt, ...
    tspan, ...
    all_measurements, ...
    flight_measurements, ...
    measurements, ...
    inputs, ...
    sensor_var, ...
    drop_info, data_stationary] = format_flysight_data(full_dir, smooth_window, alt_mean_window);

drop_time = drop_info.time_drop;
land_time = drop_info.time_land;

[R, Q, P0] = get_noise_params(sensor_var, dt);

%% Run the Kalman Filter
kf = Airdrop_EKF(R, Q, P0, dt);

acc_gps = flight_measurements.acc_gps;

kf.initialize(true, flight_measurements.accel.data(1, :)', ...
    flight_measurements.gyro.data(1, :)', ...
    flight_measurements.mag.data(1, :)', ...
    flight_measurements.gps.data(1, :)', ...
    flight_measurements.baro.data(1, :)');

kf.run_filter(measurements, inputs, tspan, acc_gps, drop_time, 1, verbose);

covariances = kf.P_hist;

x_est = kf.x_hist(:, 2:end)';
p_est = x_est(:, kf.x_inds.P_E);
v_est = x_est(:, kf.x_inds.V_E);
e_est = x_est(:, kf.x_inds.e);
eul_est = quat2eul(e_est);
w_b_est = x_est(:, kf.x_inds.b_g);
a_b_est = x_est(:, kf.x_inds.b_a);
p_b_est = x_est(:, kf.x_inds.b_p);
m_b_est = x_est(:, kf.x_inds.b_m);
b_b_est = x_est(:, kf.x_inds.b_b);

t_plot = tspan(1:end-1);
t_plot_drop = [drop_time, land_time];
% Speed, heading angle, altitude, windspeed and direction

drop_idx_start = find(t_plot > drop_time, 1);
drop_idx_stop = find(t_plot > land_time, 1);

drop_t_plot = t_plot(drop_idx_start:drop_idx_stop) - t_plot(drop_idx_start);

time_utc = all_measurements.gps_all.GNSS.datetime_utc(end);

[~, weather] = load_weather_data(time_utc);
weather.alt_agl(1) = 0;

%% Get CARP Inputs

system_data = load_system_data(full_dir);

estimates = struct( ...
    'all', x_est, ...
    'pos', p_est, ...
    'vel', v_est, ...
    'quat', e_est, ...
    'eul', eul_est, ...
    'gyro_bias', w_b_est, ...
    'accel_bias', a_b_est, ...
    'pos_bias', p_b_est, ...
    'mag_bias', m_b_est, ...
    'baro_bias', b_b_est);

drop_estimates = struct( ...
    'all', x_est(drop_idx_start:drop_idx_stop, :), ...
    'pos', p_est(drop_idx_start:drop_idx_stop, :), ...
    'vel', v_est(drop_idx_start:drop_idx_stop, :), ...
    'eul', eul_est(drop_idx_start:drop_idx_stop, :), ...
    'quat', e_est(drop_idx_start:drop_idx_stop, :), ...
    'gyro_bias', w_b_est(drop_idx_start:drop_idx_stop, :), ...
    'accel_bias', a_b_est(drop_idx_start:drop_idx_stop, :), ...
    'pos_bias', p_b_est(drop_idx_start:drop_idx_stop, :), ...
    'mag_bias', m_b_est(drop_idx_start:drop_idx_stop, :), ...
    'baro_bias', b_b_est(drop_idx_start:drop_idx_stop, :));

data_out = struct( ...
    'dt', dt, ...
    't_plot', t_plot, ...
    't_plot_drop', t_plot_drop, ...
    'drop_t_plot', drop_t_plot, ...
    'tspan', tspan, ...
    'estimates', estimates, ...
    'inputs', inputs, ...
    'drop_estimates', drop_estimates, ...
    'cov', covariances, ...
    'kf', kf, ...
    'measurements', flight_measurements, ...
    'stationary_measurements', data_stationary, ...
    'drop_info', drop_info, ...
    'time_utc', time_utc, ...
    'weather', weather, ...
    'system_data', system_data);

planned_landing_lla = system_data.planned_impact_lat_lon;
takeoff_lla = [flight_measurements.gps_all.GNSS.lat(1), flight_measurements.gps_all.GNSS.lon(1)];

[x,y] = gps_dist(takeoff_lla(1), takeoff_lla(2), planned_landing_lla(1), planned_landing_lla(2));

planned_landing_enu = [x, y];
drop_pos_enu = data_out.drop_info.gps_drop(1:2);

carp = get_carp_params(data_out);
carp.planned_relative_landing = planned_landing_enu - drop_pos_enu;
carp.system_data = system_data;

data_out.carp = carp;

winds = [weather.alt_agl * 1e3, mod(weather.direction, 360), ks2mps(weather.win_speed)];

data_out.winds.profile = winds;

drop_temp = interp1(weather.alt_agl, weather.temperature, carp.altitude / 1000);
activation_temp = interp1(weather.alt_agl, weather.temperature, system_data.planned_activation / 1000);

data_out.carp.drop_temp = drop_temp;
data_out.carp.activation_temp = activation_temp;

end

function [x,y] = gps_dist(lat1, lon1, lat2, lon2)
    % 1. Constants for Earth's Radius (approximate)
    R = 6371000; % Earth radius in meters
    
    % 2. Convert to Radians
    phi1 = deg2rad(lat1);
    phi2 = deg2rad(lat2);
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    
    % 3. Calculate East and North distances (Flat Earth Approximation)
    % Account for latitude changing the distance between longitudes
    y = dlat * R;
    x = dlon * R * cos(phi1);
end