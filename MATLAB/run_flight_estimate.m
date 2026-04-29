clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

full_dir = fullfile(parent_dir, drop_dir);
%% Get Drop Parameters

% Run Data
data_out = get_flight_estimates(drop_dir);

%% Load FG Data
% Extrapolate GPS coordinates from initial location
initial_gps = [data_out.measurements.gps_all.GNSS.lat(1),...
    data_out.measurements.gps_all.GNSS.lon(1),...
    data_out.measurements.gps_all.GNSS.hMSL(1)];
estimated_pos = enu2lla(data_out.estimates.pos, initial_gps, 'flat');

gps_time = data_out.measurements.gps_all.GNSS.time(end-length(estimated_pos)+1:end);

% Format data matrix for Simulink model to stream to FlightGear
flightgear_data = [gps_time,...     % Time
    estimated_pos(:,2),...          % Longitude
    estimated_pos(:,1),...          % Latitude
    estimated_pos(:,3),...          % Altitude
    data_out.estimates.eul(:,1),... % Roll
    data_out.estimates.eul(:,2),... % Pitch
    data_out.estimates.eul(:,3)];   % Yaw
% Trim to the last 3,500 datapoints (timeframe when drop occurs)
flightgear_data = flightgear_data(end-3500:end, :);

% Save ground truth GPS coordinates for comparison with computer vision
% algorithm
flightgear_data(:,1) = flightgear_data(:,1) - flightgear_data(1, 1);
cv_data = [flightgear_data(:, 1),...
    flightgear_data(:, 3),...
    flightgear_data(:, 2),...
    flightgear_data(:, 4)*0.3048];
cv_data = [round(cv_data(:,1), 1), round(cv_data(:,2), 5), round(cv_data(:,3),5), round(cv_data(:, 4),1)];

flightgear_file = "MATLAB/flightgear_data.mat";
cv_file = "/path/to/ground_truth_GPS.csv";

save(flightgear_file, "flightgear_data");
writematrix(cv_data, cv_file);

%% Plot Data
% fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates_smoothed");
fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates");

return
fig_idx = plot_meas(data_out.measurements.gps, ...
    data_out.measurements.accel, ...
    data_out.measurements.gyro, ...
    data_out.measurements.mag, ...
    data_out.measurements.baro, ...
    data_out.measurements.gps_vel, fig_idx);

return
fig_idx = new_fig(fig_idx);

fig_idx = plot_meas(data_out.stationary_measurements.data_gps, ...
    data_out.stationary_measurements.data_accel, ...
    data_out.stationary_measurements.data_gyro, ...
    data_out.stationary_measurements.data_mag, ...
    data_out.stationary_measurements.data_baro, ...
    data_out.stationary_measurements.data_gps_vel, fig_idx);