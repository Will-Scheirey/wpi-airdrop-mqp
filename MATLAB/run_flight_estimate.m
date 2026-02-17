clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN172_Lt3_n12_08072025_side_2";
full_dir = fullfile(parent_dir, drop_dir);

%% Get Drop Parameters

%% Run Data
data_out = get_flight_estimates(full_dir);

%% Plot Data
fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates_smoothed");
% fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates");

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