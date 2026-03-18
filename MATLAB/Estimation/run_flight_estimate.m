clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN152_Lt1_n15_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir, true);

%% Plot Data
fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates");

fig_idx = plot_meas(data_out.measurements.gps, ...
    data_out.measurements.accel, ...
    data_out.measurements.gyro, ...
    data_out.measurements.mag, ...
    data_out.measurements.baro, ...
    data_out.measurements.gps_vel, fig_idx, data_out.t_plot_drop);