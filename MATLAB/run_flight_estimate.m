clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN153_Lt1_n16_08052025_Inside";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir);

%% Plot Data
fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates_smoothed");
% fig_idx = plot_estimates(data_out, data_out.t_plot, data_out.tspan, "estimates");

%{
fig_idx = plot_meas(data_out.measurements.gps, ...
    data_out.measurements.accel, ...
    data_out.measurements.gyro, ...
    data_out.measurements.mag, ...
    data_out.measurements.baro, ...
    data_out.measurements.gps_vel, fig_idx);

fig_idx = new_fig(fig_idx);
plot(data_out.inputs.time, data_out.inputs.accel); hold on
%}
% data_out.measurements.accel

%{
fig_idx = plot_meas(data_out.stationary_measurements.data_gps, ...
    data_out.stationary_measurements.data_accel, ...
    data_out.stationary_measurements.data_gyro, ...
    data_out.stationary_measurements.data_mag, ...
    data_out.stationary_measurements.data_baro, ...
    data_out.stationary_measurements.data_gps_vel, fig_idx);
%}
return

time = data_out.measurements.gps_all.GNSS.datetime_utc(end);

[weather_data, the_weather] = load_weather(time);

the_weather.win_speed = ks2mps(the_weather.win_speed);
the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
the_weather.alt_agl(1) = 0;

alt_est = data_out.estimates.pos(:, 3);
v_est = data_out.estimates.vel(:, 1:2);

wind_speed_interp = interp1(the_weather.alt_agl, the_weather.win_speed, alt_est);
wind_angle_interp = interp1(the_weather.alt_agl, the_weather.wind_direction, alt_est);

wind_vec = wind_speed_interp .* [sind(wind_angle_interp), cosd(wind_angle_interp)];
wind_vec_est = data_out.kf.x_hist(24:25, 1:end-1);

%{
groundspeed = vecnorm(v_est, 2, 2);

wind_sign = 1;

windspeed_vec = v_est + wind_vec*wind_sign;
windspeed = vecnorm(windspeed_vec, 2, 2);

v_meas = [data_out.measurements.gps_vel.data(:, 1), data_out.measurements.gps_vel.data(:, 2)];
alt_meas = data_out.measurements.gps.data(:, 3);

wind_speed_interp = interp1(the_weather.alt_agl, the_weather.win_speed, alt_meas);
wind_angle_interp = interp1(the_weather.alt_agl, the_weather.wind_direction, alt_meas);

wind_vec = wind_speed_interp .* [sind(wind_angle_interp), cosd(wind_angle_interp)];

groundspeed_meas = vecnorm(v_meas, 2, 2);

windspeed_vec = v_meas + wind_vec*wind_sign;
windspeed_meas = vecnorm(windspeed_vec, 2, 2);

fig_idx = new_fig(fig_idx);
clf
plot(data_out.t_plot, windspeed, 'DisplayName', 'Windspeed', 'LineWidth', 1.5); hold on
plot(data_out.t_plot, groundspeed, 'DisplayName', 'Groundspeed', 'LineWidth', 1.5);
xlim(data_out.t_plot_drop)
legend

fig_idx = new_fig(fig_idx);
clf
plot(data_out.measurements.gps.time, windspeed_meas, 'DisplayName', 'Windspeed', 'LineWidth', 1.5); hold on
plot(data_out.measurements.gps.time, groundspeed_meas, 'DisplayName', 'Groundspeed', 'LineWidth', 1.5);
xlim(data_out.t_plot_drop)
legend
%}