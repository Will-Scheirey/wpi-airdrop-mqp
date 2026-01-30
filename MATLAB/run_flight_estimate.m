clear; clc;

parent_dir = "haars_data";
drop_dir = "DN172_Lt3_n12_08072025_side_2";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir);

%% Plot Data
fig_idx = plot_estimates(data_out);

time = data_out.measurements.gps_all.GNSS.datetime_utc(end);

[weather_data, the_weather] = load_weather(time);

the_weather.win_speed = ks2mps(the_weather.win_speed);
the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
the_weather.alt_agl(1) = 0;

alt_est = data_out.estimates.pos(:, 3);
v_est = data_out.estimates.vel(:, 1:2);

wind_speed_interp = interp1(the_weather.alt_agl, the_weather.win_speed, alt_est);
wind_angle_interp = interp1(the_weather.alt_agl, the_weather.wind_direction, alt_est);

wind_vec = wind_speed_interp .* [cosd(wind_angle_interp), sind(wind_angle_interp)];

groundspeed = vecnorm(v_est, 2, 2);

windspeed_vec = v_est + wind_vec;
windspeed = vecnorm(windspeed_vec, 2, 2);

v_meas = [data_out.measurements.gps_all.GNSS.velN, data_out.measurements.gps_all.GNSS.velE];
alt_meas = data_out.measurements.gps_all.GNSS.hMSL;

wind_speed_interp = interp1(the_weather.alt_agl, the_weather.win_speed, alt_meas);
wind_angle_interp = interp1(the_weather.alt_agl, the_weather.wind_direction, alt_meas);

wind_vec = wind_speed_interp .* [cosd(wind_angle_interp), sind(wind_angle_interp)];

groundspeed_meas = vecnorm(v_meas, 2, 2);

windspeed_vec = v_meas + wind_vec;
windspeed_meas = vecnorm(windspeed_vec, 2, 2);

fig_idx = new_fig(fig_idx);
clf
plot(data_out.t_plot, windspeed, 'DisplayName', 'Windspeed', 'LineWidth', 1.5); hold on
plot(data_out.t_plot, groundspeed, 'DisplayName', 'Groundspeed', 'LineWidth', 1.5);
xlim(data_out.t_plot_drop)
legend

fig_idx = new_fig(fig_idx);
clf
plot(data_out.measurements.gps_all.GNSS.time, windspeed_meas, 'DisplayName', 'Windspeed', 'LineWidth', 1.5); hold on
plot(data_out.measurements.gps_all.GNSS.time, groundspeed_meas, 'DisplayName', 'Groundspeed', 'LineWidth', 1.5);
xlim(data_out.t_plot_drop)
legend