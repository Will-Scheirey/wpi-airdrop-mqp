clearvars -except data data_accel data_gyro data_mag data_gps data_baro data_gps_vel data_flysight_sensor data_flysight_gps; clc; close all

folder = uigetdir("haars_data");

if folder == 0
    return
end

sensor_filename = fullfile(folder, "SENSOR.CSV");
gps_filename = fullfile(folder, "TRACK.CSV");

if ~isfile(sensor_filename) || ~isfile(gps_filename)
    error("The required files do not exist within " + folder)
end

[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_flysight_data(sensor_filename, gps_filename, false);

[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor, ...
    data_flysight_gps] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);


%% Plot
fig_idx = plot_meas(data_gps, data_accel, data_gyro, data_mag, data_baro);

%% Altitude

temp = data_flysight_sensor.BARO.temperature + 273.15;
pressure = data_flysight_sensor.BARO.pressure;

density_alt = density_altitude(pressure*100, temp);
density_alt = density_alt - density_alt(1);

plot(data_flysight_sensor.BARO.time - data_flysight_sensor.BARO.time(1), density_alt, '--k', 'LineWidth', 2)

%{
uif = uifigure;
g = geoglobe(uif);
geoplot3(g,data_flysight_gps.GNSS.lat,data_flysight_gps.GNSS.lon,data_flysight_gps.GNSS.hMSL,"r", "LineWidth", 3)
%}