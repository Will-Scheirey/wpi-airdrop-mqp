clearvars -except data data_accel data_gyro data_mag data_gps data_baro data_gps_vel data_flysight_sensor data_flysight_gps; clc;

folder = uigetdir("haars_data");

% DN145_Lt1_n03_08052025_side

if folder == 0
    return
end

sensor_filename = fullfile(folder, "SENSOR.CSV");
gps_filename = fullfile(folder, "TRACK.CSV");

if ~isfile(sensor_filename) || ~isfile(gps_filename)
    error("The required files do not exist within " + folder)
end

%{
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
%}

[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_and_trim_flysight(sensor_filename, gps_filename);

drop_info = get_drop_info(data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps);

drop_time = drop_info.time_drop;
land_time = drop_info.time_land;

trim_fcn = @(data) data(find(data.time > drop_time, 1):end, :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

trim_fcn = @(data) data(1:find(data.time > land_time, 1), :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

data_gps.data = data_gps.data - data_gps.data(1, :);

data_gyro.data = movmean(data_gyro.data, 10, 1);

%% Plot
fig_idx = plot_meas(data_gps, data_accel, data_gyro, data_mag, data_baro, data_gps_vel);

return
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