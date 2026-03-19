% Plots flysight measurements and GPS on a map

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
trim_fcn1 = @(data) data(data.time < drop_time, :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
GPS1 = trim_fcn1(data_flysight_gps.GNSS);
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

%% Altitude

uif = uifigure;
g = geoglobe(uif);
geoplot3(g,data_flysight_gps.GNSS.lat,data_flysight_gps.GNSS.lon,data_flysight_gps.GNSS.hMSL,"r", "LineWidth", 3);
hold(g, 'on');
geoplot3(g,GPS1.lat,GPS1.lon,GPS1.hMSL,"b", "LineWidth", 3)