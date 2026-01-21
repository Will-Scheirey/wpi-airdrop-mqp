dirn_ame = "DN172_Lt3_n12_08072025_side_2";

sensor_filename = fullfile(dir_name, "SENSOR.CSV");
gps_filename = fullfile(dir_name, "TRACK.CSV");

fprintf("#%d, Name: %s\n", n, dir_name)

[data_accel,...
data_gyro,...
data_mag,...
data_gps,...
data_baro,...
data_gps_vel,...
data_flysight_sensor,...
data_flysight_gps] = get_trim_align(sensor_filename, gps_filename);

accel_mean_window = 10;
accel_norm = movmean(vecnorm(data_accel.data, 2, 2), accel_mean_window, 1);
accel_drop = 15;
step_back = 10;

drop_idx = find(accel_norm > accel_drop, 1);
drop_time = data_accel.time(drop_idx) - step_back;

trim_fcn = @(data) data(find(data.time > drop_time, 1):end, :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

data_gps.data(:, 1:2) = data_gps.data(:, 1:2) - data_gps.data(1, 1:2);

plot_meas(data_gps, data_accel, data_gyro, data_mag, data_baro)