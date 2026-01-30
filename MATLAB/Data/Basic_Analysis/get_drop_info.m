function drop_info = get_drop_info(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps)

accel_mean_window = 10;
accel_norm = movmean(vecnorm(data_accel.data, 2, 2), accel_mean_window, 1);
accel_drop = 15;
step_back = 2;

vel_land = 2;
step_forward = 2;

drop_idx = find(accel_norm > accel_drop, 1);

if isempty(drop_idx)
    drop_info = [];
    return
end

drop_time = data_accel.time(drop_idx) - step_back;

trim_fcn = @(data) data(find(data.time > drop_time, 1):end, :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);

vel_norm = movmean(vecnorm(data_gps_vel.data, 2, 2), accel_mean_window, 1);
land_idx = find(vel_norm < vel_land, 1);
land_time = data_gps_vel.time(land_idx) + step_forward;

if isempty(land_idx)
    drop_info = [];
    return
end

trim_fcn = @(data) data(1:find(data.time > land_time, 1), :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);


drop_info = struct( ...
    'gps_drop', [data_gps.data(1,1:2), data_baro.data(1)], ...
    'vel_drop', data_gps_vel.data(1,:),...
    'gps_land', [data_gps.data(end,1:2), data_baro.data(end)], ...
    'time_drop', drop_time, ...
    'time_land', land_time ...
    );

end