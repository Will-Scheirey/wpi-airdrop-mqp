 function [data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor, ...
    data_flysight_gps] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps)

if isempty(data_gps)
    return
end

gps_shift = data_baro.time(end) - data_gps.time(end);
data_gps.time = data_gps.time + gps_shift;
data_gps_vel.time = data_gps_vel.time + gps_shift;
data_flysight_gps.GNSS.time = data_flysight_gps.GNSS.time + gps_shift;

vel_start = 30;
dur_back = 250;

vel_end = 0.5;
dur_forward = 60;

gps_vel_norm = vecnorm(data_gps_vel.data, 2, 2);

time_moving = data_gps_vel.time(find(gps_vel_norm > vel_start, 1));
start_time = time_moving - dur_back;

time_stopped = gps_vel_norm < vel_end;
stop_idx = find(all([time_stopped, data_gps_vel.time > time_moving], 2), 1);
stop_time = data_gps_vel.time(stop_idx) + dur_forward;

trim_fcn = @(data) data(find(data.time > start_time, 1):find(data.time > stop_time, 1), :);

data_accel = trim_fcn(data_accel);
data_gyro = trim_fcn(data_gyro);
data_mag = trim_fcn(data_mag);
data_gps = trim_fcn(data_gps);
data_baro = trim_fcn(data_baro);
data_gps_vel = trim_fcn(data_gps_vel);
data_flysight_gps.GNSS = trim_fcn(data_flysight_gps.GNSS);
end