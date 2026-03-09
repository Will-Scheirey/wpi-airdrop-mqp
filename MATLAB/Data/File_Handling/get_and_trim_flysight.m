function [data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps] = get_and_trim_flysight(sensor_filename, gps_filename)


[data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_flysight_data(sensor_filename, gps_filename, false);



[~, data_moving] = trim_flysight(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);

data_accel = data_moving.data_accel;
data_gyro = data_moving.data_gyro;
data_mag = data_moving.data_mag;
data_gps = data_moving.data_gps;
data_baro = data_moving.data_baro;
data_gps_vel = data_moving.data_gps_vel;
data_flysight_sensor = data_moving.data_flysight_sensor;
data_flysight_gps = data_moving.data_flysight_gps;

end