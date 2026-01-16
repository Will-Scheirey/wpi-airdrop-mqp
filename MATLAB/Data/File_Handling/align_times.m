function [data_gps,...
    data_gps_vel,...
    data_flysight_gps] = align_times(data_gps, data_baro, data_gps_vel, data_flysight_gps)


    gps_shift = data_baro.time(end) - data_gps.time(end);
    
    data_gps.time = data_gps.time + gps_shift;
    data_gps_vel.time = data_gps_vel.time + gps_shift;
    data_flysight_gps.GNSS.time = data_flysight_gps.GNSS.time + gps_shift;
end