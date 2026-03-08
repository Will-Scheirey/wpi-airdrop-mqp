function [var_accel, var_gyro, var_gps, var_mag, var_baro] = calibrate_sensors(data_stationary)
    accel = data_stationary.data_accel;
    gyro  = data_stationary.data_gyro;
    gps   = data_stationary.data_gps;
    mag   = data_stationary.data_mag; 
    baro  = data_stationary.data_baro;

    var_accel = var(accel.data);
    var_gyro = var(gyro.data);
    var_gps = var(gps.data);
    var_mag = var(mag.data);
    var_baro = var(baro.data);    
end