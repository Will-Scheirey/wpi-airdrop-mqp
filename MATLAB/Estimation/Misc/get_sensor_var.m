function [var_accel, var_gyro, var_gps, var_mag, var_baro] = get_sensor_var(data_stationary)
    % GET_SENSOR_VAR Calculates sensor variance from measurements
    %   This function is used for sensor calibration before flight,
    %   calculating variance per sensor axis
    %   
    % INPUTS:
    %   data_stationary : Measurements during the calibration period
    %       .data_accel
    %       .data_gyro
    %       .data_gps
    %       .data_mag
    %       .data_baro
    % OUTPUTS:
    %   var_accel : Calculated accelerometer variance
    %   var_gyro  : Calculated gyroscope variance
    %   var_gps   : Calculated GPS position variance
    %   var_mag   : Calculated magnetometer variance
    %   var_baro  : Calculated barometer

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