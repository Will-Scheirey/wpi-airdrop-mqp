function [data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps] = get_flysight_data(sensor_filename, gps_filename, display)
    
    if nargin < 3
        display = false;
    end

    % Loading from the CSV can take multiple minutes. So, check if we have 
    % a .mat file already saved. If we do, use it. Otherwise, load the 
    % data and save a .mat file for next time.
    sensor_mat = replace(sensor_filename, "CSV", "mat");
    if isfile(sensor_mat)
        if display
            disp("Loading Flysight Sensor from .mat...")
        end
        data_flysight_sensor = load(sensor_mat).data_flysight_sensor;
    else
        if display
            disp("Loading Flysight Sensor from .csv...")
        end
        data_flysight_sensor = load_flysight_file(sensor_filename);
        save(sensor_mat, "data_flysight_sensor");
    end

    gps_mat = replace(gps_filename, "CSV", "mat");
    if isfile(gps_mat)
        if display
            disp("Loading Flysight GPS from .mat...")
        end
        data_flysight_gps = load(gps_mat).data_flysight_gps;
    else
        if display
            disp("Loading Flysight GPS from .csv...")
        end
        data_flysight_gps = load_flysight_file(gps_filename);
        save(gps_mat, "data_flysight_gps");
    end

    if display
        disp("Processing...")
    end

    imu_time = (data_flysight_sensor.IMU.time - data_flysight_sensor.IMU.time(1));
    data_accel = table(imu_time, [data_flysight_sensor.IMU.ax, data_flysight_sensor.IMU.ay, data_flysight_sensor.IMU.az] * 9.81, 'VariableNames', {'time', 'data'});
    
    gyro_meas = [data_flysight_sensor.IMU.wx, data_flysight_sensor.IMU.wy, data_flysight_sensor.IMU.wz];
    gyro_meas = deg2rad(gyro_meas);
    data_gyro = table(imu_time, gyro_meas, 'VariableNames', {'time', 'data'});

    mag_time = (data_flysight_sensor.MAG.time - data_flysight_sensor.MAG.time(1));
    data_mag = table(mag_time, [data_flysight_sensor.MAG.x, data_flysight_sensor.MAG.y, data_flysight_sensor.MAG.z], 'VariableNames', {'time', 'data'});

    baro_alt = atmospalt(data_flysight_sensor.BARO.pressure);
    % baro_alt = baro_alt - baro_alt(1);
    baro_time = (data_flysight_sensor.BARO.time - data_flysight_sensor.BARO.time(1));
    data_baro = table(baro_time, baro_alt, 'VariableNames', {'time', 'data'});

    if ~isfield(data_flysight_gps, "GNSS")
        data_gps = table();
        data_gps_vel = table();
        return
    end

    data_gps_lla = [data_flysight_gps.GNSS.lat, data_flysight_gps.GNSS.lon, data_flysight_gps.GNSS.hMSL];
    data_gps_enu = lla2enu(data_gps_lla(:, 1:3), data_gps_lla(1, 1:3), 'flat');

    data_gps = table(data_flysight_gps.GNSS.time, data_gps_enu, 'VariableNames', {'time', 'data'});

    data_gps_vel = table(data_flysight_gps.GNSS.time, [data_flysight_gps.GNSS.velE, data_flysight_gps.GNSS.velN, -data_flysight_gps.GNSS.velD], 'VariableNames', {'time', 'data'});
end