function [data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data_2()

    t_start = 1.21e7;
    t_end = 1.24e7;

    data = readtable("payloadFlightIREC2024.csv");

    data = data(all([data.timestamp > t_start, data.timestamp < t_end], 2), :);
    data.timestamp = (data.timestamp - data.timestamp(1)) / 1000;

    data_accel = table(data.timestamp, [data.accelX, data.accelY, data.accelZ] * 9.81, 'VariableNames', {'time', 'data'});
    data_gyro = table(data.timestamp, deg2rad([data.gyroX, data.gyroY, data.gyroZ]), 'VariableNames', {'time', 'data'});

    data_gps_lla = [data.gpsLat/1e7, data.gpsLong/1e7, data.altitude];
    data_gps_enu = lla2enu(data_gps_lla, data_gps_lla(1, :), 'flat');

    data_gps = table(data.timestamp, data_gps_enu, 'VariableNames', {'time', 'data'});
    % data_gps = data_gps(1:100:end, :);

    data_mag = table(data.timestamp, [data.magX, data.magY, data.magZ]/1e3, 'VariableNames', {'time', 'data'});
    data_baro = table(data.timestamp, data.altitude - data.altitude(1), 'VariableNames', {'time', 'data'});
end
