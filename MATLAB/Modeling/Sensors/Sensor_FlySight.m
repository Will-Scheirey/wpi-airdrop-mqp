function sensor_characteristics = Sensor_FlySight(Hz, G_mode)

    if nargin < 2
        G_mode = 3;
    end

    An_list = [70, 75, 80, 110];                  % [ug Hz^(-1/2)]
    
    accel_noise_density = An_list(G_mode) * 1e-6; % [g Hz^(-1/2)]
    gyro_noise_density  = deg2rad(3.8)    * 1e-3; % [rad Hz^(-1/2)]

    mag_rms = 3e-3; % [gauss]
    baro_std_dev = 0.17; % [m]

    sensor_characteristics = struct( ...
            'gps_std_dev',   1,...
            'grnd_vel_std_dev', 2,...
            'accel_std_dev', std_dev_from_noise(accel_noise_density, Hz),...
            'gyro_std_dev',  std_dev_from_noise(gyro_noise_density,  Hz),...
            'baro_std_dev',  baro_std_dev,...
            'mag_std_dev',   mag_rms / 0.55... % Divide by approximate field strength in Massachusetts
        );
end