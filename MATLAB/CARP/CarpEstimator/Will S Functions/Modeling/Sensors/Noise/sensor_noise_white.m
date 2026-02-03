function measurements = sensor_noise_white(y, std_dev)
%SENSOR_EMULATOR Summary of this function goes here
%   Detailed explanation goes here

    noise = std_dev * randn(size(y));

    measurements = y + noise;
end