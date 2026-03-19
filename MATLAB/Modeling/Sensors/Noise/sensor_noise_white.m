function y_out = sensor_noise_white(y, std_dev)
% SENSOR_NOISE_WHITE Applies white sensor noise to values
%   This function applies white gaussian noise to values with a specified
%   standard deviation
%
% INPUTS:
%   y : The un-noised values
%   std_dev : The white noise standard deviation
%
% OUTPUTS:
%   y_out : The noised values

    noise = std_dev * randn(size(y));
    y_out = y + noise;
end