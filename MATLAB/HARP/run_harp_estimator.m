clear; clc; close all
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN151_Lt1_n14_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

harp_estimates = get_harp_estimates(full_dir);

%% Plotting and Display
displayHARPSummary(harp_estimates);
plotHARP2(harp_estimates);
