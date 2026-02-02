clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN171_Lt3_n11_08072025_side_2";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;