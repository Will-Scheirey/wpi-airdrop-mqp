clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN152_Lt1_n15_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;

%% Converts Data into Inputs for HARP Computer
inputs = convertDataOutToInputs(data_out);
outputs = computeHARP(inputs); 

