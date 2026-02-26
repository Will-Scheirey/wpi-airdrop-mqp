clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN150_Lt1_n13_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;

%% Converts Data into Inputs for HARP Computer
close all
inputs = convertDataOutToInputs(data_out);
[outputs, inputs] = computeHARP(inputs); 

dynamic_model = HARP_Dynamic_Model('inputs', inputs, 'data_out', data_out, 'carp_data', carp_data);

% Display summary
displayHARPSummary(outputs, inputs, dynamic_model);
plotHARP2(outputs, inputs, m2ft(data_out.carp.relative_traj), m2ft(data_out.carp.planned_relative_landing));

