% RUN_HARP_ESTIMATOR Entry point for running the HARP estimator on a single drop.
%   Loads flight data from a specified drop directory, converts it to HARP
%   inputs, runs the HARP computation, runs the high-fidelity dynamic model,
%   and generates summary output and plots.
%
%   Update 'drop_dir' to point to the desired drop subdirectory inside
%   'haars_data' before running.
%
% DEPENDENCIES (must be on the MATLAB path):
%   get_flight_estimates      - Load and process raw flight data
%   convertDataOutToInputs    - Convert data_out to HARP inputs format
%   computeHARP               - Run HARP estimation pipeline
%   HARP_Dynamic_Model        - Run high-fidelity trajectory propagator
%   displayHARPSummary        - Print formatted results summary
%   plotHARP2                 - Generate trajectory comparison plots
%   weather
%   haars_data

clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN151_Lt1_n14_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;

%% Converts Data into Inputs for HARP Computer
close all
inputs = convertDataOutToInputs(data_out);
[outputs, inputs] = computeHARP(inputs); 

dynamic_model = HARP_Dynamic_Model('inputs', inputs, 'data_out', data_out, 'carp_data', carp_data);

% Display summary
displayHARPSummary(outputs, inputs);
plotHARP2(outputs, inputs, data_out.carp.relative_traj, data_out.carp.planned_relative_landing, dynamic_model);
