% CARP_FROM_FLIGHT Load and extract CARP data from a single drop's flight estimates.
%   Short entry-point script that loads processed flight data for a
%   specified drop directory and extracts the CARP sub-struct. Useful for
%   inspecting or debugging carp_data without running the full estimator.
%
%   Update 'drop_dir' to point to the desired drop subdirectory inside
%   'haars_data' before running.
%
% OUTPUTS (workspace variables):
%   data_out  : Full flight estimates struct from get_flight_estimates
%   carp_data : CARP sub-struct (data_out.carp), containing aircraft state
%               and environmental conditions at the computed release point

clear; clc;

addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN149_Lt1_n12_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;