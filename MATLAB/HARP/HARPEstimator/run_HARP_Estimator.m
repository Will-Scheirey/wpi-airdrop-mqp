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


%% PLOT
% Create 3D plot
figure(1)
clf
hold on;
grid on;

% Plot all drop trajectories
for i = 1:num_sims
    traj = all_results{i}.propagator.trajectory;
    plot3(traj(:,1), traj(:,2), traj(:,3), ...
          'Color', [0.5 0.5 0.8 0.3], 'LineWidth', 0.8);
end

% Plot HARP position (Landing point)

plot3(harp.position_x, harp.position_y, 0, ...
      'ro', 'MarkerSize', 25, 'LineWidth', 3, 'DisplayName', 'HARP Landing Point');

plot3(carp_data.land_location(1), carp_data.land_location(2), 0, 'mo', 'MarkerSize', 25, 'DisplayName', 'Actual Landing Point');
