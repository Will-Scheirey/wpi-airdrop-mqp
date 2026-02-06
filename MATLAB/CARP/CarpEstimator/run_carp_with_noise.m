clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN153_Lt1_n16_08052025_Inside";
full_dir = fullfile(parent_dir, drop_dir);

%% Run Data
data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;

% Number of simulations
num_sims = 1;

all_results = cell(num_sims, 1);

%% CARP
for i = 1:num_sims
    all_results{i} = Carp_Estimator('carp_data', carp_data);
   
    if mod(i, 10) == 0
        fprintf('Completed %d/%d simulations\n', i, num_sims);
    end
end

%% PLOT

disp('Results structure:');
disp(all_results{1});

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

% Plot CARP position (release point)
carp = all_results{1}.carp;
ftd_meters = carp.ftd * 0.9144;  % yards to meters
carp_east = ftd_meters * sind(carp.dz_heading);
carp_north = ftd_meters * cosd(carp.dz_heading);
carp_alt = carp.true_alt * 0.3048;  % feet to meters

plot3(carp_east, carp_north, 0, ...
      'ro', 'MarkerSize', 25, 'LineWidth', 3, 'DisplayName', 'CARP Landing Point');

% Plot all impact points
for i = 1:num_sims
    final_pos = all_results{i}.propagator.trajectory(end, :);
    plot3(final_pos(1), final_pos(2), final_pos(3), ...
          'b.', 'MarkerSize', 10);
end

plot3(carp_data.land_location(1), carp_data.land_location(2), 0, 'rx', 'MarkerSize', 10)

plot3(carp_data.relative_traj(:, 1), carp_data.relative_traj(:, 2), carp_data.relative_traj(:, 3))

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');
title(sprintf('%d Airdrop Simulations with CARP Estimate', num_sims));
legend('Trajectories', 'CARP Landing Point', 'Impact Points', 'Location', 'best');
view(45, 30);
hold off;

% Calculate impact statistics
all_impacts = zeros(num_sims, 3);
for i = 1:num_sims
    all_impacts(i, :) = all_results{i}.propagator.trajectory(end, :);
end
axis equal

