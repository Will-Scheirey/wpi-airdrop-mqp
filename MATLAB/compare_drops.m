clearvars -except drops; clc;

% Plots and compares drops from a .mat file containing pre-processed drop
% data

if ~exist('drops', 'var')
    results_all = load("all_drops_results.mat");
    
    drops = results_all.results_all;
end

figure(1)
clf

num_drops = numel(drops);
did_labels = false;
success = zeros(num_drops, 1);

harp_all = zeros(num_drops, 2);
actual_all = zeros(num_drops, 2);
planned_all = zeros(num_drops, 2);
sim_all = zeros(num_drops, 2);
drop_alt_all = zeros(num_drops, 1);

for i=1:num_drops

    drop = drops{i};
    if ~strcmp(drop.status, 'success')
        continue
    end

    first_lat = drop.data_out.measurements.gps_all.GNSS.lat(1);
    first_lon = drop.data_out.measurements.gps_all.GNSS.lon(1);

    first_gps   = [first_lat, first_lon, 0];
    planned_gps = [drop.data_out.system_data.planned_impact_lat_lon, 0];

    traj_enu = drop.data_out.drop_estimates.pos;

    alt = traj_enu(1,3);

    start_idx = 1;
    if traj_enu(1,3) < 2e3
        start_idx = 80;
    end

    traj_enu = traj_enu(start_idx:end, :);

    traj_lla = enu2lla(traj_enu, first_gps, 'flat');

    traj = lla2enu(traj_lla, planned_gps, 'flat');

    traj(:, 3) = traj(:, 3) - traj(end, 3);

    times = drop.data_out.drop_t_plot;

    if times(end) - times(1) > 200
        continue
    end

    success(i) = 1;

    show_label = 'off';
    if ~did_labels
        show_label = 'on';
        did_labels = true;
    end

    plot3(traj(:, 1), traj(:, 2), traj(:, 3), '-k', 'DisplayName', 'Trajectory', 'HandleVisibility', show_label); hold on
    plot3(traj(1,1), traj(1,2), traj(1,3), '.b', 'MarkerSize', 20, 'DisplayName', 'Drop', 'HandleVisibility', show_label)
    plot3(traj(end,1), traj(end,2), traj(end,3), '.r', 'MarkerSize', 20, 'DisplayName', 'Landing', 'HandleVisibility', show_label)

    traj1 = drop.dynamic_model.trajectory;
    traj1(:, 1:2) = traj1(:, 1:2) + traj(1, 1:2);
    %{
    plot3(traj1(:, 1), traj1(:, 2), traj1(:, 3), '-b', 'HandleVisibility', show_label, 'DisplayName', 'Simulated Trajectory')
    plot3(traj1(end,1), traj1(end,2), traj1(end,3), '.m', 'MarkerSize', 20, 'DisplayName', 'Simulated Landing', 'HandleVisibility', show_label)
    %}

    actual_all(i, :) = traj(end, 1:2);
    planned_all(i, :) = -traj(1, 1:2);
    sim_all(i, :)    = traj1(end, 1:2);
    drop_alt_all(i)  = traj(1, 3);
end

success = success == 1;

plot3(0, 0, 0, 'gx', 'MarkerSize', 20, 'LineWidth', 5, 'DisplayName', 'Planned Landing Location');

xlabel("East (m)")
ylabel("North (m)")
zlabel("Up (m)")
title("Drop Trajectories Relative to Planned Landing Location")
legend('Location', 'northeast')
grid on
zlim([0, 6000])
% 
% view(0, 90);

actual_all = actual_all(success, :);
sim_all    = sim_all   (success, :);
drop_alt_all = drop_alt_all(success);
planned_all = planned_all(success, :);

sim_actual_diff = (actual_all - sim_all);
sim_actual_diff_avg = mean(sim_actual_diff);

planned_actual_diff = actual_all;

planned_actual_diff_avg = norm(mean(abs(planned_actual_diff))) / norm(mean(abs(planned_all)))
planned_actual_standard_dev = std(actual_all) ./ mean(abs(planned_all))
