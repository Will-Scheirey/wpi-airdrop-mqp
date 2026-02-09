clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";
drop_dir = "DN149_Lt1_n12_08052025_side";
full_dir = fullfile(parent_dir, drop_dir);

%% Get flight Estimates
data_out = get_flight_estimates(full_dir);

carp_data = data_out.carp;

%% Run Simulations
% Number of simulations
num_sims = 1;

all_results = cell(num_sims, 1);

for i = 1:num_sims
    all_results{i} = Carp_Estimator('carp_data',carp_data);
    
    if mod(i, 10) == 0
        fprintf('Completed %d/%d simulations\n', i, num_sims);
    end
end

%% EXTRACT SIM DATA
t_sim = all_results{1}.propagator.t_plot;
y_sim = all_results{1}.propagator.y_sim;
[f_d_c, windspeed_c, aoa_c_curr] = get_model_property(t_sim, all_results{1}.propagator.y_sim, all_results{1}.propagator.model_obj, 'drag_force_c', 'windspeed_c', 'aoa_c_curr');

vel_inertial = zeros(length(t_sim), 3);
f_d_inertial = zeros(length(t_sim), 3);

for idx = 1:length(t_sim)

    vel = all_results{1}.propagator.y_sim(idx, 4:6)';

    quat_p = all_results{1}.propagator.y_sim(idx, 7:10);
    rotm_p = ecef2body_rotm(quat_p);

    quat_c = all_results{1}.propagator.y_sim(idx, 20:23);
    rotm_c = ecef2body_rotm(quat_c);

    vel_inertial(idx, :) = rotm_p * vel;
    f_d_inertial(idx, :) = rotm_c * f_d_c(idx, :)';
end


%% TEST

dt = data_out.t_plot(2) - data_out.t_plot(1);
mean_win = 3 / dt;

drop_idx = find(data_out.tspan > data_out.t_plot_drop(1), 1);
land_idx = find(data_out.tspan > data_out.t_plot_drop(2), 1);

accel_calc = data_out.kf.accel_calc_all;
accel_calc = accel_calc(drop_idx:land_idx, :);

accel_vert = accel_calc(:, 3);

vel_est = data_out.drop_estimates_smoothed.vel;
vel_vert = vel_est(:, 3);
diff_accel = diff(vel_vert) / dt;

figure(1)
clf

plot(diff_accel, 'LineWidth', 1.5, 'DisplayName', 'Accel From Diff'); hold on
plot(accel_vert, 'LineWidth', 1.5, 'DisplayName', 'Accel Calc')
% plot()

legend
return

%% PLOT
disp('Results structure:');
disp(all_results{1});

% Create 3D plot
fig_idx = new_fig(1);
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
carp_east = ftd_meters * sind(carp_data.heading);
carp_north = ftd_meters * cosd(carp_data.heading);
carp_alt = carp_data.altitude * 0.3048;  % feet to meters

plot3(carp_east, carp_north, 0, ...
      'ro', 'MarkerSize', 25, 'LineWidth', 3, 'DisplayName', 'CARP Landing Point');

plot3(carp_data.land_location(1), carp_data.land_location(2), 0, 'mo', 'MarkerSize', 25, 'DisplayName', 'Actual Landing Point');

% Plot all impact points
for i = 1:num_sims
    final_pos = all_results{i}.propagator.trajectory(end, :);
    plot3(final_pos(1), final_pos(2), final_pos(3), ...
          'b.', 'MarkerSize', 10);
end

plot3(carp_data.relative_traj(:, 1), carp_data.relative_traj(:, 2), carp_data.relative_traj(:, 3))

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');
title(sprintf('%d Airdrop Simulations with CARP Estimate', num_sims));
legend('Trajectories', 'CARP Landing Point', 'Impact Points', 'Location', 'best');
view(45, 30);

num = length(data_out.weather.win_speed);
wind_vec = ks2mps(data_out.weather.win_speed) / 1000 .* -[sind(data_out.weather.wind_direction), cosd(data_out.weather.wind_direction)];
% quiver3(zeros(num, 1), zeros(num, 1), ft2m(data_out.weather.alt_agl * 1000), wind_vec(:, 1), wind_vec(:, 2), zeros(num, 1))

hold off;

axis equal

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1);
plot(data_out.drop_t_plot, data_out.drop_estimates_smoothed.vel(:, 1), 'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on
plot(t_sim, vel_inertial(:, 1), 'LineWidth', 1.5, 'DisplayName', 'Simulated')
legend
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("Vel X")
xlim([data_out.drop_t_plot(1), data_out.drop_t_plot(end)])

subplot(3,1,2);
plot(data_out.drop_t_plot, data_out.drop_estimates_smoothed.vel(:, 2), 'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on
plot(t_sim, vel_inertial(:, 2), 'LineWidth', 1.5, 'DisplayName', 'Simulated')
legend
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("Vel Y")
xlim([data_out.drop_t_plot(1), data_out.drop_t_plot(end)])

subplot(3,1,3);
plot(data_out.drop_t_plot, data_out.drop_estimates_smoothed.vel(:, 3), 'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on
plot(t_sim, vel_inertial(:, 3), 'LineWidth', 1.5, 'DisplayName', 'Simulated')
legend
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("Vel Z")
xlim([data_out.drop_t_plot(1), data_out.drop_t_plot(end)])

fig_idx = new_fig(fig_idx);
clf
plot(t_sim, vecnorm(f_d_inertial, 2, 2))
title("Drag Force Norm")

fig_idx = new_fig(fig_idx);
clf
plot(t_sim, f_d_inertial)
title("Drag Force Norm")

fig_idx = new_fig(fig_idx);
clf
plot(t_sim, f_d_inertial / 1e5)
legend("X", "Y", "Z")

fig_idx = new_fig(fig_idx);
clf

plot3(traj(:,1), traj(:,2), traj(:,3), ...
      'Color', [0.5 0.5 0.8 0.3], 'LineWidth', 0.8); hold on

step = 50;

h = quiver3(traj(1:step:end,1), traj(1:step:end,2), traj(1:step:end,3), f_d_inertial(1:step:end,1), f_d_inertial(1:step:end, 2), f_d_inertial(1:step:end, 3), 5, 'Clipping', 'off', 'LineWidth', 1.5);
h.MaxHeadSize = 0.1;
xlabel("X")
ylabel("Y")
zlabel("Z")

step = 50;

fig_idx = new_fig(fig_idx);
clf
plot3(traj(:,1), traj(:,2), traj(:,3), ...
      'Color', [0.5 0.5 0.8 0.3], 'LineWidth', 0.8); hold on
h = quiver3(traj(1:step:end,1), traj(1:step:end,2), traj(1:step:end,3), windspeed_c(1:step:end,1), windspeed_c(1:step:end, 2), windspeed_c(1:step:end, 3), 5, 'Clipping', 'off', 'LineWidth', 1.5);
h.MaxHeadSize = 0.1;
xlabel("X")
ylabel("Y")
zlabel("Z")

fig_idx = new_fig(fig_idx);
clf
plot(t_sim, aoa_c_curr)
return
fig_idx = new_fig(fig_idx);
clf
run_animation(t_sim, y_sim, 1, 1, false)