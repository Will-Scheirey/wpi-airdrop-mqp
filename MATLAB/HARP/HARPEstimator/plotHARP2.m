% PLOTHARP2 Generate multi-panel visualizations of HARP trajectory and results.
%   Produces three separate figure windows comparing the HARP-estimated
%   trajectory against the actual flight trajectory and planned landing:
%     Figure 1 - 3D trajectory view with wind vectors
%     Figure 2 - Side view (East vs. Altitude)
%     Figure 3 - Top-down view (East vs. North)
%
%   All position units are converted from feet (HARP internals) to meters
%   for display. The flight trajectory is offset so the release point (HARP)
%   aligns with the origin.
%
% INPUTS:
%   outputs         : HARP outputs structure from computeHARP, containing:
%                       - harp.position_x/y  : HARP offset from PI (ft)
%                       - harp.distance      : HARP distance from PI (ft)
%                       - harp.bearing       : HARP bearing from PI (deg)
%                       - hvVector           : high-velocity vector struct
%                       - deployedVector     : deployed vector struct
%                       - winds.profile      : wind profile array
%                       - timing.groundspeed : aircraft groundspeed (kts)
%   inputs          : HARP inputs structure from convertDataOutToInputs,
%                     containing:
%                       - altitude.dropIndicatedTrue : drop altitude (ft)
%                       - mission.type               : 'HALO' or 'HAHO'
%                       - aircraft.magneticCourse    : aircraft course (deg)
%                       - winds.profile              : wind profile array
%   flight_traj     : Nx3 matrix of actual flight trajectory in ENU (m),
%                     as [East, North, Up] columns
%   planned_landing : 1x2 vector of planned landing position in ENU (m),
%                     as [East, North]
%   dynamic_model   : Struct output from HARP_Dynamic_Model containing:
%                       - trajectory : Mx3 ENU position array (m)
%
% OUTPUTS:
%   None. Three figure windows are created.
%
% NOTES:
%   - The first 79 rows of flight_traj are trimmed (flight_traj(80:end,:))
%     to remove pre-drop aircraft data.
%   - The mid-point altitude for HALO (mid_z) is a rough linear estimate
%     (70% of drop altitude) and is not physically derived.
%   - Position error between HARP-estimated PI and actual landing is printed
%     to the command window.
%
% See also COMPUTEHARP, HARP_DYNAMIC_MODEL

function plotHARP2(harp_estimates) %outputs, inputs, flight_traj, planned_landing, dynamic_model)

outputs = harp_estimates.outputs;
inputs  = harp_estimates.inputs;
flight_traj = harp_estimates.data_out.carp.relative_traj;
planned_landing = harp_estimates.data_out.carp.planned_relative_landing;
dynamic_model = harp_estimates.dynamic_model;

% Plot HARP with 4 views: 3D trajectory, side view, top view, and wind vectors

% Extract key positions
pi_x = ft2m(outputs.harp.position_x); %inputs.altitude.landing_location(:,1);
pi_y = ft2m(outputs.harp.position_y); %inputs.altitude.landing_location(:,2);
pi_z = 0; % Ground level

harp_x = 0;
harp_y = 0;
harp_z = ft2m(inputs.altitude.dropIndicatedTrue); % Release altitude

% Calculate intermediate points for trajectory
dweDir = -outputs.deployedVector.direction + 180;
dwe_x = outputs.deployedVector.windEffect * sind(dweDir);
dwe_y = outputs.deployedVector.windEffect * cosd(dweDir);


if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
    hvdeDir = -outputs.hvVector.direction + 180;
    hvde_x = ft2m(outputs.hvVector.de) * sind(hvdeDir);
    hvde_y = ft2m(outputs.hvVector.de) * cosd(hvdeDir);
    mid_x = dwe_x + hvde_x;
    mid_y = dwe_y + hvde_y;
    % Approximate altitude for HALO (HV drift occurs at high altitude)
    mid_z = ft2m(inputs.altitude.dropIndicatedTrue) * 0.7;  %Rough estimate
else
    mid_x = dwe_x;
    mid_y = dwe_y;
    mid_z = ft2m(inputs.altitude.dropIndicatedTrue) * 0.5; % Midpoint altitude
end


%% SUBPLOT 1: 3D Trajectory View
figure('Position', [50, 50, 1600, 1200]);
hold on;
grid on;
axis equal;
view(45, 30);

% Plot trajectory path
if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
    % HALO: HARP -> high altitude drift -> deployed drift -> PI
    plot3([harp_x, mid_x, dwe_x, pi_x], ...
        [harp_y, mid_y, dwe_y, pi_y], ...
        [harp_z, mid_z, mid_z*0.3, pi_z], ...
        'b-', 'LineWidth', 2);
else
    % HAHO: HARP -> deployed drift -> PI
    plot3([harp_x, dwe_x, pi_x], ...
        [harp_y, dwe_y, pi_y], ...
        [harp_z, mid_z, pi_z], ...
        'b-', 'LineWidth', 2);
end

% Plot key points
plot3(pi_x, pi_y, pi_z, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot3(harp_x, harp_y, harp_z, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');

flight_traj  = flight_traj(80:end,:);

planned_landing = planned_landing + ([harp_x, harp_y] - flight_traj(1, 1:2));

flight_traj = flight_traj + ([harp_x, harp_y, harp_z] - flight_traj(1, :));
plot3(flight_traj(:, 1), flight_traj(:, 2), flight_traj(:, 3), '-m', 'LineWidth', 2)

%Dynamic Model Trajectory
plot3(dynamic_model.trajectory(:, 1),dynamic_model.trajectory(:, 2),dynamic_model.trajectory(:, 3), 'LineWidth', 2);

% plot_wind_traj(inputs.winds.profile(:, 1), inputs.winds.profile(:, 2) + 180, inputs.winds.profile(:, 3), flight_traj, 50, 1/2);

plot3(planned_landing(1), planned_landing(2), 0, 'rx', 'MarkerSize', 20, 'DisplayName', 'Planned Landing');

err = [pi_x, pi_y, pi_z] - flight_traj(end, :);
fprintf("Position Error: %0.2f m - (%0.2f, %0.2f) ft", norm(err), err(1), err(2));

% Labels
text(pi_x + 10, pi_y, pi_z, 'Point of Impact', 'FontSize', 10, 'FontWeight', 'bold');
text(harp_x + 10, harp_y, harp_z, 'HARP', 'FontSize', 10, 'FontWeight', 'bold');

xlabel('East (m)', 'FontSize', 10);
ylabel('North (m)', 'FontSize', 10);
zlabel('Altitude (m)', 'FontSize', 10);
title('3D Trajectory View', 'FontSize', 12, 'FontWeight', 'bold');
legend('Trajectory', 'PI', 'HARP', 'Estimated Trajectory','Dynamic Model Trajectory', 'Planned Landing','Location', 'best');


hold off;
axis square

%% SUBPLOT 2: Side View (East-Altitude)
figure('Position', [50, 50, 1600, 1200]);
hold on;
grid on;


% Project trajectory onto East-Altitude plane
if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
    plot([harp_x, mid_x, dwe_x, pi_x], ...
        [harp_z, mid_z, mid_z*0.3, pi_z], ...
        'b-', 'LineWidth', 2);
else
    plot([harp_x, dwe_x, pi_x], ...
        [harp_z, mid_z, pi_z], ...
        'b-', 'LineWidth', 2);
end

plot(flight_traj(:, 1), flight_traj(:, 3), '-m', 'LineWidth', 1)

% Plot key points
plot(pi_x, pi_z, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot(harp_x, harp_z, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');

% Ground line
xlim_vals = xlim;
plot(xlim_vals, [0, 0], 'k-', 'LineWidth', 2);

xlabel('East (m)', 'FontSize', 10);
ylabel('Altitude (m)', 'FontSize', 10);
title('Side View (East-Altitude)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Trajectory', 'Actual Trajectory', 'PI', 'Release Point', 'Location', 'best');
hold off;

%% SUBPLOT 3: Top-Down View (Original 2D map)
figure('Position', [50, 50, 1600, 1200]);
hold on;
grid on;
axis equal;

% PI and HARP
plot(pi_x, pi_y, 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'PI');
text(pi_x + 10, pi_y + 10, 'PI', 'FontSize', 10, 'FontWeight', 'bold');

plot(harp_x, harp_y, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'HARP');
text(harp_x + 10, harp_y + 10, 'HARP', 'FontSize', 10, 'FontWeight', 'bold');

% Flight trajectory (top-down: columns 1 and 2 are X and Y)
plot(flight_traj(:, 1), flight_traj(:, 2), '-m', 'LineWidth', 1.5, 'DisplayName', 'Estimated Trajectory');

% Planned landing
plot(planned_landing(1), planned_landing(2), 'rx', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Planned Landing');

% Actual landing (end of flight_traj)
plot(flight_traj(end, 1), flight_traj(end, 2), 'ms', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Actual Landing');

plot([harp_x, mid_x, dwe_x, pi_x], ...
    [harp_y, mid_y, dwe_y, pi_y], ...
    'b-', 'LineWidth', 2, 'DisplayName', 'HARP Trajectory')

xlabel('East (m)', 'FontSize', 10);
ylabel('North (m)', 'FontSize', 10);
title('Top-Down View', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

hold off;

end