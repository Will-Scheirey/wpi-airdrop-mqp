function plotHARP2(outputs, inputs, flight_traj)
    % Plot HARP with 4 views: 3D trajectory, side view, top view, and wind vectors
    figure('Position', [50, 50, 1600, 1200]);
    
    % Extract key positions
    pi_x = outputs.harp.position_x; %inputs.altitude.landing_location(:,1);
    pi_y = outputs.harp.position_y; %inputs.altitude.landing_location(:,2);
    pi_z = 0; % Ground level
    
    harp_x = 0;
    harp_y = 0;
    harp_z = inputs.altitude.dropIndicatedTrue; % Release altitude
    
    % Calculate intermediate points for trajectory
    dweDir = outputs.deployedVector.direction + 180;
    dwe_x = outputs.deployedVector.windEffect * sind(dweDir);
    dwe_y = outputs.deployedVector.windEffect * cosd(dweDir);
    
    if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
        hvdeDir = outputs.hvVector.direction + 180;
        hvde_x = outputs.hvVector.de * sind(hvdeDir);
        hvde_y = outputs.hvVector.de * cosd(hvdeDir);
        mid_x = dwe_x + hvde_x;
        mid_y = dwe_y + hvde_y;
        % Approximate altitude for HALO (HV drift occurs at high altitude)
        mid_z = inputs.altitude.dropIndicatedTrue * 0.7;  %Rough estimate
    else
        mid_x = dwe_x;
        mid_y = dwe_y;
        mid_z = inputs.altitude.dropIndicatedTrue * 0.5; % Midpoint altitude
    end
    
    ftdDir = inputs.aircraft.magneticCourse + 180;
    ftd_x = outputs.ftd.distance * sind(ftdDir);
    ftd_y = outputs.ftd.distance * cosd(ftdDir);
    
    % LAR circles
    theta = linspace(0, 2*pi, 100);
    lar_x = outputs.lar.adjustedRadius * cos(theta) + harp_x;
    lar_y = outputs.lar.adjustedRadius * sin(theta) + harp_y;
    lar_max_x = outputs.lar.maxRadius * cos(theta) + harp_x;
    lar_max_y = outputs.lar.maxRadius * sin(theta) + harp_y;
    
    % Run-in line
    line_length = max(outputs.harp.distance * 1.5, outputs.lar.maxRadius * 2);
    run_x = line_length * sind(inputs.aircraft.magneticCourse);
    run_y = line_length * cosd(inputs.aircraft.magneticCourse);
    
    %% SUBPLOT 1: 3D Trajectory View
    subplot(2, 2, 1);
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

    
    % Plot vertical line from HARP to ground
    % plot3([harp_x, harp_x], [harp_y, harp_y], [harp_z, 0], 'k:', 'LineWidth', 1);
    
    % Plot run-in line at altitude
    % plot3([0, run_x], [0, run_y], [harp_z, harp_z], 'k:', 'LineWidth', 1);

    flight_traj = flight_traj + ([harp_x, harp_y, harp_z] - flight_traj(1, :));
    plot3(flight_traj(:, 1), flight_traj(:, 2), flight_traj(:, 3), '-m', 'LineWidth', 1)

    err = [pi_x, pi_y, pi_z] - flight_traj(end, :);
    fprintf("Position Error: %0.2f ft - (%0.2f, %0.2f) ft", norm(err), err(1), err(2));
    
    % Labels
    text(pi_x, pi_y, pi_z + 500, 'PI', 'FontSize', 10, 'FontWeight', 'bold');
    text(harp_x, harp_y, harp_z + 500, 'HARP', 'FontSize', 10, 'FontWeight', 'bold');
    
    xlabel('East (ft)', 'FontSize', 10);
    ylabel('North (ft)', 'FontSize', 10);
    zlabel('Altitude (ft)', 'FontSize', 10);
    title('3D Trajectory View', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Trajectory', 'PI', 'HARP', 'Estimated Trajectory', 'Location', 'best');

    hold off;
    
    %% SUBPLOT 2: Side View (East-Altitude)
    subplot(2, 2, 2);
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
    
    % Plot key points
    plot(pi_x, pi_z, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    plot(harp_x, harp_z, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');
    
    % Plot vertical line from HARP
    plot([harp_x, harp_x], [harp_z, 0], 'k:', 'LineWidth', 1);
    
    % Ground line
    xlim_vals = xlim;
    plot(xlim_vals, [0, 0], 'k-', 'LineWidth', 2);
    
    xlabel('East (ft)', 'FontSize', 10);
    ylabel('Altitude (ft)', 'FontSize', 10);
    title('Side View (East-Altitude)', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Trajectory', 'PI', 'Release Point', 'Location', 'best');
    hold off;
    
    %% SUBPLOT 3: Top-Down View (Original 2D map)
    subplot(2, 2, 3);
    hold on;
    grid on;
    axis equal;
    
    % Plot PI at origin
    plot(pi_x, pi_y, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    text(0, 100, 'PI', 'FontSize', 10, 'FontWeight', 'bold');
    
    % Plot HARP
    plot(harp_x, harp_y, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');
    text(harp_x + 100, harp_y + 100, 'HARP', 'FontSize', 10, 'FontWeight', 'bold');
    
    % Plot LAR circles
    %plot(lar_x, lar_y, 'b--', 'LineWidth', 2);
    %plot(lar_max_x, lar_max_y, 'b:', 'LineWidth', 1);
    
    % Plot vectors
    % Deployed wind effect
    quiver(0, 0, dwe_x, dwe_y, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(dwe_x/2, dwe_y/2 + 100, 'DWE', 'FontSize', 9, 'Color', 'g');
    
    % High velocity drift (if HALO)
    if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
        quiver(dwe_x, dwe_y, hvde_x, hvde_y, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        text(dwe_x + hvde_x/2, dwe_y + hvde_y/2 + 100, 'HVDE', 'FontSize', 9, 'Color', 'm');
        start_x = dwe_x + hvde_x;
        start_y = dwe_y + hvde_y;
    else
        start_x = dwe_x;
        start_y = dwe_y;
    end
    
    % Forward travel distance
    quiver(start_x, start_y, ftd_x, ftd_y, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text(start_x + ftd_x/2 + 100, start_y + ftd_y/2, 'FTD', 'FontSize', 9);
    
    % Run-in heading line
    %plot([0, run_x], [0, run_y], 'k:', 'LineWidth', 1);
    
    xlabel('East (ft)', 'FontSize', 10);
    ylabel('North (ft)', 'FontSize', 10);
    title('Top-Down View', 'FontSize', 12, 'FontWeight', 'bold');
    legend('PI', 'HARP', sprintf('LAR (%.0f%% DDD)', inputs.safety.percentage*100), ...
        'LAR (100% DDD)', 'Location', 'best');
    hold off;
    
    %% SUBPLOT 4: 3D Wind Vector Diagram
    subplot(2, 2, 4);
    hold on;
    grid on;
    view(45, 30);
    
    % Origin point
    plot3(harp_x, harp_y, harp_z, 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'k');
    
    
    % Deployed wind effect vector
    dwe_vector_x = dwe_x - start_x;
    dwe_vector_y = dwe_y - start_y;
    % Scale for visualization
    scale = 1;
    quiver3(0, 0, 0, dwe_vector_x*scale, dwe_vector_y*scale, -harp_z*0.3, ...
        'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
    text(dwe_vector_x*scale/2, dwe_vector_y*scale/2, -harp_z*0.15, ...
        'DWE', 'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');
    
    % High velocity drift vector (if HALO)
    if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
        hvde_vector_x = hvde_x;
        hvde_vector_y = hvde_y;
        quiver3(0, 0, -harp_z*0.3, hvde_vector_x*scale, hvde_vector_y*scale, -harp_z*0.4, ...
            'm', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
        text(hvde_vector_x*scale/2, hvde_vector_y*scale/2, -harp_z*0.5, ...
            'HVDE', 'FontSize', 10, 'Color', 'm', 'FontWeight', 'bold');
    end
    
    % Aircraft velocity vector
    ftd_scale = 0.3; % Scale down for visualization
    quiver3(0, 0, -harp_z*0.7, ftd_x*ftd_scale, ftd_y*ftd_scale, 0, ...
        'k', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
    text(ftd_x*ftd_scale/2, ftd_y*ftd_scale/2, -harp_z*0.7, ...
        'Aircraft', 'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');
    
    % Resultant vector to HARP
    quiver3(0, 0, 0, harp_x*0.5, harp_y*0.5, -harp_z, ...
        'r', 'LineWidth', 3, 'MaxHeadSize', 0.8);
    text(harp_x*0.25, harp_y*0.25, -harp_z/2, ...
        'Total', 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
    
    % Add wind direction indicators if available
    if isfield(inputs, 'winds')
        % This section can be expanded if wind profile data is available
        title_str = '3D Wind Vectors';
    else
        title_str = '3D Effect Vectors';
    end
    
    xlabel('East (ft)', 'FontSize', 10);
    ylabel('North (ft)', 'FontSize', 10);
    zlabel('Altitude (ft)', 'FontSize', 10);
    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    
    % Set axis limits for better visualization
    max_val = max([abs(dwe_x), abs(dwe_y), harp_z]) * 0.6;
    xlim([-max_val, max_val]);
    ylim([-max_val, max_val]);
    zlim([-harp_z, harp_z*0.1]);
    
    legend('Origin', 'DWE', 'Location', 'best');
    hold off;
    
    %% Overall title
    sgtitle(sprintf('%s HARP Solution - %s | Altitude: %.0f m | Distance: %.0f m', ...
        inputs.mission.type, inputs.parachute.type, harp_z, outputs.harp.distance), ...
        'FontSize', 14, 'FontWeight', 'bold');
end