function plotHARP(outputs, inputs)
            % Plot HARP and LAR on 2D map
            
            figure('Position', [100, 100, 800, 800]);
            hold on;
            grid on;
            axis equal;
            
            % Plot PI at origin
            plot(0, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
            text(0, 100, 'PI', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Plot HARP
            harp_x = outputs.harp.position_x;
            harp_y = outputs.harp.position_y;
            plot(harp_x, harp_y, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
            text(harp_x + 100, harp_y + 100, 'HARP', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Plot LAR (adjusted radius)
            theta = linspace(0, 2*pi, 100);
            lar_x = outputs.lar.adjustedRadius * cos(theta) + harp_x;
            lar_y = outputs.lar.adjustedRadius * sin(theta) + harp_y;
            plot(lar_x, lar_y, 'b--', 'LineWidth', 2);
            
            % Plot maximum LAR (100% DDD)
            lar_max_x = outputs.lar.maxRadius * cos(theta) + harp_x;
            lar_max_y = outputs.lar.maxRadius * sin(theta) + harp_y;
            plot(lar_max_x, lar_max_y, 'b:', 'LineWidth', 1);
            
            % Plot vectors
            % Deployed wind effect
            dweDir = outputs.deployedVector.direction + 180;
            dwe_end_x = outputs.deployedVector.windEffect * sind(dweDir);
            dwe_end_y = outputs.deployedVector.windEffect * cosd(dweDir);
            quiver(0, 0, dwe_end_x, dwe_end_y, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            text(dwe_end_x/2, dwe_end_y/2 + 100, 'DWE', 'FontSize', 10, 'Color', 'g');
            
            % High velocity drift (if HALO)
            if strcmp(inputs.mission.type, 'HALO') && outputs.hvVector.de > 0
                hvdeDir = outputs.hvVector.direction + 180;
                hvde_x = outputs.hvVector.de * sind(hvdeDir);
                hvde_y = outputs.hvVector.de * cosd(hvdeDir);
                quiver(dwe_end_x, dwe_end_y, hvde_x, hvde_y, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
                text(dwe_end_x + hvde_x/2, dwe_end_y + hvde_y/2 + 100, 'HVDE', 'FontSize', 10, 'Color', 'm');
                start_x = dwe_end_x + hvde_x;
                start_y = dwe_end_y + hvde_y;
            else
                start_x = dwe_end_x;
                start_y = dwe_end_y;
            end
            
            % Forward travel distance
            ftdDir = inputs.aircraft.magneticCourse + 180;
            ftd_x = outputs.ftd.distance * sind(ftdDir);
            ftd_y = outputs.ftd.distance * cosd(ftdDir);
            quiver(start_x, start_y, ftd_x, ftd_y, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            text(start_x + ftd_x/2 + 100, start_y + ftd_y/2, 'FTD', 'FontSize', 10);
            
            % Run-in heading line
            line_length = max(outputs.harp.distance * 1.5, outputs.lar.maxRadius * 2);
            run_x = line_length * sind(inputs.aircraft.magneticCourse);
            run_y = line_length * cosd(inputs.aircraft.magneticCourse);
            plot([0, run_x], [0, run_y], 'k:', 'LineWidth', 1);
            
            % Labels and formatting
            xlabel('East (meters)', 'FontSize', 12);
            ylabel('North (meters)', 'FontSize', 12);
            title(sprintf('%s HARP Solution - %s', inputs.mission.type, inputs.parachute.type), ...
                'FontSize', 14, 'FontWeight', 'bold');
            
            legend('PI', 'HARP', sprintf('LAR (%.0f%% DDD)', inputs.safety.percentage*100), ...
                'LAR (100% DDD)', 'Location', 'best');
            
            hold off;
        end