function displayHARPSummary(outputs, inputs)
            % Display formatted HARP solution summary
            
            fprintf('\n========================================\n');
            fprintf('HARP COMPUTATION SUMMARY\n');
            fprintf('========================================\n\n');
            
            fprintf('Mission Type: %s\n', inputs.mission.type);
            fprintf('Parachute: %s (%d lbs)\n\n', inputs.parachute.type, inputs.parachute.weight);
            
            fprintf('ALTITUDES:\n');
            fprintf('  Drop Absolute Altitude: %d ft AGL\n', round(outputs.altitudes.dropAbsolute));
            fprintf('  Stabilization Altitude: %d ft AGL\n', round(outputs.altitudes.stabilization));
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  Actuation Altitude: %d ft AGL\n', round(outputs.altitudes.absoluteActuation));
                fprintf('  Full Deployment Altitude: %d ft AGL\n', round(outputs.altitudes.fullDeployment));
            end
            fprintf('\n');
            
            fprintf('BALLISTIC WINDS:\n');
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  HV Ballistic Wind: %03d° / %d kts\n', ...
                    round(outputs.winds.hvBallistic(1)), round(outputs.winds.hvBallistic(2)));
            end
            fprintf('  Deployed Ballistic Wind: %03d° / %d kts\n', ...
                round(outputs.winds.deployedBallistic(1)), round(outputs.winds.deployedBallistic(2)));
            fprintf('  Drop Altitude Wind: %03d° / %d kts\n\n', ...
                round(outputs.winds.dropAltitude(1)), round(outputs.winds.dropAltitude(2)));
            
            fprintf('VECTORS:\n');
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  HV Drift Effect: %d m\n', round(outputs.hvVector.de));
            end
            fprintf('  Deployed Wind Effect: %d m\n', round(outputs.deployedVector.windEffect));
            fprintf('  Forward Travel Distance: %d m\n', round(outputs.ftd.distance));
            fprintf('  Total Wind Effect: %d m\n\n', round(outputs.harp.totalWindEffect));
            
            fprintf('HARP POSITION:\n');
            fprintf('  Distance from PI: %d m\n', round(outputs.harp.distance));
            fprintf('  Bearing from PI: %03d°\n', round(outputs.harp.bearing));
            fprintf('  Coordinates (m): E %.1f, N %.1f\n\n', ...
                outputs.harp.position_x, outputs.harp.position_y);
            
            fprintf('LAUNCH ACCEPTABILITY REGION:\n');
            fprintf('  Maximum Radius (100%% DDD): %d m\n', round(outputs.lar.maxRadius));
            fprintf('  Adjusted Radius (%d%% DDD): %d m\n', ...
                round(inputs.safety.percentage*100), round(outputs.lar.adjustedRadius));
            fprintf('  Usable Green Light Length: %d m\n', round(outputs.lar.usableLength));
            fprintf('  Safety Factor: %d ft\n\n', inputs.safety.factor);
            
            fprintf('TIMING:\n');
            fprintf('  Groundspeed: %d kts\n', round(outputs.timing.groundspeed));
            fprintf('  Stopwatch Time: %.1f sec\n', outputs.timing.stopwatchTime);
            fprintf('  Green Light Time: %.1f sec\n', outputs.timing.greenLightTime);
            fprintf('  Red Light Time: %.1f sec\n\n', outputs.timing.redLightTime);
            
            fprintf('========================================\n');
        end