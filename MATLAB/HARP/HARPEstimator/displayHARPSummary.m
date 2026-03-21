% DISPLAYHARPSUMMARY Print a formatted summary of HARP computation results.
%   Prints a summary of the HARP solution to the command
%   window, including mission type, parachute info, altitude breakdowns,
%   ballistic winds, vector distances, and the final HARP position and
%   bearing from the Point of Impact (PI).
%
% INPUTS:
%   outputs : HARP outputs structure returned by computeHARP, containing:
%               - altitudes   : struct of computed altitude values (ft)
%               - winds       : struct of ballistic wind vectors
%               - hvVector    : struct of high-velocity drift values (HALO)
%               - deployedVector : struct of deployed canopy drift values
%               - ftd         : struct of forward travel distance values
%               - harp        : struct of final HARP position values
%               - timing      : struct of groundspeed and timing values
%   inputs  : HARP inputs structure returned by convertDataOutToInputs,
%             containing:
%               - mission.type       : 'HALO' or 'HAHO'
%               - parachute.type     : parachute model name string
%               - parachute.weight   : system weight (lbs)
%               - altitude.dropIndicatedTrue : drop altitude (ft)
%
% OUTPUTS:
%   None. Results are printed to the command window via fprintf.

function displayHARPSummary(outputs, inputs)
            % Display formatted HARP solution summary
            
            fprintf('\n========================================\n');
            fprintf('HARP COMPUTATION SUMMARY\n');
            fprintf('========================================\n\n');
            
            fprintf('Mission Type: %s\n', inputs.mission.type);
            fprintf('Parachute: %s (%d kgs)\n\n', inputs.parachute.type,  round(lb2kg(inputs.parachute.weight)));
            
            fprintf('ALTITUDES:\n');
            fprintf('  Drop Absolute Altitude: %d m AGL\n', round(ft2m(outputs.altitudes.dropAbsolute)));
            fprintf('  Stabilization Altitude: %d m AGL\n', round(ft2m(outputs.altitudes.stabilization)));
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  Actuation Altitude: %d m AGL\n', round(ft2m(outputs.altitudes.absoluteActuation)));
                fprintf('  Full Deployment Altitude: %d m AGL\n', round(ft2m(outputs.altitudes.fullDeployment)));
            end
            fprintf('\n');
            
            fprintf('BALLISTIC WINDS:\n');
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  HV Ballistic Wind: %03d° / %d kts\n', ...
                    round(outputs.winds.hvBallistic(1)), round(ft2m(outputs.winds.hvBallistic(2))));
            end
            fprintf('  Deployed Ballistic Wind: %03d° / %d kts\n', ...
                round(outputs.winds.deployedBallistic(1)), round(ft2m(outputs.winds.deployedBallistic(2))));
            fprintf('  Drop Altitude Wind: %03d° / %d kts\n\n', ...
                round(outputs.winds.dropAltitude(1)), round(outputs.winds.dropAltitude(2)));
            
            fprintf('VECTORS:\n');
            if strcmp(inputs.mission.type, 'HALO')
                fprintf('  HV Drim Effect: %d m\n', round(outputs.hvVector.de));
            end
            fprintf('  Deployed Wind Effect: %d m\n', round(ft2m(outputs.deployedVector.windEffect)));
            fprintf('  Forward Travel Distance: %d m\n', round(ft2m(outputs.ftd.distance)));
            fprintf('  Total Wind Effect: %d m\n\n', round(ft2m(outputs.harp.totalWindEffect)));
            
            fprintf('HARP POSITION:\n');
            fprintf('  Distance from PI: %d m\n', round(ft2m(outputs.harp.distance)));
            fprintf('  Bearing from PI: %03d°\n', round(outputs.harp.bearing));
            fprintf('  Location (m): E %.1f, N %.1f\n\n', ...
                outputs.harp.position_x, outputs.harp.position_y);
            fprintf('  Groundspeed: %d kts\n', round(outputs.timing.groundspeed));
            
            
            fprintf('========================================\n');
        end