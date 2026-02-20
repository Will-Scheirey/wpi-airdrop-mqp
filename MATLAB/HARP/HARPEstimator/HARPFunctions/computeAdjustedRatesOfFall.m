function arof = computeAdjustedRatesOfFall(inputs, altitudes, ~)
            % Compute density-corrected rates of fall (Items 14-16, 28-30)
            
            arof = struct();
            
            if strcmp(inputs.mission.type, 'HALO')
                % High Velocity Adjusted Rate of Fall
                arof.hvAvgTemp = (inputs.temps.drop + inputs.temps.actuation) / 2;
                arof.hvMidPressure = altitudes.actuationPressure + (altitudes.hvFallDistance / 2);
                
                % Density altitude correction
                tempRatio = 288.15 / (arof.hvAvgTemp + 273.15);
                pressureRatio = arof.hvMidPressure / altitudes.dropPressure;
                arof.hvARoF = inputs.parachute.hvRoF * sqrt(tempRatio * (1 + pressureRatio * 0.1));
            else
                arof.hvAvgTemp = 0;
                arof.hvMidPressure = 0;
                arof.hvARoF = 0;
            end
            
            % Deployed Adjusted Rate of Fall
            arof.deployedAvgTemp = (inputs.temps.surface + inputs.temps.actuation) / 2;
            
            if strcmp(inputs.mission.type, 'HALO')
                arof.deployedMidPressure = ((altitudes.actuationPressure - inputs.parachute.dd) + ...
                                           altitudes.piPressure) / 2;
            else
                arof.deployedMidPressure = (altitudes.dropPressure + altitudes.piPressure) / 2;
            end
            
            tempRatio = 288.15 / (arof.deployedAvgTemp + 273.15);
            pressureRatio = arof.deployedMidPressure / altitudes.dropPressure;
            arof.deployedARoF = inputs.parachute.deployedRoF * sqrt(tempRatio * (1 + pressureRatio * 0.1));
end
