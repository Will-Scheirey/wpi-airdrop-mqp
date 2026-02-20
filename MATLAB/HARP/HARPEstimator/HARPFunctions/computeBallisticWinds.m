function winds = computeBallisticWinds(inputs, altitudes)
            % Compute vectorial average of winds
            
            winds = struct();
            
            if isfield(inputs.winds, 'hvBallistic') && isfield(inputs.winds, 'deployedBallistic')
                % Pre-computed winds provided
                winds.hvBallistic = inputs.winds.hvBallistic;
                winds.deployedBallistic = inputs.winds.deployedBallistic;
                winds.dropAltitude = inputs.winds.dropAltitude;
            else
                % Compute from wind profile
                windProfile = inputs.winds.profile;
                
                % High Velocity Ballistic Wind
                if strcmp(inputs.mission.type, 'HALO')
                    hvWinds = extractWindsInRange(windProfile, ...
                        altitudes.absoluteActuation, altitudes.stabilization, 2000);
                    winds.hvBallistic = vectorialAverageWind(hvWinds);
                else
                    winds.hvBallistic = [0, 0];
                end
                
                % Deployed Ballistic Wind
                if strcmp(inputs.mission.type, 'HALO')
                    startAlt = altitudes.absoluteActuation;
                else
                    startAlt = altitudes.stabilization;
                end
                
                deployedWinds = extractWindsInRange(windProfile, 0, startAlt, 1000);
                winds.deployedBallistic = vectorialAverageWind(deployedWinds);
                
                % Drop altitude wind
                winds.dropAltitude = interpolateWind(windProfile, altitudes.dropAbsolute);
            end
        end