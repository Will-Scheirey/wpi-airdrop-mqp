function deployedVector = computeDeployedVector(inputs, altitudes, arof, winds, constants)
            % Compute deployed (under canopy) vector
            
            deployedVector = struct();
            
            % Item 31: Total Deployed Time
            deployedVector.totalTime = (altitudes.fullDeployment) / ...
                                       (arof.deployedARoF);
            
            % Item 32: Deployed Drive Time
            deployedVector.driveTime = (altitudes.deployedDriveFallDistance ) / ...
                                       (arof.deployedARoF);
            
            % Item 34: Deployed Wind Effect
            deployedVector.windEffect = winds.deployedBallistic(2) * deployedVector.totalTime * ...
                                        constants.knotsToFPS;
            
            % Item 36: Deployed Drive Distance
            deployedVector.driveDistance = inputs.parachute.forwardDrive * deployedVector.driveTime * ...
                                          constants.knotsToFPS;
            
            deployedVector.direction = winds.deployedBallistic(1);
        end