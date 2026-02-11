function deployedVector = computeDeployedVector(inputs, altitudes, arof, winds, constants)
            % Compute deployed (under canopy) vector
            
            deployedVector = struct();
            
            % Item 31: Total Deployed Time
            deployedVector.totalTime = (altitudes.fullDeployment * constants.feetToMeters) / ...
                                       (arof.deployedARoF * constants.feetToMeters);
            
            % Item 32: Deployed Drive Time
            deployedVector.driveTime = (altitudes.deployedDriveFallDistance * constants.feetToMeters) / ...
                                       (arof.deployedARoF * constants.feetToMeters);
            
            % Item 34: Deployed Wind Effect
            deployedVector.windEffect = winds.deployedBallistic(2) * deployedVector.totalTime * ...
                                        constants.knotsToMetersPerSec;
            
            % Item 36: Deployed Drive Distance
            deployedVector.driveDistance = inputs.parachute.forwardDrive * deployedVector.driveTime * ...
                                          constants.knotsToMetersPerSec;
            
            deployedVector.direction = winds.deployedBallistic(1);
        end