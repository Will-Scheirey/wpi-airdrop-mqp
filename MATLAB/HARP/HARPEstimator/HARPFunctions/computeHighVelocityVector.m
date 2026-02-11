function hvVector = computeHighVelocityVector(inputs, altitudes, arof, winds, constants)
            % Compute high velocity (freefall) vector for HALO
            
            hvVector = struct();
            
            % Item 17: High Velocity Time of Fall
            hvVector.tof = altitudes.hvFallDistance / arof.hvARoF;
            
            % Item 19: High Velocity Total Time of Fall
            hvVector.totalTof = hvVector.tof + inputs.parachute.tfc;
            
            % Item 21: High Velocity Drift Effect
            hvVector.de = winds.hvBallistic(2) * hvVector.totalTof * constants.knotsToMetersPerSec;
            hvVector.direction = winds.hvBallistic(1);
        end