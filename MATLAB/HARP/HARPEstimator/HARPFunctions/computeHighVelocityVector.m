% COMPUTEHIGHVELOCITYVECTOR Compute the freefall (high-velocity) drift vector.
%   Calculates the lateral displacement of the payload during the freefall
%   phase for HALO missions, from drop altitude to parachute actuation.
%   Corresponds to Items 17-21 on AF Form 4015.
%
%   This function should only be called for HALO missions. For HAHO, the
%   caller (computeHARP) substitutes a zero-valued struct.
%
% INPUTS:
%   inputs    : HARP inputs struct, requiring:
%                 - parachute.tfc : Time of fall to canopy opening (s)
%   altitudes : Altitude struct from computeAltitudes, requiring:
%                 - hvFallDistance : High-velocity fall distance (ft)
%   arof      : Adjusted rates of fall struct from computeAdjustedRatesOfFall:
%                 - hvARoF : Adjusted high-velocity rate of fall (ft/s)
%   winds     : Ballistic winds struct from computeBallisticWinds:
%                 - hvBallistic : 1x2 [direction(°), speed(kts)]
%   constants : Constants struct from defineConstants, requiring:
%                 - knotsToFPS : Knots to feet-per-second conversion factor
%
% OUTPUTS:
%   hvVector : Struct containing:
%                - tof      : High-velocity time of fall (s) [Item 17]
%                - totalTof : Total HV time including canopy opening (s) [Item 19]
%                - de       : High-velocity drift effect distance (ft) [Item 21]
%                - direction: Direction of HV wind drift (°)

function hvVector = computeHighVelocityVector(inputs, altitudes, arof, winds, constants)
            % Compute high velocity (freefall) vector for HALO
            
            hvVector = struct();
            
            % Item 17: High Velocity Time of Fall
            hvVector.tof = altitudes.hvFallDistance / arof.hvARoF;
            
            % Item 19: High Velocity Total Time of Fall
            hvVector.totalTof = hvVector.tof + inputs.parachute.tfc;
            
            % Item 21: High Velocity Drift Effect
            hvVector.de = winds.hvBallistic(2) * hvVector.totalTof * constants.knotsToFPS;
            hvVector.direction = winds.hvBallistic(1);
        end