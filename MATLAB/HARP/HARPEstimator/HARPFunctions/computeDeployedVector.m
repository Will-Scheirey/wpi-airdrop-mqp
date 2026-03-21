% COMPUTEDEPLOYEDVECTOR Compute the deployed (under canopy) displacement vector.
%   Calculates the time of fall and lateral drift of the payload during the
%   canopy descent phase, from full deployment to ground. Also computes
%   the forward drive distance available during the drive phase.
%   Corresponds to Items 31-36 on AF Form 4015.
%
% INPUTS:
%   inputs    : HARP inputs struct, requiring:
%                 - parachute.forwardDrive : Canopy forward drive speed (kts)
%   altitudes : Altitude struct from computeAltitudes, requiring:
%                 - fullDeployment            : Full deployment altitude AGL (ft)
%                 - deployedDriveFallDistance : Drive phase fall distance (ft)
%   arof      : Adjusted rates of fall struct from computeAdjustedRatesOfFall:
%                 - deployedARoF : Adjusted deployed rate of fall (ft/s)
%   winds     : Ballistic winds struct from computeBallisticWinds:
%                 - deployedBallistic : 1x2 [direction(°), speed(kts)]
%   constants : Constants struct from defineConstants, requiring:
%                 - knotsToFPS : Knots to feet-per-second conversion factor
%
% OUTPUTS:
%   deployedVector : Struct containing:
%                      - totalTime    : Total deployed time of fall (s) [Item 31]
%                      - driveTime    : Time available for forward drive (s) [Item 32]
%                      - windEffect   : Lateral drift due to deployed wind (ft) [Item 34]
%                      - driveDistance: Forward drive distance during drive phase (ft) [Item 36]
%                      - direction    : Direction of deployed ballistic wind (°)

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
            deployedVector.windEffect = winds.deployedBallistic(2) * deployedVector.totalTime / ...
                                        constants.knotsToFPS;
            
            % Item 36: Deployed Drive Distance
            deployedVector.driveDistance = inputs.parachute.forwardDrive * deployedVector.driveTime / ...
                                          constants.knotsToFPS;
            
            deployedVector.direction = winds.deployedBallistic(1);
        end