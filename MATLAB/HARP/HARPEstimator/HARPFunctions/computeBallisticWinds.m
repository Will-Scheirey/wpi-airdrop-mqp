% COMPUTEBALLISTICWINDS Compute vectorial average ballistic winds for HARP.
%   Derives the high-velocity (freefall) and deployed (canopy) ballistic
%   wind vectors by either using pre-computed values from the inputs struct
%   or computing them from a full wind profile. Also extracts the wind at
%   drop altitude for timing computations.
%
%   For HAHO missions, the high-velocity ballistic wind is set to [0, 0]
%   since there is no freefall phase.
%
% INPUTS:
%   inputs    : HARP inputs struct. If inputs.winds contains pre-computed
%               fields (hvBallistic, deployedBallistic, dropAltitude), they
%               are used directly. Otherwise, inputs.winds.profile is
%               required:
%                 - winds.profile : Nx3 array of [altitude(ft), direction(°),
%                                   speed(kts)] wind profile data
%                 - mission.type  : 'HALO' or 'HAHO'
%   altitudes : Altitude struct from computeAltitudes, requiring:
%                 - absoluteActuation : Absolute actuation altitude (ft); HALO
%                 - stabilization     : Stabilization altitude (ft)
%                 - dropAbsolute      : Drop absolute altitude (ft)
%
% OUTPUTS:
%   winds : Struct containing:
%             - hvBallistic       : 1x2 [direction(°), speed(kts)] for HV phase
%             - deployedBallistic : 1x2 [direction(°), speed(kts)] for canopy phase
%             - dropAltitude      : 1x2 [direction(°), speed(kts)] at drop altitude

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