% COMPUTEADJUSTEDRATESOFFALL Compute density-corrected rates of fall.
%   Applies temperature and pressure corrections to the parachute's
%   tabulated rates of fall to account for non-standard atmospheric
%   conditions at the relevant altitudes. Corresponds to Items 14-16
%   and 28-30 of AF Form 4015.
%
%   For HALO missions, both a high-velocity (freefall) and a deployed
%   (canopy) adjusted rate of fall are computed. For HAHO missions,
%   only the deployed rate of fall is computed (hvARoF is set to zero).
%
%   The density correction uses the formula:
%     ARoF = tabulated_RoF * sqrt( (288.15 / (T_avg + 273.15)) *
%                                   (1 + P_mid / P_drop * 0.1) )
%   where T_avg is the mean temperature (°C) over the fall segment and
%   P_mid is the mid-segment pressure altitude (ft).
%
% INPUTS:
%   inputs    : HARP inputs struct, requiring:
%                 - mission.type          : 'HALO' or 'HAHO'
%                 - temps.drop            : Temperature at drop altitude (°C)
%                 - temps.actuation       : Temperature at actuation altitude (°C)
%                 - temps.surface         : Surface temperature (°C)
%                 - parachute.hvRoF       : Tabulated HV rate of fall (ft/s)
%                 - parachute.deployedRoF : Tabulated deployed rate of fall (ft/s)
%                 - parachute.dd          : Deployment distance (ft)
%   altitudes : Altitude struct from computeAltitudes, requiring:
%                 - dropPressure          : Drop pressure altitude (ft)
%                 - actuationPressure     : Actuation pressure altitude (ft)
%                 - hvFallDistance        : High-velocity fall distance (ft)
%                 - piPressure            : PI pressure altitude (ft)
%   ~         : Third argument (constants) is accepted but unused
%
% OUTPUTS:
%   arof : Struct containing:
%            - hvAvgTemp         : Mean temperature over HV fall (°C); 0 for HAHO
%            - hvMidPressure     : Mid-segment pressure altitude for HV fall (ft); 0 for HAHO
%            - hvARoF            : Adjusted HV rate of fall (ft/s); 0 for HAHO
%            - deployedAvgTemp   : Mean temperature over deployed fall (°C)
%            - deployedMidPressure : Mid-segment pressure altitude for deployed fall (ft)
%            - deployedARoF      : Adjusted deployed rate of fall (ft/s)

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
