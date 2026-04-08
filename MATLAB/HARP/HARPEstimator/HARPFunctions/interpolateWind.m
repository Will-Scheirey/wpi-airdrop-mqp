% INTERPOLATEWIND Interpolate wind direction and speed at a given altitude.
%   Performs linear interpolation on a wind profile to return the wind
%   vector at a specified altitude. Altitudes below or above the profile
%   range are clamped to the nearest endpoint (no extrapolation).
%
% INPUTS:
%   windProfile : Nx3 array of [altitude(ft), direction(°), speed(kts)],
%                 sorted by altitude ascending
%   altitude    : Target altitude at which to interpolate (ft)
%
% OUTPUTS:
%   wind : 1x2 vector of [direction(°), speed(kts)] at the requested altitude
%
% NOTES:
%   - Wind direction interpolation is arithmetic (linear), not circular.
%     This can produce incorrect results near the 0°/360° wraparound
%     (e.g. interpolating between 350° and 10° yields ~180° instead of 0°).
%     For high-accuracy use, circular interpolation should be applied.
%   - Altitudes exactly at a profile endpoint return that endpoint's values
%     without interpolation.
%
% See also EXTRACTWINDSINRANGE, VECTORIALAVERAGEWIND

function wind = interpolateWind(windProfile, altitude)
            % Interpolate wind at specific altitude
            
            if altitude <= windProfile(1, 1)
                wind = windProfile(1, 2:3);
            elseif altitude >= windProfile(end, 1)
                wind = windProfile(end, 2:3);
            else
                direction = interp1(windProfile(:, 1), windProfile(:, 2), altitude);
                speed = interp1(windProfile(:, 1), windProfile(:, 3), altitude);
                wind = [direction, speed];
            end
        end