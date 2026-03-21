% COMPUTEFORWARDTRAVELDISTANCE Compute aircraft forward travel distance during drop sequence.
%   Calculates the horizontal distance the aircraft travels from the time
%   the first jumper/payload exits until the last one does. Accounts for
%   wind correction angle and derives the actual groundspeed along the
%   drop course. Corresponds to Items 46-47 on AF Form 4015.
%
% INPUTS:
%   inputs    : HARP inputs struct, requiring:
%                 - parachute.et              : Equipment time (s)
%                 - parachute.dq              : Door/queue delay time (s)
%                 - winds.profile             : Nx3 array of [altitude(ft),
%                                               direction(°), speed(kts)]
%                 - aircraft.magneticCourse   : Aircraft magnetic course (°)
%                 - aircraft.airspeed         : Aircraft true airspeed (kts)
%   constants : Constants struct from defineConstants, requiring:
%                 - knotsToFPS : Knots to feet-per-second conversion factor
%
% OUTPUTS:
%   ftd : Struct containing:
%           - ftt          : Forward travel time (s) [Item 46]
%           - wca          : Wind correction angle (°)
%           - heading      : Aircraft true heading corrected for wind (°)
%           - groundspeed  : Aircraft groundspeed along course (kts)
%           - distance     : Forward travel distance (ft) [Item 47]
%
% NOTES:
%   - Wind at drop altitude is taken from the last row of winds.profile,
%     which should correspond to the highest (drop) altitude entry.
%   - The headwind component is positive for a tailwind (adds to airspeed).

function ftd = computeForwardTravelDistance(inputs, constants)
            % Compute forward travel distance
            
            ftd = struct();
            
            % Item 46: Forward Travel Time
            ftd.ftt = inputs.parachute.et + inputs.parachute.dq;
            
            % Compute groundspeed
            windDir = inputs.winds.profile(end, 2);
            windSpd = inputs.winds.profile(end, 3);
            trueCourse = inputs.aircraft.magneticCourse;
            
            % Wind correction angle
            ftd.wca = asind(windSpd * sind(windDir - trueCourse) / inputs.aircraft.airspeed);
            ftd.heading = trueCourse + ftd.wca;
            
            % Groundspeed
            headwind = windSpd * cosd(windDir - ftd.heading);
            ftd.groundspeed = inputs.aircraft.airspeed + headwind;
            
            % Item 47: Forward Travel Distance
            ftd.distance = ftd.groundspeed * ftd.ftt / constants.knotsToFPS;
        end