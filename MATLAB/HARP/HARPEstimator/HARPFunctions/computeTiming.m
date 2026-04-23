% COMPUTETIMING Compute green light, red light, and stopwatch timing for aircrew.
%   Derives the aircraft groundspeed at drop altitude and calculates the
%   usable green light time and red light time based on the LAR length and
%   HARP stopwatch distance. Corresponds to Items 54-55 on AF Form 4015.
%
% INPUTS:
%   inputs    : HARP inputs struct, requiring:
%                 - aircraft.magneticCourse : Aircraft magnetic course (°)
%                 - aircraft.airspeed       : Aircraft true airspeed (kts)
%   harp      : HARP position struct from computeHARPPosition:
%                 - stopwatchTime : Stopwatch time to HARP (s)
%   lar       : LAR struct from computeLAR:
%                 - usableLength : Usable green light window length (ft)
%   winds     : Ballistic winds struct from computeBallisticWinds:
%                 - dropAltitude : 1x2 [direction(°), speed(kts)] at drop altitude
%   constants : Constants struct from defineConstants, requiring:
%                 - knotsToFPS : Knots to feet-per-second conversion factor
%
% OUTPUTS:
%   timing : Struct containing:
%              - groundspeed    : Aircraft groundspeed at drop altitude (kts)
%              - greenLightTime : Duration of usable green light window (s) [Item 54]
%              - redLightTime   : Time from start to end of green light window (s) [Item 55]
%              - stopwatchTime  : Stopwatch time to HARP (s), echoed from harp struct
%
% NOTES:
%   - Groundspeed is recomputed here independently of computeForwardTravelDistance
%     because timing uses the wind at drop altitude rather than the surface wind.
%   - redLightTime = stopwatchTime + greenLightTime, representing when the
%     last payload should exit.

function timing = computeTiming(inputs, harp, lar, winds, constants)
            % Compute timing information
            
            timing = struct();
            
            % Recompute groundspeed
            windDir = winds.dropAltitude(1);
            windSpd = winds.dropAltitude(2);
            trueCourse = inputs.aircraft.magneticCourse;
            
            wca = asind(windSpd * sind(windDir - trueCourse) / inputs.aircraft.airspeed);
            trueHeading = trueCourse + wca;
            headwind = windSpd * cosd(windDir - trueHeading);
            timing.groundspeed = inputs.aircraft.airspeed + headwind;
            
            % Item 54: Usable Green Light Time
            timing.greenLightTime = lar.usableLength / (timing.groundspeed * constants.knotsToFPS);
            
            % Item 55: Red Light Time
            timing.redLightTime = harp.stopwatchTime + timing.greenLightTime;
            timing.stopwatchTime = harp.stopwatchTime;
        end