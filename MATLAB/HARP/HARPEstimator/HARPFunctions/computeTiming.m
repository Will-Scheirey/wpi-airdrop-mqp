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
            timing.greenLightTime = lar.usableLength / (timing.groundspeed * constants.knotsToMetersPerSec);
            
            % Item 55: Red Light Time
            timing.redLightTime = harp.stopwatchTime + timing.greenLightTime;
            timing.stopwatchTime = harp.stopwatchTime;
        end