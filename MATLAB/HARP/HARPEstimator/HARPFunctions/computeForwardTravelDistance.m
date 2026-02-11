function ftd = computeForwardTravelDistance(inputs, altitudes, winds, constants)
            % Compute forward travel distance
            
            ftd = struct();
            
            % Item 46: Forward Travel Time
            ftd.ftt = inputs.parachute.et + inputs.parachute.dq;
            
            % Compute groundspeed
            windDir = winds.dropAltitude(1);
            windSpd = winds.dropAltitude(2);
            trueCourse = inputs.aircraft.magneticCourse;
            
            % Wind correction angle
            ftd.wca = asind(windSpd * sind(windDir - trueCourse) / inputs.aircraft.airspeed);
            ftd.heading = trueCourse + ftd.wca;
            
            % Groundspeed
            headwind = windSpd * cosd(windDir - ftd.heading);
            ftd.groundspeed = inputs.aircraft.airspeed + headwind;
            
            % Item 47: Forward Travel Distance
            ftd.distance = ftd.groundspeed * ftd.ftt * constants.knotsToMetersPerSec;
        end