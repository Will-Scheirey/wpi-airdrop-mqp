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