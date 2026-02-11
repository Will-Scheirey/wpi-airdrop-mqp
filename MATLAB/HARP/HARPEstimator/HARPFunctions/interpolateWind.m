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