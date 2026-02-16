function harp = computeHARPPosition(hvVector, deployedVector, ftd, inputs)
            % Compute HARP position relative to PI
            
            harp = struct();
            
            % Item 37: Total Wind Effect
            harp.totalWindEffect = hvVector.de + deployedVector.windEffect;
            
            % Deployed wind effect vector (upwind from PI)
            dweDir = deployedVector.direction + 180;
            if dweDir >= 360
                dweDir = dweDir - 360;
            end
            dwe_x = deployedVector.windEffect * sind(dweDir);
            dwe_y = deployedVector.windEffect * cosd(dweDir);
            
            % High velocity drift effect vector
            if hvVector.de > 0
                hvdeDir = hvVector.direction + 180;
                if hvdeDir >= 360
                    hvdeDir = hvdeDir - 360;
                end
                hvde_x = hvVector.de * sind(hvdeDir);
                hvde_y = hvVector.de * cosd(hvdeDir);
            else
                hvde_x = 0;
                hvde_y = 0;
            end
            
            % Forward travel distance
            ftdDir = inputs.aircraft.magneticCourse + 180;
            if ftdDir >= 360
                ftdDir = ftdDir - 360;
            end
            ftd_x = ftd.distance * sind(ftdDir);
            ftd_y = ftd.distance * cosd(ftdDir);
            
            % HARP position
            harp.position_x = dwe_x + hvde_x + ftd_x;
            harp.position_y = dwe_y + hvde_y + ftd_y;
            
            harp.distance = sqrt(harp.position_x^2 + harp.position_y^2);
            harp.bearing = atan2d(harp.position_x, harp.position_y);
            if harp.bearing < 0
                harp.bearing = harp.bearing + 360;
            end
            
            % Item 48-49: Stopwatch Distance and Time
            harp.stopwatchDistance = harp.distance;
            harp.stopwatchTime = harp.stopwatchDistance / (ftd.groundspeed * 1.94);
        end