function avgWind = vectorialAverageWind(winds)
            % Compute vectorial average of winds
            
            dirRange = max(winds(:, 1)) - min(winds(:, 1));
            spdRange = max(winds(:, 2)) - min(winds(:, 2));
            
            if dirRange <= 90 && spdRange <= 15
                % Arithmetic average
                avgWind = mean(winds, 1);
            else
                % Vectorial average
                uSum = 0;
                vSum = 0;
                
                for i = 1:size(winds, 1)
                    dir = winds(i, 1);
                    spd = winds(i, 2);
                    
                    u = -spd * sind(dir);
                    v = -spd * cosd(dir);
                    
                    uSum = uSum + u;
                    vSum = vSum + v;
                end
                
                uAvg = uSum / size(winds, 1);
                vAvg = vSum / size(winds, 1);
                
                avgSpeed = sqrt(uAvg^2 + vAvg^2);
                avgDir = atan2d(-uAvg, -vAvg);
                
                if avgDir < 0
                    avgDir = avgDir + 360;
                end
                
                avgWind = [avgDir, avgSpeed];
            end
        end