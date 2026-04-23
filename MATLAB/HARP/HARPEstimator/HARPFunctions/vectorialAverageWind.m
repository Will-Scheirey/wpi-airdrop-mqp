% VECTORIALAVERAGEWIND Compute the vectorial or arithmetic average of wind samples.
%   Selects between arithmetic and vectorial averaging based on the
%   variability of the input wind samples. If the direction range is within
%   90° and the speed range is within 15 kts, arithmetic averaging is used.
%   Otherwise, winds are decomposed into u/v components, averaged, and
%   recomposed into direction and speed.
%
%   This mirrors the averaging method prescribed in AFMAN 11-231 for
%   computing ballistic winds from a profile.
%
% INPUTS:
%   winds : Mx2 array of [direction(°), speed(kts)] wind samples,
%           as produced by extractWindsInRange
%
% OUTPUTS:
%   avgWind : 1x2 vector of [direction(°), speed(kts)] representing the
%             average wind across all input samples
%
% NOTES:
%   - The arithmetic averaging branch uses MATLAB's mean() across rows,
%     which averages direction and speed independently. This is only
%     appropriate when wind variation is small (per the thresholds above).
%   - Vectorial decomposition uses the meteorological convention:
%       u = -speed * sin(direction)   (positive u = wind from west)
%       v = -speed * cos(direction)   (positive v = wind from south)
%   - The output direction is computed as atan2d(-uAvg, -vAvg), which
%     recovers the meteorological "from" direction correctly.
%   - Direction wrapping: output direction is shifted to [0, 360) if
%     atan2d returns a negative value.
%   - Direction range (dirRange) is computed as max - min, which may
%     underestimate the true range near the 0°/360° wraparound.

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