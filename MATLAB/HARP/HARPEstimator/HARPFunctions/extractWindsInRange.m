 function windsOut = extractWindsInRange(windProfile, minAlt, maxAlt, interval)
            % Extract winds at specified intervals within altitude range
            altitudes = minAlt:interval:maxAlt;
            windsOut = zeros(length(altitudes), 2);
            
            for i = 1:length(altitudes)
                windsOut(i, :) = interpolateWind(windProfile, altitudes(i));
            end
        end