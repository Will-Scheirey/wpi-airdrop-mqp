function lar = computeLAR(inputs, deployedVector, ~)
            % Compute Launch Acceptability Region
            
            lar = struct();
            
            % Item 51: Adjusted LAR Radius
            lar.adjustedRadius = deployedVector.driveDistance * inputs.safety.percentage;
            lar.maxRadius = deployedVector.driveDistance;
            
            % Item 52: Offset
            if isfield(inputs.dz, 'offset')
                lar.offset = inputs.dz.offset;
            else
                lar.offset = 0;
            end
            
            % Item 53: Usable Green Light Length
            if lar.offset < lar.adjustedRadius
                lar.usableLength = 2 * sqrt(lar.adjustedRadius^2 - lar.offset^2);
            else
                lar.usableLength = 0;
                warning('Offset exceeds LAR radius!');
            end
            
            lar.safetyPercentage = inputs.safety.percentage;
        end