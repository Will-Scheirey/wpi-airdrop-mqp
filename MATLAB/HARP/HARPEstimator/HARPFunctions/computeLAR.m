% COMPUTELAR Compute the Launch Acceptability Region (LAR) parameters.
%   Calculates the adjusted LAR radius, any lateral offset of the drop zone
%   centerline from the PI, and the resulting usable green light length
%   (the chord length through the LAR circle at the offset distance).
%   Corresponds to Items 51-53 on AF Form 4015.
%
% INPUTS:
%   inputs         : HARP inputs struct, requiring:
%                      - safety.percentage : LAR safety factor (0-1, e.g. 0.80)
%                      - dz.offset         : Lateral offset of DZ from PI (ft);
%                                            optional, defaults to 0
%   deployedVector : Deployed vector struct from computeDeployedVector:
%                      - driveDistance : Maximum forward drive distance (ft)
%   ~              : Third argument (constants) is accepted but unused
%
% OUTPUTS:
%   lar : Struct containing:
%           - adjustedRadius   : LAR radius after safety factor (ft) [Item 51]
%           - maxRadius        : Full drive distance without safety factor (ft)
%           - offset           : Lateral DZ offset from PI (ft) [Item 52]
%           - usableLength     : Green light window length (ft) [Item 53]
%           - safetyPercentage : Safety percentage applied (echoed from inputs)
%
% NOTES:
%   - If offset >= adjustedRadius, the aircraft track misses the LAR entirely
%     and usableLength is set to 0 with a warning issued.
%   - usableLength is the chord of the LAR circle: 2 * sqrt(r² - d²),
%     where r = adjustedRadius and d = offset.

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