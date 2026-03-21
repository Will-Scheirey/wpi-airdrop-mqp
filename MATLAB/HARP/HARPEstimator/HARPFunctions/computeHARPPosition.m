% COMPUTEHARPPOSITION Compute HARP position and bearing relative to the PI.
%   Combines the high-velocity drift effect, deployed wind effect, and
%   aircraft forward travel distance into a single (x, y) offset vector
%   from the Point of Impact (PI). Also computes the stopwatch distance
%   and time for aircrew use. Corresponds to Items 37-49 on AF Form 4015.
%
%   All vector directions are rotated by (direction + 180 - 45) before
%   decomposition into East/North components. The -45° rotation accounts
%   for the map grid convention used on AF Form 4015.
%
% INPUTS:
%   hvVector       : High-velocity vector struct from computeHighVelocityVector:
%                      - de        : HV drift effect (ft); 0 for HAHO
%                      - direction : HV wind direction (°)
%   deployedVector : Deployed vector struct from computeDeployedVector:
%                      - windEffect   : Deployed wind drift (ft)
%                      - driveDistance: Forward drive distance (ft)
%                      - direction    : Deployed wind direction (°)
%   ftd            : Forward travel struct from computeForwardTravelDistance:
%                      - distance    : Forward travel distance (ft)
%                      - groundspeed : Aircraft groundspeed (kts)
%   inputs         : HARP inputs struct, requiring:
%                      - aircraft.magneticCourse : Aircraft magnetic course (°)
%
% OUTPUTS:
%   harp : Struct containing:
%            - totalWindEffect   : Sum of HV and deployed drift effects (ft) [Item 37]
%            - position_x        : HARP East offset from PI (ft)
%            - position_y        : HARP North offset from PI (ft)
%            - distance          : HARP distance from PI (ft)
%            - bearing           : HARP bearing from PI (°, 0-360)
%            - stopwatchDistance : Distance for stopwatch computation (ft) [Item 48]
%            - stopwatchTime     : Stopwatch time for aircrew (s) [Item 49]
%
% NOTES:
%   - The stopwatchTime formula divides by (groundspeed * 1.94), which
%     converts kts to meters/sec. If position_x/y are in feet this may
%     produce inconsistent units — worth verifying against AFMAN 11-231.
%   - Direction wrapping is handled manually (mod 360 via subtraction).

function harp = computeHARPPosition(hvVector, deployedVector, ftd, inputs)
            % Compute HARP position relative to PI
            
            harp = struct();
            
            % Item 37: Total Wind Effect
            harp.totalWindEffect = hvVector.de + deployedVector.windEffect;
            
            % Deployed wind effect vector (upwind from PI)
            dweDir = deployedVector.direction + 180 - 45;
            if dweDir >= 360
                dweDir = dweDir - 360;
            end
            dwe_x = deployedVector.windEffect * sind(dweDir);
            dwe_y = deployedVector.windEffect * cosd(dweDir);
            
            % High velocity drift effect vector
            if hvVector.de > 0
                hvdeDir = hvVector.direction + 180 - 45;
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
            ftdDir = inputs.aircraft.magneticCourse + 180 - 45;
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