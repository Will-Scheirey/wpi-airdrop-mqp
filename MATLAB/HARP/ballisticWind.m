function w = ballisticWind(winds_table, alt_top_ft, alt_bottom_ft, phase)
% ballisticWind - compute mean ballistic wind vector (speed_kt, dir_deg_true)
% winds_table: Nx3 [alt_ft, dir_deg_true, speed_kt], altitudes descending or ascending
% alt_top_ft: top altitude (e.g., stabilization altitude)
% alt_bottom_ft: bottom altitude (e.g., actuation altitude or surface)
% phase: 'HV' or 'Deployed' or 'Total' (for info)
%
% This function returns a vectorial mean (x,y) average of winds between the two altitudes,
% converting directions to u/v components and averaging by altitude weighting.
%
if isempty(winds_table)
    w.speed_kt = NaN;
    w.dir_deg = NaN;
    return;
end

% ensure winds table is sorted by altitude descending (top to bottom)
winds = sortrows(winds_table, -1);

% Limit integration between alt_top and alt_bottom by interpolating speeds and directions
alts = winds(:,1);
dirs = winds(:,2);
spds = winds(:,3);

% create a fine altitude grid between top and bottom at 1000 ft steps (MB-4 suggests every 1k-2k ft)
Ngrid = max(2, ceil(abs(alt_top_ft - alt_bottom_ft)/1000)+1);
alt_grid = linspace(alt_top_ft, alt_bottom_ft, Ngrid);

u = zeros(size(alt_grid));
v = zeros(size(alt_grid));
for i=1:length(alt_grid)
    a = alt_grid(i);
    % interpolate dir and speed to altitude a
    sp = interp1(alts, spds, a, 'linear', 'extrap');
    dr = interp1(alts, dirs, a, 'linear', 'extrap');
    % convert to u,v (true bearing -> math angle: 0deg north, clockwise)
    theta = deg2rad(90 - dr); % convert meteorological to math
    u(i) = sp * cos(theta);
    v(i) = sp * sin(theta);
end

% average u,v
u_mean = mean(u);
v_mean = mean(v);

% convert back to speed & direction
speed_mean = sqrt(u_mean^2 + v_mean^2);
dir_math = atan2(v_mean, u_mean);
dir_deg = 90 - rad2deg(dir_math);
dir_deg = mod(dir_deg,360);

w.speed_kt = speed_mean;
w.dir_deg = dir_deg;
w.u = u_mean;
w.v = v_mean;
end
