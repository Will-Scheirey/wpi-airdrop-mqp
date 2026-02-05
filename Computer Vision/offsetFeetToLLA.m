function [lat_new, lon_new] = offsetFeetToLLA(lat0_deg, lon0_deg, dNorth_ft, dEast_ft)
%OFFSETFEETTOLLA Convert local N/E offsets (feet) to new lat/lon in decimal degrees.
%
% Inputs:
%   lat0_deg   - reference latitude  (decimal degrees)
%   lon0_deg   - reference longitude (decimal degrees)
%   dNorth_ft  - offset north (+) / south (–) in feet
%   dEast_ft   - offset east  (+) / west  (–) in feet
%
% Outputs:
%   lat_new    - new latitude  (decimal degrees)
%   lon_new    - new longitude (decimal degrees)

    % Length of 1 degree latitude in feet (approx.)
    ft_per_deg_lat = 364000;

    % Length of 1 degree longitude in feet at the reference latitude
    ft_per_deg_lon = 364000 * cosd(lat0_deg);

    % Convert offsets to degree changes
    dLat_deg = dNorth_ft / ft_per_deg_lat;
    dLon_deg = dEast_ft  / ft_per_deg_lon;

    % Apply offsets
    lat_new = lat0_deg + dLat_deg;
    lon_new = lon0_deg + dLon_deg;
end