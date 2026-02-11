function ARoF_fps = computeAdjustedRoF(RoF_fps, dropTempC, surfaceTempC, midPresAlt_ft)
% computeAdjustedRoF - density correction for rate of fall.
% AFMAN expects MB-4 DENSITY ALTITUDE window usage. Here is a physics-based approx:
% Adjustment based on sqrt(rho_ref / rho_actual). We'll approximate rho via ISA lapse:
%
% NOTE: This is an approximation. For exact MB-4 results, replace this function with
% MB-4 table lookups or MAJCOM-approved interpolation.
%
% Convert midPresAlt_ft to meters
h_m = midPresAlt_ft * 0.3048;
% approximate temperature at mid altitude (linear interp between drop and surface temp)
% assume dropTempC is temp at drop altitude and surfaceTempC at surface; we use mid altitude fraction:
T_midC = dropTempC + (surfaceTempC - dropTempC) * 0.5;
T_midK = T_midC + 273.15;
% approximate pressure using barometric formula (std)
P0 = 101325;
g = 9.80665; Rair = 287.05; L = 0.0065;
T0 = 288.15;
if h_m < 11000
    P = P0 * (1 - L*h_m/T0)^(g/(Rair*L));
else
    P = P0 * exp(-g*h_m/(Rair*T_midK));
end
rho = P/(Rair*T_midK);
rho_ref = 1.225; % sea level standard
% scale RoF by sqrt(rho_ref / rho) since drag ~ rho*V^2; terminal velocity ~ 1/sqrt(rho)
ARoF_fps = RoF_fps * sqrt(rho_ref / rho);
end
