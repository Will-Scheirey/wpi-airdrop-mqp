% CARP_CALCULATOR2 Compute CARP solution using an ISA-based atmospheric model.
%   Updated CARP calculator that derives actual air density at drop altitude
%   using the International Standard Atmosphere (ISA) lapse rate model,
%   rather than using hard-coded density values. Rate of fall is also
%   derived from flight data rather than a fixed ballistics table value.
%
%   Results are in a mix of imperial and metric units — see field notes below.
%
% INPUTS:
%   carp_data : Struct of flight and mission data, requiring:
%                 - altitude     : Drop altitude (ft)
%                 - airspeed     : Aircraft indicated airspeed (kts)
%                 - groundspeed  : Aircraft groundspeed (kts) — used for gs directly
%                 - heading      : Aircraft heading (°)
%                 - wind_speed   : Wind speed (kts)
%                 - time_of_fall : Measured time of fall for this drop (s)
%
% OUTPUTS:
%   carp : Struct containing:
%            - stab_altitude : Stabilization altitude = drop_altitude - VD (ft)
%            - cas           : Calibrated airspeed (kts)
%            - eas           : Equivalent airspeed (kts)
%            - tas           : True airspeed (kts)
%            - gs            : Groundspeed (kts)
%            - adj_rof       : Density-corrected rate of fall (ft/s)
%            - tof           : Time of fall from stabilization (s)
%            - total_tof     : Total time of fall including TFC (s)
%            - ftt           : Forward travel time (s)
%            - ftd           : Forward travel distance (yards)
%            - drift_eff     : Drift effect (yards)
%            - dz_heading    : DZ heading, negated from carp_data.heading (°)
%            - RoF           : Derived rate of fall = altitude / time_of_fall (ft/s)
%            - VD            : Vertical distance constant used (ft)
%            - TFC           : Time of fall constant used (s)
%
% NOTES:
%   - RoF is computed as altitude / time_of_fall — this gives average descent
%     rate over the full drop, not just the stabilized freefall segment.
%   - VD is hard-coded to 370 ft; this should match the parachute type in use.
%   - dz_heading = -carp_data.heading negates the heading; verify this sign
%     convention is correct for the coordinate system in use.
%   - The ISA model used: T = T_o + a*h, p = p_o*(T/T_o)^(-1 - g/(a*R))
%     where h is altitude in meters, T_o = 288.16 K, a = -0.0065 K/m.

function carp = Carp_Calculator2(carp_data)
% Extract parameters
drop_altitude = carp_data.altitude;
indicated_airspeed = carp_data.airspeed;
correctionFactor = 1.0;

% Air Density Parameters
p_o = 1.225;                       % standard air density (kg/m^3)
T_o = 288.16; %temp at altitude (k)
a = -0.0065; %temp gradient (k/m)
R = 287; %gas constant (J/kg*k)
g = 9.8; %gravity (m/s^2)
T= T_o +a*ft2m(carp_data.altitude);


p = p_o*(T/T_o)^(-1-(g/(a*R)));                          % actual air density at altitude (kg/m^3)

% Parachute Ballistics Data
RoF = carp_data.altitude/carp_data.time_of_fall;                           % ft/s rate of fall                           % ft AGL (Point of Impact)
VD = 370;                           % ft vertical distance
TFC = 5.6;                           % sec time of fall constant
exit_time = 4.2;                      % sec from parachute ballistics
DQ = 2.7;

ballistic_wind = carp_data.wind_speed;
wind_speed = carp_data.wind_speed;  % ft/s
DZ_course = -carp_data.heading;
drift_correction = 0;

% Altitude Calculations
carp.stab_altitude = drop_altitude - VD;

% Airspeed Calculations
carp.cas = indicated_airspeed * correctionFactor;
carp.eas = carp.cas * sqrt(p_o/p);
carp.tas = carp.eas * sqrt(p_o/p);
carp.gs = wind_speed + carp.tas;

% Time and Distance Calculations
carp.adj_rof = RoF * sqrt(p_o/p);
carp.tof = carp.stab_altitude / carp.adj_rof;
carp.total_tof = TFC + carp.tof;
carp.ftt = exit_time + DQ;
carp.ftd = (carp.ftt * carp.gs) / 1.78;  % yards
carp.drift_eff = (ballistic_wind * carp.total_tof) / 1.78;  % yards

% Navigation
carp.dz_heading = DZ_course + drift_correction;

% Store raw parameters for conversion
carp.RoF = RoF;
carp.VD = VD;
carp.TFC = TFC;

end

