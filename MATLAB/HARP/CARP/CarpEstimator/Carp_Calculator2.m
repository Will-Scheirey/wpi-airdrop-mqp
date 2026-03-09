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

