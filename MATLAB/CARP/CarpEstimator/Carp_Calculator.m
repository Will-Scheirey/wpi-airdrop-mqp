function carp = Carp_Calculator(params)
    % Extract parameters
    drop_altitude = params.drop_altitude;
    terrain_elevation = params.terrain_elevation;
    dz_altimeter = params.dz_altimeter;
    true_altitude_temperature = params.true_altitude_temperature;
    indicated_airspeed = params.indicated_airspeed;
    correctionFactor = 1.0;
    surface_temperature = params.surface_temperature;
    
    % Air Density Parameters
p_o = 0.0765;                       % standard air density (lb/ft^3)
p = 0.0763;                          % actual air density at altitfude (lb/ft^3)

% Parachute Ballistics Data
RoF = 55;                           % ft/s rate of fall
PI = 0;                           % ft AGL (Point of Impact)
VD = 0;                           % ft vertical distance
TFC = 5.6;                           % sec time of fall constant
exit_time = 4.2;                      % sec from parachute ballistics
DQ = 2.7;
    
    ballistic_wind = params.ballistic_wind;
    drop_altitude_wind = params.drop_altitude_wind;
    wind_speed = 15;  % ft/s
    DZ_course = params.DZ_course;
    drift_correction = 0;
    
    % Altitude Calculations
    carp.terrain_elevation = terrain_elevation; 
    carp.true_alt = drop_altitude + terrain_elevation;
    carp.pav = pressure_altitude_variation(dz_altimeter, carp.true_alt);
    carp.pressure_alt = carp.true_alt + carp.pav;
    carp.corrected_drop_alt = drop_altitude - 5;
    carp.indicated_alt = carp.corrected_drop_alt + terrain_elevation;
    carp.alt_above_PI = carp.true_alt - PI;
    carp.stab_altitude = carp.alt_above_PI - VD;
    
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
    carp.params = params;
    carp.RoF = RoF;
    carp.PI = PI;
    carp.VD = VD;
    carp.TFC = TFC;
    
end
