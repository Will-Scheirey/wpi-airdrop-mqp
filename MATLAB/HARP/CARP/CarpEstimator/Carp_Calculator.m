% CARP_CALCULATOR Compute CARP solution from a parameter struct (legacy version).
%   Calculates altitude, airspeed, time-of-fall, drift effect, and forward
%   travel distance using hard-coded parachute ballistics and air density
%   values. This is the original, simplified CARP calculator and does not
%   use a full atmospheric model — see Carp_Calculator2 for the updated
%   ISA-based version.
%
%   Several ballistics constants (RoF, VD, TFC, etc.) are hard-coded
%   inside this function rather than read from the parachute database.
%   Air density values (p_o, p) are also hard-coded and do not vary with
%   altitude.
%
% INPUTS:
%   params : Struct containing mission parameters, requiring:
%              - drop_altitude            : Drop altitude AGL (ft)
%              - terrain_elevation        : DZ terrain elevation MSL (ft)
%              - dz_altimeter             : DZ altimeter setting (inHg)
%              - true_altitude_temperature: Temperature at drop altitude (°C)
%              - indicated_airspeed       : Aircraft IAS (kts)
%              - surface_temperature      : Surface temperature at DZ (°C)
%              - ballistic_wind           : Ballistic wind speed (kts)
%              - drop_altitude_wind       : Wind speed at drop altitude (kts)
%              - DZ_course                : DZ centerline course (°)
%
% OUTPUTS:
%   carp : Struct containing computed CARP values:
%            - terrain_elevation  : Terrain elevation, passed through (ft)
%            - true_alt           : True altitude = drop_altitude + terrain_elevation (ft)
%            - pav                : Pressure altitude variation (ft)
%            - pressure_alt       : Pressure altitude (ft)
%            - corrected_drop_alt : Drop altitude minus 5 ft correction (ft)
%            - indicated_alt      : Indicated altitude (ft)
%            - alt_above_PI       : Altitude above Point of Impact (ft)
%            - stab_altitude      : Stabilization altitude (ft)
%            - cas                : Calibrated airspeed (kts)
%            - eas                : Equivalent airspeed (kts)
%            - tas                : True airspeed (kts)
%            - gs                 : Groundspeed (kts)
%            - adj_rof            : Density-corrected rate of fall (ft/s)
%            - tof                : Time of fall (s)
%            - total_tof          : Total time of fall including TFC (s)
%            - ftt                : Forward travel time (s)
%            - ftd                : Forward travel distance (yards)
%            - drift_eff          : Drift effect (yards)
%            - dz_heading         : DZ heading with drift correction (°)
%            - params             : Original input params struct
%            - RoF, PI, VD, TFC  : Hard-coded ballistics constants used
%
% NOTES:
%   - PI (Point of Impact) is hard-coded to 0 ft AGL and VD to 0 ft,
%     meaning stabilization altitude equals altitude above PI.
%   - wind_speed is hard-coded to 15 ft/s regardless of drop_altitude_wind.
%   - drift_correction is hard-coded to 0.
%   - The corrected_drop_alt applies a fixed 5 ft subtraction with no
%     physical justification documented.
%   - For a more physically accurate computation, use Carp_Calculator2.
%
% See also CARP_CALCULATOR2, CARP_ESTIMATOR, PRESSURE_ALTITUDE_VARIATION

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
