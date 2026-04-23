% INPUTS Template script defining all input parameters for CARP computation.
%   This script declares and documents every input variable required by the
%   CARP pipeline. It is intended as a reference and starting point — values
%   marked with 'X' or 'x' are placeholders that must be filled in before
%   running any CARP calculations.
%
%   Variables are grouped by category:
%     - Altitude parameters
%     - Air density parameters
%     - Parachute ballistics constants
%     - Wind and navigation parameters
%
% NOTES:
%   - Items 23, 24, 25, 31-35 from the standard CARP form are noted as
%     not applicable to this project.
%   - All placeholder values (X, x) must be replaced with real mission
%     data or parachute ballistics table values before use.
%

close all; clear all; clc; 

%%Inputs for the CARP Parameters

drop_altitude = 25000; %ft above ground level
terrain_elevation = 200; %ft elevation of highest point in DZ
dz_altimeter = 20; 
true_altitude_temperature = 20; %Celcius
indicated_airspeed = 130; %knots
p_o = 0.0765; %standard air density in lb/ft^3 (1.225 kg/m^3) 
p = 0.034; %actual air density (ex: at 25000ft p = 0.034lb/ft^3)
correctionFactor = 0.02; %correction factor for instrumentation of aircraft
RoF = 10; %ft/s rate of fall extracted from parachute balistics data
surface_temperature = 20; %Celcius temperature of at the DZ on the ground
PI = 200; %ft AGL (PI is usually equal to terrain elevation)
VD = 180; %ft veritcal distance (parachute balistics data)
TFC = X; %sec time of fall constant usually take from parachute balistics
ballistic_wind = X; %the expected wind which affects the load on its way to the ground
drop_altitude_wind = X; %expected wind at drop altitude
DZ_course = X; %deg DZ centerline course referenced in magnetic,true, or grid
drift_correction = X; %expected run-in heading correction necessary to parallel DZ course
exit_time = x; %sec obtained from parachute balistics data
DQ = x; %sec deceleration quotient obtained from parachute balistics data
wind_speed = x; %ft/s wind speed that effects airspeed



%%items that are not helpful for our project
%item23, item24, item 25, item31, item32, item33, item34, item35