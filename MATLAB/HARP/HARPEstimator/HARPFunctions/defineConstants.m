% DEFINECONSTANTS Return physical and unit conversion constants for HARP.
%   Returns a struct of all constants used throughout the HARP computation
%   pipeline. Centralizing these values here avoids magic numbers scattered
%   across functions and makes future calibration changes straightforward.
%
% INPUTS:
%   None
%
% OUTPUTS:
%   constants : Struct containing:
%                 - knotsToMPS            : Knots to meters/second (0.51444)
%                 - knotsToYPS            : Knots to yards/second (0.56)
%                 - knotsToFPS            : Knots to feet/second (0.56 * 3 = 1.68)
%                 - knotsToMetersPerSec   : Formula conversion factor (1.94)
%                                          NOTE: this is the inverse of knotsToMPS
%                                          and is used in stopwatch time computation
%                 - metersToYards         : Meters to yards (1.0936)
%                 - metersToNM            : Meters to nautical miles (1/1852)
%                 - nmToMeters            : Nautical miles to meters (1852)
%                 - feetToMeters          : Feet to meters (0.3048)
%                 - metersToFeet          : Meters to feet (3.28084)
%                 - standardPressure      : Standard sea-level pressure (29.92 inHg)
%                 - altimeterConversion   : Feet per 0.01 inHg deviation (1000)
%                 - yardsConversionFactor : Knots to yards/sec per manual (1.78)
%
% NOTES:
%   - knotsToFPS = 0.56 * 3 = 1.68 ft/s per knot. The standard value is
%     1.68781 ft/s per knot, so this is a close approximation.
%   - knotsToMetersPerSec = 1.94 appears to be the inverse relationship
%     (m/s to knots is ~1.944), used specifically in the stopwatch time
%     formula in computeHARPPosition.
%   - yardsConversionFactor (1.78) is noted as sourced from the AFMAN manual
%     and differs slightly from knotsToYPS (0.56); the distinction between
%     these two yard conversions should be verified against AFMAN 11-231.

function constants = defineConstants()
            % Physical and conversion constants
            constants = struct();
            constants.knotsToMPS = 0.51444;           % knots to meters/second
            constants.knotsToYPS = 0.56;              % knots to yards/second
            constants.knotsToFPS = 0.56 * 3;
            constants.knotsToMetersPerSec = 1.94;     % Formula conversion (from manual)
            constants.metersToYards = 1.0936;         % meters to yards
            constants.metersToNM = 1/1852;            % meters to nautical miles
            constants.nmToMeters = 1852;              % nautical miles to meters
            constants.feetToMeters = 0.3048;          % feet to meters
            constants.metersToFeet = 3.28084;         % meters to feet
            constants.standardPressure = 29.92;       % inches Hg
            constants.altimeterConversion = 1000;     % 0.01" Hg = 10 feet
            constants.yardsConversionFactor = 1.78;   % knots to yards/sec (from manual)
        end