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