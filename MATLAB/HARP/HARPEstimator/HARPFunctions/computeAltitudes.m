function altitudes = computeAltitudes(inputs, constants)
            % Compute all altitude-related parameters (Items 1-11 on AF Form 4015)
            
            altitudes = struct();
            
            % Item 2: Pressure Altitude Variation (PAV)
            % Formula A: PAV = (29.92 - Altimeter Setting) × 1000
            altitudes.pav = (constants.standardPressure - inputs.altitude.dzAltimeter) * ...
                            constants.altimeterConversion;
            
            % Item 3: Drop Pressure Altitude
            altitudes.dropPressure = inputs.altitude.dropIndicatedTrue + altitudes.pav;
            
            % Item 5: Drop True Altitude
            if isfield(inputs.altitude, 'dValue')
                altitudes.dropTrue = altitudes.dropPressure + inputs.altitude.dValue;
            else
                altitudes.dropTrue = altitudes.dropPressure;
            end
            
            % Item 7: Drop Absolute Altitude
            altitudes.dropAbsolute = altitudes.dropTrue - inputs.altitude.piElevation;
            
            % Item 9: Stabilization Altitude
            altitudes.stabilization = altitudes.dropAbsolute - inputs.parachute.vd;
            
            % Item 10: PI Pressure Altitude
            altitudes.piPressure = inputs.altitude.piElevation + altitudes.pav;
            
            % For HALO: Compute actuation altitudes (Items 11a-11d)
            if strcmp(inputs.mission.type, 'HALO')
                % Item 11b: Actuation Indicated True Altitude
                altitudes.actuationIndicatedTrue = inputs.altitude.actuationAGL + ...
                                                   inputs.altitude.piElevation;
                
                % Item 11c: Actuation Pressure Altitude
                % tempCorrection = (inputs.temps.actuation + 273.15) / 288.15;
                altitudes.actuationPressure = altitudes.actuationIndicatedTrue + altitudes.pav;
                
                % Item 11d: Absolute Actuation Altitude
                altitudes.absoluteActuation = altitudes.actuationPressure + ...
                                             inputs.altitude.dValue - inputs.altitude.piElevation;
                
                % Item 12: High Velocity Fall Distance
                altitudes.hvFallDistance = altitudes.stabilization - altitudes.absoluteActuation;
                
                % Item 25: Full Deployment Altitude (FDA)
                altitudes.fullDeployment = inputs.altitude.actuationAGL - inputs.parachute.dd;
            else
                % HAHO: No freefall
                altitudes.actuationIndicatedTrue = 0;
                altitudes.actuationPressure = 0;
                altitudes.absoluteActuation = 0;
                altitudes.hvFallDistance = 0;
                altitudes.fullDeployment = altitudes.stabilization;
            end
            
            % Item 26: Deployed Drive Fall Distance
            altitudes.deployedDriveFallDistance = altitudes.fullDeployment - inputs.safety.factor;
        end