function [outputs, inputs] = computeHARP(inputs)
            % Main HARP computation following AFMAN 11-231 Chapter 5
            
            % Get constants
            constants = defineConstants();
            
            % Section 1: Calculate Pressure and True Altitudes
            altitudes = computeAltitudes(inputs, constants);
            
            % Section 2: Compute Adjusted Rates of Fall
            arof = computeAdjustedRatesOfFall(inputs, altitudes, constants);
            
            % Section 3: Compute Ballistic Winds
            winds = computeBallisticWinds(inputs, altitudes);
            
            % Section 4: Compute High Velocity Vector (HALO only)
            if strcmp(inputs.mission.type, 'HALO')
                hvVector = computeHighVelocityVector(inputs, altitudes, arof, winds, constants);
            else
                hvVector = struct('de', 0, 'tof', 0, 'totalTof', 0, 'direction', 0);
            end
            
            % Section 5: Compute Deployed Vector
            deployedVector = computeDeployedVector(inputs, altitudes, arof, winds, constants);
            
            % Section 6: Compute Forward Travel Distance
            ftd = computeForwardTravelDistance(inputs, constants);
            
            % Section 7: Compute Total Wind Effect and HARP
            harp = computeHARPPosition(hvVector, deployedVector, ftd, inputs);
            
            % Section 8: Compute Launch Acceptability Region
            lar = computeLAR(inputs, deployedVector, constants);
            
            % Section 9: Compute Timing Information
            timing = computeTiming(inputs, harp, lar, winds, constants);
            
            % Assemble outputs
            outputs = struct();
            outputs.inputs = inputs;
            outputs.constants = constants;
            outputs.altitudes = altitudes;
            outputs.arof = arof;
            outputs.winds = winds;
            outputs.hvVector = hvVector;
            outputs.deployedVector = deployedVector;
            outputs.ftd = ftd;
            outputs.harp = harp;
            outputs.lar = lar;
            outputs.timing = timing;
        end