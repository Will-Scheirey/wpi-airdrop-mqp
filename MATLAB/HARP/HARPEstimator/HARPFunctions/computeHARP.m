% COMPUTEHARP Main HARP computation pipeline following AFMAN 11-231 Chapter 5.
%   Orchestrates the full sequence of HARP calculations by calling each
%   sub-function in order and assembling all intermediate and final results
%   into a single outputs struct. This is the top-level computational
%   function for the HARP estimator.
%
%   Computation sections:
%     1. Pressure and true altitudes        (computeAltitudes)
%     2. Adjusted rates of fall             (computeAdjustedRatesOfFall)
%     3. Ballistic winds                    (computeBallisticWinds)
%     4. High-velocity vector (HALO only)   (computeHighVelocityVector)
%     5. Deployed vector                    (computeDeployedVector)
%     6. Forward travel distance            (computeForwardTravelDistance)
%     7. Total wind effect and HARP position(computeHARPPosition)
%     8. Launch Acceptability Region        (computeLAR)
%     9. Timing information                 (computeTiming)
%
% INPUTS:
%   inputs : HARP inputs struct from convertDataOutToInputs, containing all
%            mission, parachute, altitude, temperature, wind, aircraft,
%            safety, and DZ configuration fields
%
% OUTPUTS:
%   outputs : Struct containing all intermediate and final results:
%               - inputs         : Original inputs struct (passed through)
%               - constants      : Physical/conversion constants struct
%               - altitudes      : Altitude computation results
%               - arof           : Adjusted rates of fall results
%               - winds          : Ballistic wind vectors
%               - hvVector       : High-velocity drift vector (HALO) or zeros
%               - deployedVector : Deployed canopy drift vector
%               - ftd            : Forward travel distance results
%               - harp           : HARP position and distance from PI
%               - lar            : Launch Acceptability Region parameters
%               - timing         : Green/red light timing results
%   inputs  : Inputs struct, returned unchanged (allows callers to use
%             updated handle if needed)
%
% See also CONVERTDATAOUTTOINPUTS, COMPUTEALTITUDES, COMPUTEADJUSTEDRATESOFFALL,
%          COMPUTEBALLISTICWINDS, COMPUTEHIGHVELOCITYVECTOR, COMPUTEDEPLOYEDVECTOR,
%          COMPUTEFORWARDTRAVELDISTANCE, COMPUTEHARPPOSITION, COMPUTELAR, COMPUTETIMING

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