
% HARPCOMPUTER High-level interface for computing High Altitude Release Points.
%   This classdef provides a static entry point for the full HARP
%   computation pipeline. It accepts a raw data_out structure, converts it
%   to the internal inputs format, and returns the complete set of HARP
%   outputs.
%
% METHODS (Static):
%   compute - Convert data_out to inputs and run the HARP computation
%
% USAGE:
%   outputs = HARPComputer.compute(data_out);

classdef HARPComputer
    % HARPComputer - Complete system for computing High Altitude Release Points
    
    methods (Static)
        
        function outputs = compute(data_out)
            % Main computation function
            % Convert data_out to inputs structure, then compute HARP
            
            inputs = convertDataOutToInputs(data_out);
            outputs = computeHARP(inputs);
        end
    end
end


% outputs = HARPComputer.compute(data_out);