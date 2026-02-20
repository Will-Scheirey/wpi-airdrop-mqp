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