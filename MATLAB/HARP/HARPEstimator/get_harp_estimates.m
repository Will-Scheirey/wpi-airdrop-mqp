function harp = get_harp_estimates(full_dir, verbose)
    
    if nargin < 2
        verbose = false;
    end
    data_out = get_flight_estimates(full_dir, verbose);
    
    carp_data = data_out.carp;
    
    %% Converts Data into Inputs for HARP Computer
    inputs = convertDataOutToInputs(data_out);
    [outputs, inputs] = computeHARP(inputs); 
    
    dynamic_model = HARP_Dynamic_Model('inputs', inputs, 'data_out', data_out, 'carp_data', carp_data);

    harp.data_out = data_out;
    harp.carp_data = carp_data;
    harp.inputs = inputs;
    harp.outputs = outputs;
    harp.dynamic_model = dynamic_model;
end