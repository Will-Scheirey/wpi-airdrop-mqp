clear; clc;
addpath(genpath("MATLAB"));
addpath(genpath("weather"));
addpath(genpath("haars_data"));

parent_dir = "haars_data";

% Get all drop subdirectories from parent_dir
all_entries = dir(parent_dir);
is_dir = [all_entries.isdir];
all_dirs = all_entries(is_dir);
all_dirs = all_dirs(~ismember({all_dirs.name}, {'.', '..'}));  % <-- fixes issue #2


results_all = cell(length(all_dirs), 1);

parfor i = 1:length(all_dirs)
    drop_dir = all_dirs(i).name;
    full_dir = fullfile(parent_dir, drop_dir);

    fprintf('\n--- Processing (%d/%d): %s ---\n', i, length(all_dirs), drop_dir);

    results = struct();
    try
        sensor_size = dir(fullfile(full_dir, "SENSOR.csv")).bytes;

        if sensor_size > 100e6
            error("Filesize too big!")
        end

        harp_estimates = get_harp_estimates(full_dir);

        data_out     = get_flight_estimates(full_dir);          % <-- inside loop
        carp_data    = data_out.carp;
        inputs       = convertDataOutToInputs(data_out);
        [outputs, inputs] = computeHARP(inputs);
        dynamic_model = HARP_Dynamic_Model('inputs', inputs, 'data_out', data_out, ...
                                           'carp_data', carp_data);

        field_name = matlab.lang.makeValidName(drop_dir);
        results.drop_dir      = drop_dir;
        results.data_out      = harp_estimates.data_out;
        results.carp_data     = harp_estimates.carp_data;
        results.inputs        = harp_estimates.inputs;
        results.outputs       = harp_estimates.outputs;
        results.dynamic_model = harp_estimates.dynamic_model;
        results.status        = 'success';

    catch ME
        fprintf('ERROR on %s: %s\n', drop_dir, ME.message);
        field_name = matlab.lang.makeValidName(drop_dir);
        results.drop_dir = drop_dir;
        results.status   = 'failed';
        results.error    = ME.message;
    end
    results_all{i} = results;
    fprintf('\n--- Processing (%d/%d): %s ---\n', i, length(all_dirs), drop_dir);
    fprintf("\tSTATUS: %s\n", results.status);
end

save_path = fullfile(parent_dir, 'all_drops_results.mat');
save(save_path, 'results_all', '-v7.3');
