function drop_data = load_system_data(full_dir)
% LOAD_SYSTEM_DATA Loads system information for a HAARS drop
%   This is a function loads system data like parachute and payload
%   mass/size information
%
%   It would be good to check that the extracted information does actually
%   correspond to what we are calling it
%
% INPUTS:
%   full_dir : Directory of the HAARS data
%
% OUTPUTS:
%   drop_data : Struct of relevant system data
%       .system                  : The parachute system name
%       .payload_weight          : Weight of the payload
%           [ lbs ]
%       .total_weight            : Weight of the payload + chutes
%           [ lbs ]
%       .dz_alt                  : Drop zone altitude in MSL
%           [ ft ]
%       .drop_alt                : Drop altitude
%           [ ft ]
%       .planned_activation      : Planned main deploy altitude
%           [ ft ]
%       .logged_activation       : Logged main deploy altitude in AGL
%           [ ft ]
%       . planned_impact_lat_lon : Planned impact point in lat/lon
%           [ deg ]
%       .logged_impact_lat_lon   : Logged impact point in lat/lon
%           [ deg ]
%
% See also GET_DROP_NUM

    % Load the CSV with drop info
    filename = 'Airdrop_System.csv';

    opts = detectImportOptions(filename);

    opts = setvaropts(opts, "Date", "Type", "datetime");
    opts = setvaropts(opts, "Date", "InputFormat", "MM/dd/uuuu");

    % Loading the table gives warnings about column names, so temporarily
    % turn off warnings
    warning('off')
    system_data = readtable(filename, opts);
    warning('on')
    
    % Extract the drop number and the row for the drop
    drop_num = get_drop_num(full_dir);
    drop_data_row = system_data(system_data.DropNumber == drop_num, :);

    drop_data = struct( ...
        'system',             drop_data_row.System, ...
        'payload_weight',     drop_data_row.SusWeight_lbs_, ...
        'total_weight',       drop_data_row.RiggedWeight_lbs_, ...
        'dz_alt',             drop_data_row.DZAltitude_ftMSL_, ...
        'drop_alt',           drop_data_row.DropAltitude_ft_, ...
        'planned_activation', drop_data_row.PlannedActivationAlt_ftAGL_, ...
        'logged_activation',  drop_data_row.ActivationAltitude_ftAGL__Log, ...
        'planned_impact_lat_lon', [drop_data_row.PlannedImpactPointLatitude, drop_data_row.PlannedImpactPointLongitude], ...
        ... % Should check that AMPCalcReleasePointLatitude is impact, it
        ... % may not be exactly impact (but close enough?)
        'logged_impact_lat_lon',  [drop_data_row.AMPCalcReleasePointLatitude, drop_data_row.AMPCalcReleasePointLongitude] ...
        );
end