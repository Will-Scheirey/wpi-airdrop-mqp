function data_out = load_flysight_file(filename)
% LOAD_FLYSIGHT_FILE  Parse a FlySight-style log and return per-sensor tables.
%   data_out = load_flysight_file(filename)
%   Returns a struct where each field is a sensor type (e.g. 'IMU') and
%   data_out.IMU is a table with columns [time, field2, field3, ...].
%
%   The function reads $COL and $UNIT metadata when present to name columns.

inputFile = filename;

if ~isfile(inputFile)
    error('Input file "%s" not found in current folder: %s', inputFile, pwd);
end

fid = fopen(inputFile,'r','n','UTF-8');
if fid < 0
    error('Could not open %s', inputFile);
end

% Metadata + data storage
colsMap  = containers.Map();  % TYPE -> column names
unitsMap = containers.Map();  % TYPE -> units (parsed but not used here)
dataMap  = struct();          % TYPE -> numeric matrix

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), continue; end
    line = strtrim(line);
    if isempty(line) || line(1) ~= '$', continue; end

    parts = strsplit(line, ',');
    tag = parts{1}(2:end);  % remove leading $

    switch upper(tag)
        case 'COL'
            if numel(parts) < 3, continue; end
            colsMap(parts{2}) = strtrim(parts(3:end));
        case 'UNIT'
            if numel(parts) < 3, continue; end
            unitsMap(parts{2}) = strtrim(parts(3:end));
        otherwise
            type = tag;
            vals = nan(1, numel(parts)-1);
            for k = 2:numel(parts)
                v = str2double(parts{k});
                if ~isnan(v)
                    vals(k-1) = v;
                else
                    % non-numeric fields -> NaN (most FlySight numeric)
                    vals(k-1) = NaN;
                end
            end
            if isfield(dataMap, type)
                dataMap.(type) = [dataMap.(type); vals];
            else
                dataMap.(type) = vals;
            end
    end
end

fclose(fid);

% Get types safely even if a variable named 'fieldnames' exists in workspace
types = builtin('fieldnames', dataMap);
if isempty(types)
    error('No sensor data found in %s', inputFile);
end

% Build output struct: for each type make a table with 'time' as first column
data_out = struct();
for i = 1:numel(types)
    t = types{i};
    mat = dataMap.(t);
    if isempty(mat)
        % empty: create empty table with no rows but with columns if possible
        if isKey(colsMap, t)
            colnames = colsMap(t);
            % ensure at least "time" column
            if isempty(colnames)
                colnames = {'time'};
            end
        else
            colnames = {'time'};
        end
        safeNames = matlab.lang.makeValidName(colnames);
        % build an empty table with those variable names
        emptyMat = nan(0, numel(safeNames));
        data_out.(t) = array2table(emptyMat, 'VariableNames', safeNames);
        continue;
    end

    ncols = size(mat,2);

    % Get column names from $COL if available, else generate generic names
    if isKey(colsMap, t)
        rawNames = colsMap(t);
        % ensure number of names matches columns
        if numel(rawNames) < ncols
            extra = arrayfun(@(k) sprintf('col%d', k), numel(rawNames)+1:ncols, 'UniformOutput', false);
            rawNames = [rawNames, extra];
        elseif numel(rawNames) > ncols
            rawNames = rawNames(1:ncols);
        end
    else
        rawNames = arrayfun(@(k) sprintf('col%d', k), 1:ncols, 'UniformOutput', false);
    end

    % Ensure first column is 'time' named consistently
    % If the first name isn't obviously a time label, still keep the provided name
    % but normalize variable names for MATLAB table.
    safeNames = matlab.lang.makeValidName(rawNames);

    % Convert numeric matrix to table
    T = array2table(mat, 'VariableNames', safeNames);

    % If the first column is not called 'time' (case-insensitive), still add a 'time' alias
    % so users can always reference T.time. We'll preserve original name and add time var.
    if ~any(strcmpi(safeNames{1}, 'time'))
        % move original first column to variable with its safe name, then add 'time' var
        timeVar = T.(safeNames{1});
        % remove original first variable
        T.(safeNames{1}) = [];
        % create new table with 'time' first
        T = addvars(T, timeVar, 'Before', 1, 'NewVariableNames', 'time');
    else
        % ensure 'time' uses the canonical lowercase name
        if ~strcmp(safeNames{1}, 'time')
            % rename the first var to 'time' preserving its data
            T.Properties.VariableNames{1} = 'time';
        end
    end

    % Assign resulting table into output struct
    data_out.(t) = T;
end

end