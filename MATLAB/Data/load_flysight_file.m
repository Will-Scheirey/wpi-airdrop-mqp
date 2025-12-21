function data_out = load_flysight_file(filename)
% LOAD_FLYSIGHT_FILE  Parse a FlySight-style log and return per-sensor tables.
%   data_out = load_flysight_file(filename)
%   Returns a struct where each field is a sensor type (e.g. 'IMU', 'GNSS') and
%   data_out.IMU  is a table with columns [time, field2, field3, ...].
%   For GNSS logs with ISO timestamps, the 'time' column is converted to
%   seconds from the first GNSS sample.

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

% Reference time for GNSS (first timestamp)
gnssBaseTime = datetime.empty;

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

            % -------- GNSS SPECIAL CASE: ISO-8601 TIME STRING ----------
            if strcmpi(type, 'GNSS')
                % parts{2} is something like '2025-08-07T13:20:02.000Z'
                timeStr = strtrim(parts{2});

                % Parse as UTC datetime
                dt = datetime(timeStr, ...
                    'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSS''Z''', ...
                    'TimeZone', 'UTC');

                % Set reference (first sample) if needed
                if isempty(gnssBaseTime)
                    gnssBaseTime = dt;
                end

                % Seconds since first GNSS sample
                tsec = seconds(dt - gnssBaseTime);

                % Allocate row: first column = time [s], rest numeric
                vals = nan(1, numel(parts)-1);
                vals(1) = tsec;

                % Remaining columns are numeric (lat, lon, hMSL, velN, ...)
                for k = 3:numel(parts)
                    v = str2double(parts{k});
                    if ~isnan(v)
                        vals(k-1) = v;
                    else
                        vals(k-1) = NaN;
                    end
                end

            else
                % -------- DEFAULT: all fields numeric ----------
                vals = nan(1, numel(parts)-1);
                for k = 2:numel(parts)
                    v = str2double(parts{k});
                    if ~isnan(v)
                        vals(k-1) = v;
                    else
                        vals(k-1) = NaN;
                    end
                end
            end

            % Append to dataMap
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
            if isempty(colnames)
                colnames = {'time'};
            end
        else
            colnames = {'time'};
        end
        safeNames = matlab.lang.makeValidName(colnames);
        emptyMat = nan(0, numel(safeNames));
        data_out.(t) = array2table(emptyMat, 'VariableNames', safeNames);
        continue;
    end

    ncols = size(mat,2);

    % Get column names from $COL if available, else generate generic names
    if isKey(colsMap, t)
        rawNames = colsMap(t);
        if numel(rawNames) < ncols
            extra = arrayfun(@(k) sprintf('col%d', k), numel(rawNames)+1:ncols, 'UniformOutput', false);
            rawNames = [rawNames, extra];
        elseif numel(rawNames) > ncols
            rawNames = rawNames(1:ncols);
        end
    else
        rawNames = arrayfun(@(k) sprintf('col%d', k), 1:ncols, 'UniformOutput', false);
    end

    safeNames = matlab.lang.makeValidName(rawNames);
    T = array2table(mat, 'VariableNames', safeNames);

    % Ensure 'time' is the first column and named 'time'
    if ~any(strcmpi(safeNames{1}, 'time'))
        timeVar = T.(safeNames{1});
        T.(safeNames{1}) = [];
        T = addvars(T, timeVar, 'Before', 1, 'NewVariableNames', 'time');
    else
        if ~strcmp(safeNames{1}, 'time')
            T.Properties.VariableNames{1} = 'time';
        end
    end

    data_out.(t) = T;
end

end