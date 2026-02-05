function data_out = load_flysight_file(filename)
% LOAD_FLYSIGHT_FILE_FAST  Faster FlySight parser for large logs.
%
% Speed tricks:
%   - Single pass (no line-count prepass)
%   - Progress based on file position (bytes), throttled
%   - Per-type chunk buffering (no [A;row] growth)
%
% Adds:
%   - For GNSS with ISO timestamps, adds datetime column: GNSS.datetime_utc (UTC)

inputFile = filename;
if ~isfile(inputFile)
    error('Input file "%s" not found. Current folder: %s', inputFile, pwd);
end

fileInfo = dir(inputFile);
totalBytes = fileInfo.bytes;

fid = fopen(inputFile, 'r', 'n', 'UTF-8');
if fid < 0
    error('Could not open %s', inputFile);
end

% ----- metadata / schema -----
colsMap          = containers.Map();   % TYPE -> column names
typeColCount     = containers.Map();   % TYPE -> nFields
typeHasIsoTime   = containers.Map();   % TYPE -> logical
typeGnssDualTime = containers.Map();   % TYPE -> logical
isoBaseTime      = datetime.empty;     % first ISO time encountered (UTC)

% ----- per-type buffers -----
bufMap   = containers.Map(); % TYPE -> cell array of numeric matrices
bufCount = containers.Map(); % TYPE -> count rows buffered in current chunk

CHUNK = 20000;

% progress throttling
t0 = tic;
lastPrint = 0;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), continue; end
    if isempty(line) || line(1) ~= '$', continue; end

    % Throttled progress (bytes-based), ~4 Hz max
    nowT = toc(t0);
    if nowT - lastPrint > 0.25
        pos = ftell(fid);
        pct = 100 * double(pos) / double(totalBytes);
        % fprintf('\rReading %s: %.1f%%', inputFile, pct);
        lastPrint = nowT;
    end

    % Find first comma fast (tag)
    c1 = find(line==',', 1, 'first');
    if isempty(c1), continue; end
    tag = line(2:c1-1);

    % Metadata lines
    if strcmpi(tag,'COL')
        parts = strsplit(line, ',');
        if numel(parts) >= 3
            type = strtrim(parts{2});
            colsMap(type) = strtrim(parts(3:end));
        end
        continue;
    elseif any(strcmpi(tag, {'UNIT','VAR','FLYS','DATA'}))
        continue;
    end

    type = tag;

    parts = strsplit(line, ',');
    nFields = numel(parts) - 1;
    if nFields <= 0, continue; end

    % First time seeing this type: infer schema
    if ~isKey(typeColCount, type)
        typeColCount(type) = nFields;

        hasIso = is_iso8601_utc(parts{2});
        typeHasIsoTime(type) = hasIso;

        isDual = false;
        if strcmpi(type,'GNSS') && hasIso && nFields >= 4
            latGuess = str2double(parts{3});
            tonGuess = str2double(parts{4});
            lonGuess = str2double(parts{5});
            if isfinite(latGuess) && abs(latGuess) <= 90 && ...
               isfinite(lonGuess) && abs(lonGuess) <= 180 && ...
               isfinite(tonGuess) && tonGuess >= 0
                isDual = true;
            end
        end
        typeGnssDualTime(type) = isDual;

        if ~isKey(colsMap, type) || (strcmpi(type,'GNSS') && isDual)
            colMeta = {};
            if isKey(colsMap, type), colMeta = colsMap(type); end
            colsMap(type) = default_colnames(type, nFields, hasIso, isDual, colMeta);
        end

        % init buffers
        bufMap(type) = {zeros(CHUNK, nFields)};
        bufCount(type) = 0;
    end

    nExpected = typeColCount(type);

    % Normalize width (pad/truncate)
    if nFields < nExpected
        parts(end+1 : 1+nExpected) = {''}; %#ok<AGROW>
    elseif nFields > nExpected
        parts = parts(1 : 1+nExpected);
    end

    hasIso = typeHasIsoTime(type);
    isDual = typeGnssDualTime(type);

    row = nan(1, nExpected);

    if hasIso
        dt = datetime(parts{2}, ...
            'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''', ...
            'TimeZone','UTC');
        if isempty(isoBaseTime), isoBaseTime = dt; end
        utc_sec = seconds(dt - isoBaseTime);

        if strcmpi(type,'GNSS') && isDual
            % input: [utc_iso, lat, t_on, lon, rest...]
            row(1) = str2double(parts{4});  % time = seconds since power-on
            row(2) = str2double(parts{3});  % lat
            row(3) = str2double(parts{5});  % lon

            outIdx = 4;
            for k = 6:numel(parts)
                v = str2double(parts{k});
                if ~isnan(v), row(outIdx) = v; end
                outIdx = outIdx + 1;
                if outIdx > nExpected-1, break; end
            end
            row(end) = utc_sec; % time_utc at end
        else
            row(1) = utc_sec;   % time (UTC seconds since first ISO)
            for k = 3:numel(parts)
                v = str2double(parts{k});
                if ~isnan(v), row(k-1) = v; end
            end
        end
    else
        for k = 2:numel(parts)
            v = str2double(parts{k});
            if ~isnan(v), row(k-1) = v; end
        end
    end

    % Append to chunk buffer
    chunks = bufMap(type);
    ci = numel(chunks);
    nInChunk = bufCount(type) + 1;

    if nInChunk > CHUNK
        chunks{end+1} = zeros(CHUNK, nExpected); %#ok<AGROW>
        ci = ci + 1;
        nInChunk = 1;
    end

    chunks{ci}(nInChunk,:) = row;
    bufCount(type) = nInChunk;
    bufMap(type) = chunks;
end

fclose(fid);

% ----- build tables -----
types = keys(bufMap);
data_out = struct();

for i = 1:numel(types)
    type = types{i};

    chunks = bufMap(type);
    nExpected = typeColCount(type);

    % Trim last chunk
    lastN = bufCount(type);
    if isempty(chunks)
        mat = nan(0, nExpected);
    else
        chunks{end} = chunks{end}(1:lastN, :);
        mat = vertcat(chunks{:});
    end

    rawNames = colsMap(type);
    if numel(rawNames) < size(mat,2)
        extra = arrayfun(@(k) sprintf('col%d', k), numel(rawNames)+1:size(mat,2), 'UniformOutput', false);
        rawNames = [rawNames, extra];
    elseif numel(rawNames) > size(mat,2)
        rawNames = rawNames(1:size(mat,2));
    end

    safeNames = matlab.lang.makeValidName(rawNames);
    safeNames = matlab.lang.makeUniqueStrings(safeNames, {}, namelengthmax);
    safeNames{1} = 'time';
    safeNames = matlab.lang.makeUniqueStrings(safeNames, {}, namelengthmax);

    T = array2table(mat, 'VariableNames', safeNames);

    % ------------------------------
    % Add GNSS datetime_utc column
    % ------------------------------
    if strcmpi(type,'GNSS') && isKey(typeHasIsoTime,'GNSS') && typeHasIsoTime('GNSS') && ~isempty(isoBaseTime)
        % Dual-time GNSS uses time_utc; normal ISO GNSS uses time
        if any(strcmpi(T.Properties.VariableNames, 'time_utc'))
            utcSec = T.time_utc;
            afterVar = 'time_utc';
        else
            utcSec = T.time;
            afterVar = 'time';
        end

        dtUtc = isoBaseTime + seconds(utcSec);
        dtUtc.TimeZone = 'UTC';

        newName = make_unique_name(T.Properties.VariableNames, 'datetime_utc');
        T = addvars(T, dtUtc, 'After', afterVar, 'NewVariableNames', newName);
    end

    data_out.(type) = T;
end

end

% -------------------- helpers --------------------
function tf = is_iso8601_utc(s)
s = strtrim(s);
tf = ~isempty(regexp(s, '^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$', 'once'));
end

function nm = make_unique_name(existing, base)
% Return a name not in "existing" by appending _1, _2, ...
existingLower = lower(string(existing));
base = char(base);
nm = base;
k = 1;
while any(existingLower == lower(string(nm)))
    nm = sprintf('%s_%d', base, k);
    k = k + 1;
end
end

function names = default_colnames(type, nFields, hasIso, isDualGnss, colMeta)
type = upper(string(type));

if type == "GNSS" && hasIso && isDualGnss
    names = cell(1, nFields);
    names{1} = 'time'; % seconds since power-on
    names{2} = 'lat';
    names{3} = 'lon';

    if ~isempty(colMeta) && numel(colMeta) >= 5
        rest = colMeta(5:end);
        for k = 1:min(numel(rest), nFields-4)
            names{3+k} = rest{k};
        end
        for k = 4:(nFields-1)
            if isempty(names{k})
                names{k} = sprintf('col%d', k);
            end
        end
    else
        defaults = {'hMSL','velN','velE','velD','hAcc','vAcc','sAcc','numSV'};
        idx = 4;
        for k = 1:numel(defaults)
            if idx <= nFields-1
                names{idx} = defaults{k};
                idx = idx + 1;
            end
        end
        for k = idx:(nFields-1)
            names{k} = sprintf('col%d', k);
        end
    end

    names{nFields} = 'time_utc';
    return;
end

if type == "GNSS"
    base = {'time','lat','lon','hMSL','velN','velE','velD','hAcc','vAcc','sAcc','numSV'};
    if hasIso, base{1} = 'time_utc'; end
    names = cell(1, nFields);
    for k = 1:min(nFields, numel(base))
        names{k} = base{k};
    end
    for k = (min(nFields, numel(base))+1):nFields
        names{k} = sprintf('col%d', k);
    end
    return;
end

if type == "IMU" && nFields == 8
    names = {'time','wx','wy','wz','ax','ay','az','temp'};
    return;
end

if type == "MAG" && nFields == 6
    names = {'time','x','y','z','temp','col6'};
    return;
end

if type == "BARO" && nFields == 7
    names = {'time','pressure','temperature','alt','vario','col6','col7'};
    return;
end

names = cell(1, nFields);
names{1} = 'time';
for k = 2:nFields
    names{k} = sprintf('col%d', k);
end
end