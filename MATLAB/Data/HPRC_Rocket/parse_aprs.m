clear; clc; close all

%% Parse APRS log into table of time, lat, lon, alt
% File name
fname = 'lift.txt';

% Read entire file as string array (requires R2019b+)
rawText  = fileread(fname);
lines    = string(splitlines(rawText));

% Indices of header lines that contain the timestamp (the "$ fm ..." lines)
msgIdx = find(contains(lines, "$ fm "));

n = numel(msgIdx);
timeVec = NaT(n,1);
latVec  = nan(n,1);
lonVec  = nan(n,1);
altFt   = nan(n,1);

% Regex for APRS position line, e.g.
% !4449.46N/07309.87W'/A=000118 ...
%  ddmm.mmN / dddmm.mmW
posPattern = "!([0-9]{2})([0-9]{2}\.[0-9]{2})([NS])\/" + ...
             "([0-9]{3})([0-9]{2}\.[0-9]{2})([EW])";

for k = 1:n
    headerLine = lines(msgIdx(k));

    % --- Time (first 8 chars "HH:MM:SS") ---
    tstr = extractBetween(headerLine, 1, 8);
    timeVec(k) = datetime(tstr, 'InputFormat', 'HH:mm:ss');

    % --- Position line is assumed to be the next line ---
    if msgIdx(k) + 1 <= numel(lines)
        posLine = strtrim(lines(msgIdx(k) + 1));
    else
        continue;  % no following line, skip
    end

    % Extract lat/lon tokens via regexp
    tokens = regexp(posLine, posPattern, 'tokens', 'once');

    if ~isempty(tokens)
        % tokens: {dd, mm.mm, N/S, ddd, mm.mm, E/W}
        latDeg = str2double(tokens{1});
        latMin = str2double(tokens{2});
        latHem = tokens{3};

        lonDeg = str2double(tokens{4});
        lonMin = str2double(tokens{5});
        lonHem = tokens{6};

        % Convert to decimal degrees
        lat = latDeg + latMin/60;
        lon = lonDeg + lonMin/60;

        if latHem == "S"
            lat = -lat;
        end
        if lonHem == "W"
            lon = -lon;
        end

        latVec(k) = lat;
        lonVec(k) = lon;
    end

    % --- Altitude in feet: /A=000118 ---
    altTok = regexp(posLine, '/A=([0-9]+)', 'tokens', 'once');
    if ~isempty(altTok)
        altFt(k) = str2double(altTok{1});
    end
end

% Remove any rows where we failed to parse lat/lon
valid = ~isnan(latVec) & ~isnan(lonVec);
timeVec = timeVec(valid);
latVec  = latVec(valid);
lonVec  = lonVec(valid);
altFt   = altFt(valid);

% Optional: convert altitude to meters
altM = altFt * 0.3048;

% Build output table
aprsTable = table(timeVec, latVec, lonVec, altFt, altM, ...
    'VariableNames', {'Time', 'Lat_deg', 'Lon_deg', 'Alt_ft', 'Alt_m'});

uif = uifigure;
g = geoglobe(uif);
geoplot3(g, aprsTable.Lat_deg, aprsTable.Lon_deg, aprsTable.Alt_ft, "b", "LineWidth", 2);