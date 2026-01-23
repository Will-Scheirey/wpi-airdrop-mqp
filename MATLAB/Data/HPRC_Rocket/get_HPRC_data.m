function [data_accel, data_gyro, data_mag, data_gps, data_baro] = get_HPRC_data()
t_start = 9.0e5;
t_end   = 1.01e6;

[data_accel, data_gyro, data_mag] = filter_imu_data("ICM20948.csv", t_start, t_end);
lla_table_aprs = get_aprs();

dt0 = datetime('15-Nov-2025 13:59:40.21', 'TimeZone', 'America/New_York');
num_days = round(days(datetime('now', 'TimeZone', 'America/New_York') - dt0));
gps_time_corr = lla_table_aprs.Time - num_days;
gps_time = posixtime(gps_time_corr) - posixtime(dt0);

to_remove = gps_time < 0;
gps_time = gps_time(~to_remove);

lla_table_aprs = lla_table_aprs(~to_remove, :);
lla_all = [lla_table_aprs.Lat_deg, lla_table_aprs.Lon_deg, lla_table_aprs.Alt_m];
ecef_data = lla2enu(lla_all, lla_all(1, :), 'flat');

data_gps = table(gps_time, ecef_data, 'VariableNames', {'time', 'data'});
data_baro_raw = get_baro_data("LPS22.csv", t_start, t_end, 34);
data_baro = table(data_baro_raw.time / 1e3, data_baro_raw.msl - data_baro_raw.msl(1), 'VariableNames', {'time', 'data'});
end

function [data_accel, data_gyro, data_mag] = filter_imu_data(filename, t_start, t_end)
data_imu = readtable(filename);
to_remove = (abs(data_imu.accelZ) + abs(data_imu.accelX) + abs(data_imu.accelY)) >= 17;
data_imu = data_imu(~to_remove, :);

idx_start = find(data_imu.time > t_start, 1);
idx_end = find(data_imu.time > t_end, 1);

data = data_imu(idx_start:idx_end, :);

data.time = data.time - data.time(1);

data_accel = table(data.time / 1e3, [data.accelX, data.accelY, data.accelZ] * 9.81, 'VariableNames', {'time', 'data'});
data_gyro = table(data.time / 1e3, [data.gyrX, data.gyrY, data.gyrZ], 'VariableNames', {'time', 'data'});
data_mag = table(data.time / 1e3, [data.magX, data.magY, data.magZ], 'VariableNames', {'time', 'data'});
end

function data = get_aprs()
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
timeVec.TimeZone = "America/New_York";
latVec  = latVec(valid);
lonVec  = lonVec(valid);
altFt   = altFt(valid);

% Optional: convert altitude to meters
altM = altFt * 0.3048;

% Build output table
data = table(timeVec, latVec, lonVec, altFt, altM, ...
    'VariableNames', {'Time', 'Lat_deg', 'Lon_deg', 'Alt_ft', 'Alt_m'});

end

function [data_baro, ground_avg] = get_baro_data(filename, t_start, t_end, zero_actual)
    data_baro = readtable(filename);

    idx_start = find(data_baro.time > t_start, 1);
    idx_end = find(data_baro.time > t_end, 1);

    msl = atmospalt(data_baro.pressure * 100);

    ground_avg = mean(msl(1:idx_start));

    data_baro = data_baro(idx_start:idx_end, :);
    data_baro.time = data_baro.time - data_baro.time(1);
    
    zero_offset = ground_avg - zero_actual;

    data_baro.msl = msl(idx_start:idx_end) - zero_offset;
end