function [weather_data, the_weather] = load_weather(the_time)
weather_dir = 'weather';

files = dir(weather_dir);

day_dir = [];

for i = 1:length(files)
    if contains(files(i).name, '.')
        continue
    end
    dir_date = datetime(files(i).name, 'TimeZone', 'UTC');

    if abs(days(dir_date - the_time)) < 1
        day_dir = files(i);
        break
    end
end

if isempty(dir_date)
    error("There is no weather data for the specified date!")
end

day_dir = fullfile(weather_dir, day_dir.name);

day_data_dir = fullfile(day_dir, 'data');

if ~isfolder(day_data_dir)
    error("There is no data file for the specified date!")
end

day_files = dir(day_data_dir);

radar_dir = fullfile(day_data_dir, 'radar');

if ~isfolder(radar_dir)
    error("There is no radar data file for the specified date!")
end

radar_files = dir(fullfile(radar_dir, '*.csv'));

weather_files = {};
dates = {};
idx = 1;

for i = 1:length(radar_files)
    file = radar_files(i);
    filename = file.name;

    [~, file_name, ~] = fileparts(filename);

    file_time = datetime(file_name, ...
        'InputFormat','MMddHHmm''Z''', ...
        'TimeZone','UTC');
    file_time.Year = the_time.Year;

    diff = abs(hours(the_time - file_time));

    if diff < 1
        weather_files{idx} = file;
        dates{idx} = file_time;
        idx = idx + 1;
    end
end

if isempty(weather_files)
    error("No radar date for the given time of day!")
end

for i = 1:length(weather_files)
    file = weather_files{i};
    filename = file.name;

    [~, file_name, ~] = fileparts(filename);

    file_time = datetime(file_name, ...
        'InputFormat','MMddHHmm''Z''', ...
        'TimeZone','UTC');
    file_time.Year = the_time.Year;

    weather_data{i} = struct('time', file_time, 'data', readtable(filename));
end
% --- Combine (if we have at least 2 files) ---
if numel(weather_data) >= 2
    t1 = weather_data{1}.time;
    t2 = weather_data{2}.time;

    d1 = abs(hours(the_time - t1));
    d2 = abs(hours(the_time - t2));

    % Inverse-distance weights (handle exact match)
    if d1 == 0
        w1 = 1; w2 = 0;
    elseif d2 == 0
        w1 = 0; w2 = 1;
    else
        w1 = 1/d1;
        w2 = 1/d2;
        s = w1 + w2;
        w1 = w1/s;  w2 = w2/s;
    end

    T1 = weather_data{1}.data;
    T2 = weather_data{2}.data;

    % Combine only numeric variables, keep non-numeric from T1
    v1 = T1.Variables;
    v2 = T2.Variables;

    numMask = varfun(@isnumeric, T1, 'OutputFormat','uniform');
    v1(:, numMask) = w1 * v1(:, numMask) + w2 * v2(:, numMask);

    the_weather = T1;
    the_weather.Variables = v1;
else
    the_weather = weather_data{1}.data;
end

end