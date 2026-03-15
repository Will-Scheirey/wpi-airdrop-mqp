function [weather_data, the_weather] = load_weather_data(the_time)
% LOAD_WEATHER_DATA Loads weather data for a given time and date
%   This is a function loads radar weather data provided with the HAARS 
%   data for a given time and date. Weather data contains air pressure,
%   air temperature, and wind direction and speed at varying altitudes
%
%   This function will throw an error if the requested time and data is not
%   included in the weather data
%
%   Since data are provided by the hour on given days, weather information 
%   is linearly interoplated between two data sets when the requested
%   datetime does not fall on an exact hour
%
%   The large majority of this code is just traversing the directories to
%   find the correct datasets
%
% INPUTS:
%   the_time : The datetime object for the requested data and time
%
% OUTPUTS:
%   weather_data : Cell array of structs for the weather data sets used
%   the_weather  : Struct of weather data for the requested datetime
%       .time : Datetime object for the data
%       .data : Table containing the weather data
% 

% Assume the filestructure is 'weather > YYYY-MM-DD > data > radar'
weather_dir = 'weather';

files = dir(weather_dir);
% Set the directory name to be empty so we can check if we found one later
day_dir = [];

% Go through all of the provided days and find the matching one
for i = 1:length(files)
    % The output of the 'dir' function includes the directories '.' (the
    % current directory, and '..' (the current directory's parent
    % directory), so skip these
    if contains(files(i).name, '.')
        continue
    end
    % Generate a datetime object from the directory
    dir_date = datetime(files(i).name, 'TimeZone', 'UTC');
    
    % Choose the directory where the days match. We use 'dateshift' to
    % shift both objects back to the start of their respective days for
    % comparison
    if dateshift(dir_date, 'start', 'day') == dateshift(the_time, 'start', 'day')
        % If we found the right directory, just exit the loop
        day_dir = files(i);
        break
    end
end

% Check that we found the a directory
if isempty(dir_date)
    error("There is no weather data for the specified date!")
end

% Generate the directory names for the day and for the data subdirectory
day_dir = fullfile(weather_dir, day_dir.name);
day_data_dir = fullfile(day_dir, 'data');

% Check a data folder exists
if ~isfolder(day_data_dir)
    error("There is no data file for the specified date!")
end

% Find the folder with the radar data
radar_dir = fullfile(day_data_dir, 'radar');

% Check the radar folder exists
if ~isfolder(radar_dir)
    error("There is no radar data file for the specified date!")
end

% Extract all the .csv files
radar_files = dir(fullfile(radar_dir, '*.csv'));

weather_files = {};
dates = {};
idx = 1;

% Loop through each of the files corresponding to a different hour
for i = 1:length(radar_files)
    file = radar_files(i);

    % Extract the filename and turn it into a datetime object
    filename = file.name;
    [~, file_name, ~] = fileparts(filename);

    file_time = datetime(file_name, ...
        'InputFormat','MMddHHmm''Z''', ...
        'TimeZone','UTC');
    file_time.Year = the_time.Year; % Set the year

    % Check if the file datetime is within one hour of the requested
    diff = abs(hours(the_time - file_time));
    if diff < 1
        % If we found a file within an hour, add it to our list
        weather_files{idx} = file;
        dates{idx} = file_time;
        idx = idx + 1;
    end
end

% Check we found weather data
if isempty(weather_files)
    error("No radar date for the given time of day!")
end

% For each of the weather data files we found within an hour of the
% requested datetime, extract the actual weather data
for i = 1:length(weather_files)
    file = weather_files{i};
    filename = file.name;

    % Generate the struct 
    weather_data{i} = struct('time', dates{i}, 'data', readtable(filename));
end

% Interpolate between datasets if we have multiple. This code was generated
% by ChatGPT but reviewed by a human. Maybe we could just use interp1?

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