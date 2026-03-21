% PROCESS_HAARS_ALL Processes all HAARS data within a .zip file
%   This script unzips, extracts, and process and saves to a .MAT file the
%   HAARS-formatted data contained within a .zip file. 
%
%   ======= NOTE: THIS CAN TAKE MULTIPLE HOURS TO RUN =======
%   
%   Depending on the number of drops and the data contained in each drop,
%   this can take a very long time to run. It is recommended to run this on
%   a remote server. This could be sped up by using parallel computing, but
%   the parallel accesses to the same directories and the parallel opening
%   of multiple files can be really wonky

% Get user input
result = questdlg('Select a .zip file to extract', 'Proceed?', 'Yes', 'Cancel', 'Yes');

if strcmp(result, "Cancel")
    return
end

file = uigetfile("*.zip");

% This is the folder where output data will be stored
main_out_dir_name = "haars_data";

% Generate a temporary folder for extraction
zip_out_dir_name = main_out_dir_name + "_temp";

zip_name = file;

% If the .zip extraction folder already exists, don't unzip the file
if ~isfolder(zip_out_dir_name)
    disp("Unzipping Main Zip")
    unzip(zip_name, zip_out_dir_name);
end

% Create the output directory
if ~isfolder(main_out_dir_name)
    mkdir(main_out_dir_name);
end

% Walk through each of the ,zip files corresponding to each drop
files = dir(fullfile(zip_out_dir_name, '*.zip'));
num_files = length(files);
for n = 1:num_files
    filename = files(n).name;
    [~, name, ext] = fileparts(filename);

    % Make sure the file we are unzipping is a .zip file
    if ~strcmp(ext, ".zip"), continue; end

    out_dir = fullfile(main_out_dir_name, name);

    % Create the output folder
    if ~isfolder(out_dir), mkdir(out_dir); end

    % If a .mat file already exists, don't bother processing the data
    if isfile(fullfile(out_dir, "SENSOR.mat"))
        disp("Skipping " + fullfile(zip_out_dir_name, filename))
        continue
    end

    to_unzip = fullfile(zip_out_dir_name, filename);

    % Some files are extremely large and would literally take many hours to
    % process individual .zip files
    if files(n).bytes > 100e6
        result = questdlg(sprintf('The following file is %0.2f bytes: \n\n%s', files(n).bytes/1e6, to_unzip), 'Process?', 'Yes', 'Cancel', 'Yes');

        if strcmp(result, 'Cancel')
            continue
        end
    end

    % Unzip the file
    disp("Unzipping " + fullfile(zip_out_dir_name, filename))
    unzip(to_unzip, out_dir);

    % Find the extracted data and make sure we found it
    subfiles = dir(out_dir);
    subdir = [];
    for i = 1:length(subfiles)
        if strcmp(name, subfiles(i).name)
            subdir = subfiles(i);
            break;
        end
    end
    if isempty(subdir), continue; end

    % Extract the subdirectory again (quirk of the HAARS data)
    subsubfiles = dir(fullfile(out_dir, subdir.name));
    subsubdir = [];
    for i = 1:length(subsubfiles)
        if ~contains(subsubfiles(i).name, ".")
            subsubdir = subsubfiles(i);
            break;
        end
    end
    if isempty(subsubdir), continue; end

    % Copy just the sensor and GPS CSV files into the output directory
    disp("Copying " + fullfile(zip_out_dir_name, filename))
    data_dir = fullfile(subsubdir.folder, subsubdir.name);

    sensor_out_name = fullfile(out_dir, "SENSOR.CSV");
    sensor_cpy_name = fullfile(data_dir, "SENSOR.CSV");
    copyfile(sensor_cpy_name, sensor_out_name);

    track_out_name = fullfile(out_dir, "TRACK.CSV");
    track_cpy_name = fullfile(data_dir, "TRACK.CSV");
    copyfile(track_cpy_name, track_out_name);

    % Delete the extracted .zip data
    rmdir(fullfile(subdir.folder, subdir.name), 's')

    % Run 'load_flysight_data', which will generate the .MAT file for the
    % data
    disp("Creating .mat file for " + fullfile(zip_out_dir_name, filename))
    try
        load_flysight_data(sensor_out_name, track_out_name);
    catch me
        rmdir(out_dir, 's')
        throw(me)
    end
end

% Remove the temporary .zip output directory
rmdir(zip_out_dir_name, 's')