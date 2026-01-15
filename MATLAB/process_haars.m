clear; clc
main_out_dir_name = "haars_data";
zip_out_dir_name = main_out_dir_name + "_extract";

zip_name = "HAARS Data.zip";

if ~isfolder(zip_out_dir_name)
    disp("Unzipping Main Zip")
    unzip(zip_name, zip_out_dir_name);
end

if ~isfolder(main_out_dir_name)
    mkdir(main_out_dir_name);
end

files = dir(fullfile(zip_out_dir_name, '*.zip'));
num_files = length(files);

for n = 1:num_files
    filename = files(n).name;
    [pathstr, name, ext] = fileparts(filename);

    if ~strcmp(ext, ".zip"), continue; end

    out_dir = fullfile(main_out_dir_name, name);

    if ~isfolder(out_dir), mkdir(out_dir); end

    if isfile(fullfile(out_dir, "SENSOR.CSV"))
        disp("Skipping " + fullfile(zip_out_dir_name, filename))
        continue
    end

    disp("Unzipping " + fullfile(zip_out_dir_name, filename))
    unzip(fullfile(zip_out_dir_name, filename), out_dir);

    subfiles = dir(out_dir);

    subdir = [];

    for i = 1:length(subfiles)
        if strcmp(name, subfiles(i).name)
            subdir = subfiles(i);
            break;
        end
    end

    if isempty(subdir), continue; end

    subsubfiles = dir(fullfile(out_dir, subdir.name));
    subsubdir = [];

    for i = 1:length(subsubfiles)
        if ~contains(subsubfiles(i).name, ".")
            subsubdir = subsubfiles(i);
            break;
        end
    end
    
    if isempty(subsubdir), continue; end

    disp("Copying " + fullfile(zip_out_dir_name, filename))
    data_dir = fullfile(subsubdir.folder, subsubdir.name);

    sensor_out_name = fullfile(out_dir, "SENSOR.CSV");
    sensor_cpy_name = fullfile(data_dir, "SENSOR.CSV");
    copyfile(sensor_cpy_name, sensor_out_name);
    
    track_out_name = fullfile(out_dir, "TRACK.CSV");
    track_cpy_name = fullfile(data_dir, "TRACK.CSV");
    copyfile(track_cpy_name, track_out_name);

    rmdir(fullfile(subdir.folder, subdir.name), 's')

    disp("Creating .mat file for " + fullfile(zip_out_dir_name, filename))
    
    try
        get_flysight_data(sensor_out_name, track_out_name);
    catch me
        rmdir(out_dir, 's')
        throw(me)
        %{
        disp("Deleting folder for " + out_dir)
        disp("Error: " + me.message)
        error("!")
        %}
    end
    %}

end

rmdir(zip_out_dir_name, 's')
