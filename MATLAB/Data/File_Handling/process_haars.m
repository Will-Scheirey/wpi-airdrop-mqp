function process_haars()
result = questdlg('Select a .zip file to extract', 'Proceed?', 'Yes', 'Cancel', 'Yes');

if strcmp(result, "Cancel")
    return
end

file = uigetfile("*.zip");

main_out_dir_name = "haars_data";
zip_out_dir_name = main_out_dir_name + "_temp";

zip_name = file;

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
    [~, name, ext] = fileparts(filename);

    if ~strcmp(ext, ".zip"), continue; end

    out_dir = fullfile(main_out_dir_name, name);

    if ~isfolder(out_dir), mkdir(out_dir); end

    if isfile(fullfile(out_dir, "SENSOR.mat"))
        disp("Skipping " + fullfile(zip_out_dir_name, filename))
        continue
    end

    to_unzip = fullfile(zip_out_dir_name, filename);

    if files(n).bytes > 100e6
        result = questdlg(sprintf('The following file is %0.2f bytes: \n\n%s', files(n).bytes/1e6, to_unzip), 'Process?', 'Yes', 'Cancel', 'Yes');

        if strcmp(result, 'Cancel')
            continue
        end
    end

    disp("Unzipping " + fullfile(zip_out_dir_name, filename))
    unzip(to_unzip, out_dir);

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
    end
end

rmdir(zip_out_dir_name, 's')
end