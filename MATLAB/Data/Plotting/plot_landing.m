clear; clc; close all
data_dir = "haars_data";

if ~isfolder(data_dir)
    error("The folder " + data_dir + " does not exist!")
end

drop_dirs = dir(data_dir);

figure(100)
for n=1:length(drop_dirs)
    the_dir = drop_dirs(n);
    dir_name = fullfile(data_dir, the_dir.name);

    if contains(dir_name, ".")
        continue
    end

    
    if contains(dir_name, "08052025")
        continue
    end
    

    if ~the_dir.folder
        continue
    end

    sensor_filename = fullfile(dir_name, "SENSOR.CSV");
    gps_filename = fullfile(dir_name, "TRACK.CSV");

    if ~isfile(sensor_filename) || ~isfile(gps_filename)
        continue
    end

    num_bytes = dir(sensor_filename).bytes;

    if num_bytes > 100e6
        continue
    end

    fprintf("#%d, Name: %s\n", n, dir_name)

    [data_accel,...
    data_gyro,...
    data_mag,...
    data_gps,...
    data_baro,...
    data_gps_vel,...
    data_flysight_sensor,...
    data_flysight_gps] = get_trim_align(sensor_filename, gps_filename);

    if isempty(data_gps)
        continue;
    end

    all_landings(n, :) = data_gps.data(end, 1:2);
end

all_landings_scaled = rescale_data(all_landings);
all_landings_scaled = rescale_data(all_landings_scaled);


%% PLOT

stddev = std(all_landings_scaled);
fprintf("Standard Dev\n\tX: %0.2fm\n\tY: %0.2fm\n", stddev(1), stddev(2));

plot(all_landings_scaled(:,1), all_landings_scaled(:,2), 'rx', 'MarkerSize', 15, 'LineWidth', 1.5); hold on
legend

function all_landings = rescale_data(all_landings)
    outlier_x = isoutlier(all_landings(:, 1));
    all_landings = all_landings(~outlier_x, :);
    
    outlier_y = isoutlier(all_landings(:, 2));
    all_landings = all_landings(~outlier_y, :);
    
    all_landings(:, 1) = all_landings(:, 1) - median(all_landings(:, 1)); 
    all_landings(:, 2) = all_landings(:, 2) - median(all_landings(:, 2)); 
end
