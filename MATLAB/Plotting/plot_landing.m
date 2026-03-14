clear; clc; close all
data_dir = "haars_data";

if ~isfolder(data_dir)
    error("The folder " + data_dir + " does not exist!")
end

drop_dirs = dir(data_dir);

use_date = false;
data_str = "08052025";

figure(100)
for n=1:length(drop_dirs)
    the_dir = drop_dirs(n);
    dir_name = fullfile(data_dir, the_dir.name);

    all_names{n} = dir_name;

    if contains(dir_name, ".")
        continue
    end

    if use_date
        if ~contains(dir_name, data_str)
            continue
        end
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

    drop_info = get_drop_info(data_accel, data_gyro, data_mag, data_gps, data_baro, data_gps_vel, data_flysight_sensor, data_flysight_gps);
    
    if isempty(drop_info)
        continue;
    end

    all_landings(n, :) = drop_info.gps_land;
    all_drops(n, :) = drop_info.gps_drop;
    all_vel(n, :) = drop_info.vel_drop; 
    time_drop(n, :) = drop_info.time_drop;
end

[all_drops_scaled, all_landings_scaled, all_vel_scaled] = rescale_data(all_landings, all_drops, all_vel, false);
[all_drops_scaled, all_landings_scaled, all_vel_scaled] = rescale_data(all_landings_scaled, all_drops_scaled, all_vel_scaled, false);
[all_drops_scaled, all_landings_scaled, all_vel_scaled] = rescale_data(all_landings_scaled, all_drops_scaled, all_vel_scaled, true);


%% PLOT

stddev = std(all_landings_scaled);
fprintf("Standard Dev\n\tX: %0.2fm\n\tY: %0.2fm\n", stddev(1), stddev(2));

z = zeros(height(all_landings_scaled), 1);

figure(1)
clf
plot3(all_drops_scaled(:,1), all_drops_scaled(:,2), all_drops_scaled(:,3), 'ob', 'MarkerSize', 15, 'LineWidth', 1.5); hold on
plot3(all_landings_scaled(:,1), all_landings_scaled(:,2), all_landings_scaled(:,3), 'rx', 'MarkerSize', 15, 'LineWidth', 1.5); hold on

for n = 1:height(all_landings_scaled)
    quiver3(all_drops_scaled(n,1), all_drops_scaled(n,2), all_drops_scaled(n,3), all_vel_scaled(n,1), all_vel_scaled(n,2), all_vel_scaled(n,3), 'g')
    plot3([all_drops_scaled(n,1), all_landings_scaled(n,1)], [all_drops_scaled(n,2), all_landings_scaled(n,2)], [all_drops_scaled(n,3), 0], '-k', 'LineWidth', 0.5); hold on
end
axis equal
xlabel("East")
ylabel("North")
zlabel("Up")

figure(2)
clf
drift = all_landings_scaled(:, 1:2) - all_drops_scaled(:, 1:2);
drift_slope = vecnorm(drift, 2, 2) ./ (all_drops_scaled(:, 3) - all_landings_scaled(:, 3));
plot(all_drops_scaled(:, 3), drift_slope, '.r', 'MarkerSize', 15, 'LineWidth', 1.5); hold on


high_alt_drop = all_drops(:,3) > 3000;
time_drop_high_alt = time_drop(high_alt_drop);
time_drop_low_alt = time_drop(~high_alt_drop);

figure(3)
time_drop_high_alt = sort(time_drop_high_alt);
plot(time_drop_high_alt)

figure(4)
time_drop_low_alt = sort(time_drop_low_alt);
plot(time_drop_low_alt)



function [all_drops, all_landings, all_vel] = rescale_data(all_landings, all_drops, all_vel, drop)
if ~drop
    outlier_x = isoutlier(all_landings(:, 1));
    all_landings = all_landings(~outlier_x, :);
    all_drops = all_drops(~outlier_x, :);
    all_vel = all_vel(~outlier_x, :);

    outlier_y = isoutlier(all_landings(:, 2));
    all_landings = all_landings(~outlier_y, :);
    all_drops = all_drops(~outlier_y, :);
    all_vel = all_vel(~outlier_y, :);
end

if drop
    outlier_x = isoutlier(all_drops(:, 1), 'mean');
    all_landings = all_landings(~outlier_x, :);
    all_drops = all_drops(~outlier_x, :);
    all_vel = all_vel(~outlier_x, :);

    outlier_y = isoutlier(all_drops(:, 2), 'mean');
    all_landings = all_landings(~outlier_y, :);
    all_drops = all_drops(~outlier_y, :);
    all_vel = all_vel(~outlier_y, :);
end

if ~drop
    all_landings_median_x = median(all_landings(:, 1));
    all_landings_median_y = median(all_landings(:, 2));

    all_landings(:, 1) = all_landings(:, 1) - all_landings_median_x;
    all_drops(:, 1) = all_drops(:, 1) - all_landings_median_x;

    all_landings(:, 2) = all_landings(:, 2) - all_landings_median_y;
    all_drops(:, 2) = all_drops(:, 2) - all_landings_median_y;
end
end
