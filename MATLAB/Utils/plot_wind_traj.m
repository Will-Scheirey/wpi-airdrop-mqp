function plot_wind_traj(wind_alt_ft, wind_direction_deg, wind_speed_fps, trajectory_ft, step, scale)
    if nargin < 5
        step = 1;
    end
    if nargin < 6
        scale = 0;
    end

    trajectory_ft = trajectory_ft(1:step:end, :);

    alt_points = trajectory_ft(:, 3);

    wind_interp_angle = interp1(wind_alt_ft, wind_direction_deg, alt_points);
    wind_interp_speed = interp1(wind_alt_ft, wind_speed_fps, alt_points);

    wind_origin = trajectory_ft;

    wind_end_x = wind_interp_speed .* cosd(wind_interp_angle);
    wind_end_y = wind_interp_speed .* sind(wind_interp_angle);
    wind_end_z = trajectory_ft(:, 3) * 0;

    quiver3(wind_origin(:, 1), wind_origin(:, 2), wind_origin(:, 3), wind_end_x, wind_end_y, wind_end_z, scale);
end