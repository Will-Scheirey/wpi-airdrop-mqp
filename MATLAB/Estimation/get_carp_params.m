function carp_out = get_carp_params(data)

t_drop_idx = find(data.t_plot > data.drop_info.time_drop, 1);

v_drop   = mps2ks(data.estimates.vel(t_drop_idx, 1:2));
alt_drop = data.estimates.pos(t_drop_idx, 3);

heading = atan2d(v_drop(1), v_drop(2));
altitude = m2ft(alt_drop);

good_alt = data.weather.alt_agl < altitude/1000;

wind_speed = mean(data.weather.win_speed(good_alt));
wind_direction = mean(data.weather.wind_direction(good_alt));

wind_vec = wind_speed * [sind(wind_direction), cosd(wind_direction)];

air_vel = v_drop - wind_vec;

airspeed = norm(air_vel);

relative_movement = data.drop_info.gps_land - data.drop_info.gps_drop;

carp_out = struct(...
    'airspeed', airspeed, ...                  % kts
    'groundspeed', norm(v_drop),...
    'heading', heading, ...              % deg
    'altitude', altitude, ...            % ft
    'wind_speed', wind_speed, ...        % kts
    'wind_direction', wind_direction, ... % deg
    'land_location', relative_movement, ..., % m
    'time_of_fall', data.drop_info.time_land - data.drop_info.time_drop ... % s
    );

end