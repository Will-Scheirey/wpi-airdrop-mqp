function carp_out = get_carp_params(data)
% GET_CARP_PARAMS Gets relevant CARP parameters from trajectory estimate
% 
% INPUTS:
%   data : Struct of data from trajectory estimate
%
% OUTPUTS:
%   carp_out : Struct of relevant carp parameters
%       .airspeed       : Airspeed of the airplane when the drop occurs
%           [ kts ]
%       .groundspeed    : Groundspeed of the plane when the drop occurs
%           [ kts ]
%       .heading        : Heading from North
%           [ deg ]
%       .altitude       : Altitude of the drop
%           [ ft ]
%       .wind_speed     : Windspeed when the drop occurs
%           [ kts ]
%           This takes the average of all windspeeds at varying altitudes
%       .wind_direction : Wind direction when the drop occurs
%           [ deg ]
%           This takes the average of wind directions at varying altitudes
%       .land_location  : Landing location relative to drop
%           [ m ] ENU
%       .time_off_fall  : Duration of the drop
%           [ s ]
%       .relative_traj  : Trajectory of the motion relative to drop
%           [ m ] ENU
%       .time_UTC       : UTC datetime object for when the drop occurs

% Find the time index when the drop occurred
t_drop_idx = find(data.t_plot > data.drop_info.time_drop, 1);

% Get the velocity, heading, and altitude when the drop occurs
v_drop   = mps2ks(data.estimates.vel(t_drop_idx, 1:2));
alt_drop = data.estimates.pos(t_drop_idx, 3) - data.estimates.pos(end, 3);

% Since velocity is in ENU, swap x and y to calculate the heading
heading  = atan2d(v_drop(1), v_drop(2));
altitude = m2ft(alt_drop);

% Average windspeed and direction
good_alt = data.weather.alt_agl < altitude/1000;

wind_speed     = mean(data.weather.win_speed(good_alt));
wind_direction = mean(data.weather.wind_direction(good_alt));

% Calculate the wind vector
wind_vec = wind_speed * [sind(wind_direction), cosd(wind_direction)];

% Calculate airspeed
air_vel  = v_drop - wind_vec;
airspeed = norm(air_vel);

% Calculate trajectory relaive to the drop location
relative_movement = data.drop_info.gps_land(1:2) - data.drop_info.gps_drop(1:2);

relative_traj         = data.estimates.pos(t_drop_idx:end, :);
relative_traj(:, 1:2) = data.estimates.pos(t_drop_idx:end, 1:2) - data.drop_info.gps_drop(1:2);
relative_traj(:, 3)   = data.estimates.pos(t_drop_idx:end, 3)   - data.estimates.pos(end, 3);

% Get the time when the drop occurs
time_utc = data.measurements.gps_all.GNSS.datetime_utc(end);

% Generate the output struct
carp_out = struct(...
    'airspeed',       airspeed, ...            % kts
    'groundspeed',    norm(v_drop),...         % kts
    'heading',        heading, ...             % deg
    'altitude',       altitude, ...            % ft
    'wind_speed',     ks2mps(wind_speed), ...  % kts
    'wind_direction', wind_direction, ...      % deg
    'land_location',  relative_movement, ...,  % m
    'time_of_fall',   data.drop_info.time_land - data.drop_info.time_drop, ... % s
    'relative_traj',  relative_traj, ...       % m
    'time_UTC',       time_utc ...             % datetime obj
    );

end