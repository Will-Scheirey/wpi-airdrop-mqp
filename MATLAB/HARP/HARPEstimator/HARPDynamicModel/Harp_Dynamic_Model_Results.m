function prop = Harp_Dynamic_Model_Results(t, y)

% Payload position is in columns 1-3 (ENU)
payload_altitude = y(:,3);

% Find when payload hits ground
final_idx = find(payload_altitude <= 0, 1);
if isempty(final_idx)
    final_idx = length(t);
    warning('Payload did not reach ground! Final altitude: %.2f m', payload_altitude(end));
end

prop.t_plot = t;
prop.y_sim = y;

% Store trajectory data for visualization
prop.time = t(1:final_idx);
prop.trajectory = y(1:final_idx, 1:3);  % East, North, Up (ENU position)

% Extract velocity data (columns 4-6 are body frame velocities)
V_body = y(1:final_idx, 4:6);
e_quat = y(1:final_idx, 7:10);  % Quaternions

% Convert body velocities to ENU
V_enu = zeros(size(V_body));
for i = 1:size(V_body,1)
    R_body2enu = quat2rotm(e_quat(i,:));  % Body to ENU rotation
    V_enu(i,:) = (R_body2enu * V_body(i,:)')';
end

% Propagator results
prop.landing_time = t(final_idx);
prop.east_displacement = y(final_idx, 1);
prop.north_displacement = y(final_idx, 2);
prop.total_horizontal_displacement = sqrt(prop.east_displacement^2 + prop.north_displacement^2);

end
