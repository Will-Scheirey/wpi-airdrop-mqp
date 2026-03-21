% HARP_DYNAMIC_MODEL_RESULTS Extract landing and trajectory data from propagator output.
%   Post-processes the raw time and state arrays from propagate_model to
%   identify ground impact, extract the ENU trajectory, and compute final
%   displacement metrics.
%
%   Body-frame velocities (states 4-6) are rotated to the ENU frame using
%   the payload quaternion (states 7-10), though the resulting ENU velocity
%   array is computed but not currently stored in the output struct.
%
% INPUTS:
%   t : Nx1 time vector from propagate_model (s)
%   y : NxM state matrix from propagate_model, where columns are:
%         1-3   : Payload ENU position (m)
%         4-6   : Payload body-frame velocity (m/s)
%         7-10  : Payload orientation quaternion [w, x, y, z]
%         11-13 : Payload angular velocity (rad/s)
%         14-16 : Parachute ENU position (m)
%         17-19 : Parachute body-frame velocity (m/s)
%         20-23 : Parachute orientation quaternion
%         24-26 : Parachute angular velocity (rad/s)
%
% OUTPUTS:
%   prop : Struct containing:
%            - t_plot      : Full time vector (s), identical to t
%            - y_sim       : Full state matrix, identical to y
%            - time        : Time vector from t=0 to ground impact (s)
%            - trajectory  : Mx3 ENU position matrix to ground impact (m)
%            - landing_time                  : Time of ground impact (s)
%            - east_displacement             : East offset at landing (m)
%            - north_displacement            : North offset at landing (m)
%            - total_horizontal_displacement : Horizontal range at landing (m)
%
% NOTES:
%   - Ground impact is defined as payload_altitude (column 3) first
%     reaching <= 0. If the payload never reaches the ground within tspan,
%     a warning is issued and the final time step is used.
%   - ENU velocities are computed internally but not currently included in
%     the output struct. Add if needed for post-processing.

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
