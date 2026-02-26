function results = Results(carp, t, y, model_obj, ~)

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

% Descent rate analysis
if final_idx > 1
    alt_change = payload_altitude(1:final_idx-1) - payload_altitude(2:final_idx);
    time_step = t(2:final_idx) - t(1:final_idx-1);
    descent_rates = alt_change ./ time_step;  % m/s
    valid_descent = descent_rates > 0;
    if any(valid_descent)
        mean_descent_rate = mean(descent_rates(valid_descent));
        terminal_descent_rate = mean(descent_rates(max(1,final_idx-100):final_idx-1));  % Last 100 steps
    else
        mean_descent_rate = 0;
        terminal_descent_rate = 0;
    end
else
    mean_descent_rate = 0;
    terminal_descent_rate = 0;
end


if isfield(model_obj, 'payload')
    fprintf('Payload Drag:\n');
    % Try to access drag properties
    fprintf('  Payload dimensions: %.2f × %.2f × %.2f m\n', ...
        model_obj.payload.width, model_obj.payload.length, model_obj.payload.height);

    % Calculate terminal velocities
    fprintf('\nTerminal Velocity Analysis:\n');
    payload_weight = model_obj.payload.m(1.225) * 9.81;  % Weight in N
    parachute_weight = model_obj.parachute.canopy_mass * 9.81;
    total_weight = payload_weight + parachute_weight;

    fprintf('  Total weight:     %.2f N (%.2f lb)\n', total_weight, total_weight/4.448);
    fprintf('  Terminal V (actual): %.2f m/s (%.2f ft/s)\n', terminal_descent_rate, terminal_descent_rate/0.3048);

    % Required drag force
    fprintf('  Required drag force: %.2f N\n', total_weight);

    % At terminal velocity, what CdS is needed?
    rho = 1.225;  % kg/m³
    V_term_actual = terminal_descent_rate;
    V_term_carp = carp.adj_rof * 0.3048;

    CdS_needed_carp = total_weight / (0.5 * rho * V_term_carp^2);
    CdS_actual = total_weight / (0.5 * rho * V_term_actual^2);

    fprintf('  CdS needed (CARP):   %.2f m²\n', CdS_needed_carp);
    fprintf('  CdS actual:          %.2f m²\n', CdS_actual);
    fprintf('  Parachute area:      %.2f m²\n', pi * model_obj.parachute.radius^2);
    fprintf('  Effective Cd:        %.3f\n', CdS_actual / (pi * model_obj.parachute.radius^2));
end
fprintf('==========================================\n');
end
