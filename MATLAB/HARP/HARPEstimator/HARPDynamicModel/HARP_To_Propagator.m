function x0 = HARP_To_Propagator(data_out)
    % Convert CARP outputs to ENU initial conditions for propagator (East
    % North Up)
    
    % Initial altitude (convert ft to m)
    alt_m = data_out.carp.altitude * 0.3048; %payload falls to the ground hence the true_altitude - terrain_elevation
    
    % Initial horizontal velocity (convert knots to m/s)
    gs_ms = data_out.carp.groundspeed * 0.514444;  % knots to m/s
    heading_rad = -deg2rad(data_out.carp.heading);
    
    % ENU velocity components (aircraft velocity at release)
    V_east = gs_ms * sin(heading_rad);
    V_north = gs_ms * cos(heading_rad);
    V_up = 0;  % Level flight - NO VERTICAL VELOCITY at release
    
    % --- Payload Initial Conditions ---
    P0 = [0; 0; alt_m];                     % ENU position [m]
    V_p0_ENU = [V_east; V_north; V_up];     % ENU velocity [m/s]
    
    % Orientation: payload aligned with velocity (heading only)
    e_p0 = eul2quat([0, 0, heading_rad], 'XYZ')';  % [roll, pitch, yaw]
    w_p0 = [0; 0; 0];                       % No angular velocity
    
    % CRITICAL FIX: Pass velocity directly in ENU frame (no transformation)
    V_p0_for_state = V_p0_ENU;  % NO TRANSFORMATION
    
    % --- Parachute Initial Conditions ---
    P0_c = P0 + [0; 0; 1];  % 1m above payload (within riser length)
    
    % Parachute already deployed and descending
    initial_descent = 0;  %starts at rest
    V_c0_ENU = [V_east; V_north; initial_descent];
    e_c0 = e_p0;  % Canopy facing up (no rotation)
    w_c0 = [0; 0; 0];
    
    % Also pass parachute velocity in ENU frame (no transformation)
    V_c0_for_state = V_c0_ENU;  % NO TRANSFORMATION
    
    % Assemble state vector (26 states)
    x0 = [
        P0;                % 1-3:   Payload position (ENU)
        ecef2body_rotm(e_p0) * V_p0_for_state;    % 4-6:   Payload velocity (ENU frame)
        e_p0;              % 7-10:  Payload quaternion
        w_p0;              % 11-13: Payload angular velocity
        P0_c;              % 14-16: Parachute position (ENU)
        ecef2body_rotm(e_c0) * V_c0_for_state;    % 17-19: Parachute velocity (ENU frame)
        e_c0;              % 20-23: Parachute quaternion
        w_c0;              % 24-26: Parachute angular velocity
    ];
end
