% CARP_TO_PROPAGATOR Convert CARP outputs to ENU initial conditions for the propagator.
%   Transforms the aircraft state at the computed release point (altitude,
%   groundspeed, heading) into the 26-element ENU state vector expected by
%   propagate_model. Both the payload and parachute are initialized at or
%   just above the release altitude with the aircraft's horizontal velocity.
%
%   The state vector layout is:
%     States 1-3   : Payload ENU position (m)
%     States 4-6   : Payload velocity in body frame (m/s)
%     States 7-10  : Payload orientation quaternion [w,x,y,z] (XYZ Euler)
%     States 11-13 : Payload angular velocity (rad/s), initialized to zero
%     States 14-16 : Parachute ENU position (m), 1 m above payload
%     States 17-19 : Parachute velocity in body frame (m/s)
%     States 20-23 : Parachute orientation quaternion
%     States 24-26 : Parachute angular velocity (rad/s), initialized to zero
%
% INPUTS:
%   carp      : CARP results struct from Carp_Calculator2 (not directly used
%               here — all values are taken from carp_data)
%   carp_data : CARP data struct from get_flight_estimates, requiring:
%                 - altitude    : Release altitude (ft)
%                 - groundspeed : Aircraft groundspeed at release (kts)
%                 - heading     : Aircraft true heading at release (°)
%
% OUTPUTS:
%   x0 : 26x1 ENU initial state vector for propagate_model
%
% NOTES:
%   - The carp argument is accepted but unused; all state information comes
%     from carp_data. The argument may be a holdover from an earlier interface.
%   - Heading sign convention: heading_rad = -deg2rad(heading) maps from
%     navigation/compass convention to math/ENU convention.
%   - Vertical velocity is assumed zero at release (level flight).
%   - The parachute is placed 1 m above the payload and initialized with
%     zero vertical velocity.
%   - body2enu_rotm is applied to velocities, but since V_p0_for_state is
%     already in ENU, the result remains in ENU (see "NO TRANSFORMATION" note).


function x0 = Carp_To_Propagator(carp, carp_data)
    % Convert CARP outputs to ENU initial conditions for propagator (East
    % North Up)
    
    % Initial altitude (convert ft to m)
    alt_m = carp_data.altitude * 0.3048; %payload falls to the ground hence the true_altitude - terrain_elevation
    
    % Initial horizontal velocity (convert knots to m/s)
    gs_ms = carp_data.groundspeed * 0.514444;  % knots to m/s
    heading_rad = -deg2rad(carp_data.heading);
    
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
        body2enu_rotm(e_p0)' * V_p0_for_state;    % 4-6:   Payload velocity (ENU frame)
        e_p0;              % 7-10:  Payload quaternion
        w_p0;              % 11-13: Payload angular velocity
        P0_c;              % 14-16: Parachute position (ENU)
        body2enu_rotm(e_c0)' * V_c0_for_state;    % 17-19: Parachute velocity (ENU frame)
        e_c0;              % 20-23: Parachute quaternion
        w_c0;              % 24-26: Parachute angular velocity
    ];
end
