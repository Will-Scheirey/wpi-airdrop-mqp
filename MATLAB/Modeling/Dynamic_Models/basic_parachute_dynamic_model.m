function x_dot = basic_parachute_dynamic_model(~, x_curr, payload, parachute)
% ======================
% --- Current States ---
% ======================

P       = x_curr(1:3);   % ECEF Position               [m]
V_p     = x_curr(4:6);   % Body axis velocity          [m   s^-1]

% eul     = x_curr(7:9);   % Body orientation, ECEF      [rad]
e_p     = x_curr(7:10);
w_p     = x_curr(11:13); % Body angular rates          [rad s^-1]

P_c = x_curr(14:16);     % Canopy ECEF Position        [m]
V_c = x_curr(17:19);     % Canopy Body Velocity        [m   s^-1]

% eul_c = x_curr(19:21);   % Canopy orientation, ECEF    [rad]
e_c     = x_curr(20:23);
w_c   = x_curr(24:26);   % Canopy angular rates        [rad s^-1]

% --- State Extraction ---

h = P(3); % Altitude

u = V_p(1); v = V_p(2); w = V_p(3);
V = max(norm(V_p), 1e-20); % Avoid undefined 

V_c_norm = max(norm(V_c), 1e-20);

p = w_p(1); q = w_p(2); r = w_p(3); % Body rates

% phi = eul(1); theta = eul(2); psi = eul(3); % Body angles

% phi_c = eul_c(1); theta_c = eul_c(2); psi_c = eul_c(3); % Body angles

% ==========================
% --- Physical Constants ---
% ==========================
g       = 9.81;        % Gravitational acceleration             [m  s^-2]
g_vec_e = [0; 0; -g];  % Gravity vector in ECEF                 [m  s^-2]

rho     = StandardAtmosphereModel.Density(h); % Density of air  [kg m^-3]

% =================
% --- Rotations ---
% =================

% [yaw pitch roll] for MATLAB default ZYX
% C_EB   = eul2rotm([psi, theta, phi])';  % ECEF -> body (body from ECEF)
% C_EB_c = eul2rotm([psi_c, theta_c, phi_c])'; % ECEF -> body (body from ECEF)

C_EB   = ecef2body_rotm(e_p);                    % ROTM from ECEF to Body
C_EB_c = ecef2body_rotm(e_c);                    % ROTM from ECEF to Body

alpha = atan(abs(w / u));                     % Angle of attack
beta  = asin(abs(v/V));                       % Side slip angle
gamma = flight_path_angle(C_EB, alpha, beta); % Flight path angle

% ==============================
% --- Spring Characteristics ---
% ==============================

k = 10000;
c = 10000;

% ===========================
% --- Equations of Motion ---
% ===========================

% --- Body Forces ---

F_g_p = C_EB   * g_vec_e * payload.mass; % Force of gravity
F_g_c = C_EB_c * g_vec_e * parachute.mass; % Force of gravity

F_d_p = -1/2 * rho * payload.CdS(0) * V * V_p;
F_d_c = -1/2 * rho * parachute.CdS(0) * norm(V_c) * V_c;

V_p_e = C_EB'   * V_p;
V_c_e = C_EB_c' * V_c;

% --- Velocity of attachment point ---
% compute relative attach vectors (ECEF)
r_attach_p_e = C_EB' * payload.P_attach_B;   % vector from payload COM -> attach (in ECEF)
r_attach_c_e = C_EB_c' * parachute.P_attach_B;

obj1 = struct( ...
    'V', V_p_e, ...                   % COM velocity, ECEF
    'omega', C_EB' * w_p, ...         % omega in ECEF
    'P_attach_rel', r_attach_p_e, ... % relative vector COM->attach in ECEF
    'P_attach', P + r_attach_p_e ...  % absolute attach pos in ECEF
    );

obj2 = struct( ...
    'V', V_c_e, ...
    'omega', C_EB_c' * w_c, ...
    'P_attach_rel', r_attach_c_e, ...
    'P_attach', P_c + r_attach_c_e ...
    );

spring = struct('l0', parachute.l0, 'k', k, 'c', c);

F_spring_e = spring_force(obj1, obj2, spring);

F_spring_p = C_EB   *  F_spring_e; % Force of the spring in the body frame
F_spring_c = C_EB_c * -F_spring_e;

% --- Body Moments ---

F_p = F_g_p + F_d_p + F_spring_p; % Body forces [N]
M_p = cross(payload.P_attach_B, F_spring_p) + -100*w_p; % Body moments [N m]

F_c = F_g_c + F_d_c + F_spring_c;
M_c = cross(parachute.P_attach_B, F_spring_c) + -100*w_c;

% ===========================
% --- Kinematics ---
% ===========================

[a_p, alpha_p] = particle_model([V_p; w_p], payload.mass, payload.I, F_p, M_p);

[a_c, alpha_c] = particle_model([V_c; w_c], parachute.mass, parachute.I, F_c, M_c);

% eul_dot_p = body_w_to_ecef(theta, phi, w_p);
% eul_dot_c = body_w_to_ecef(theta_c, phi_c, w_c);

e_p_dot   = -1/2 * quat_kinematic_matrix(w_p) * e_p; % Quaternion rates
e_c_dot   = -1/2 * quat_kinematic_matrix(w_c) * e_c; % Quaternion rates

% ==============
% --- States ---
% ==============
x_dot = [
    V_p_e;
    a_p;

    % eul_dot_p;
    e_p_dot
    alpha_p;

    V_c_e;
    a_c;

    % eul_dot_c;
    e_c_dot;
    alpha_c;
    ];
end