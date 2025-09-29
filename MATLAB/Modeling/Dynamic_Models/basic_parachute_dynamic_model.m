function x_dot = basic_parachute_dynamic_model(~, x_curr, payload)
% ======================
% --- Current States ---
% ======================

P       = x_curr(1:3);   % ECEF Position               [m]
V_p     = x_curr(4:6);   % Body axis velocity          [m   s^-1]

eul     = x_curr(7:9);   % Body orientation, ECEF      [rad]
w_p     = x_curr(10:12); % Body angular rates          [rad s^-1]

P_c = x_curr(13:15);     % Canopy ECEF Position        [m]
V_c = x_curr(16:18);     % Canopy Body Velocity        [m   s^-1]

% --- State Extraction ---
u = V_p(1); v = V_p(2); w = V_p(3);
V = max(norm(V_p), 1e-20); % Avoid undefined 

p = w_p(1); q = w_p(2); r = w_p(3); % Body rates

phi = eul(1); theta = eul(2); psi = eul(3); % Body angles

% ==========================
% --- Physical Constants ---
% ==========================
g       = 9.81;       % Gravitational acceleration       [m  s^-2]
g_vec_e = [0; 0; -g]; % Gravity vector in ECEF           [m  s^-2]

m_p    = 30;          % Parachute mass [kg]
l0     = 3;           % Parachute offset length          [m]
l_r     = 2;          % Riser length                     [m]

rho     = 1.225;      % Density of air                   [kg m^-3]

P_a = [payload.R; 0; 0]; % Attachment point of spring to object, in the body frame

% =================
% --- Rotations ---
% =================

% [yaw pitch roll] for MATLAB default ZYX
C_BE = eul2rotm([psi, theta, phi])';  % ECEF -> body (body from ECEF)

alpha = atan(abs(w / u));                     % Angle of attack
beta  = asin(abs(v/V));                       % Side slip angle
gamma = flight_path_angle(C_BE, alpha, beta); % Flight path angle

% ==============================
% --- Spring Characteristics ---
% ==============================

k = 10;
c = 1000;

% ===========================
% --- Equations of Motion ---
% ===========================

% --- Body Forces ---

F_g = C_BE*g_vec_e * payload.mass; % Force of gravity

F_d = -1/2 * rho * payload.CdS(0) * V * V_p;

V_p_e = C_BE' * V_p;
V_c_e = C_BE' * V_c;

% --- Attachment Point ---
r_attach_e = C_BE' * P_a;   % Attach vector from CG in ECEF
P_attach_e = P + r_attach_e;  % Position of attach point in ECEF

% --- Angular Velocity ---
omega_e = C_BE' * w_p;  % Angular rate in ECEF

% --- Velocity of attachment point ---
V_attach_e = V_p_e + cross(omega_e, r_attach_e);

% --- Vector from Riser to Attachment (ECEF) ---
r_p_r_vec = P_attach_e - P_c;
r_p_r = norm(r_p_r_vec);
e_p_r = r_p_r_vec / r_p_r;

extension = r_p_r - l0; % Extension of riser from nominal

v_p_r_vec = V_attach_e - V_c_e; % Velocity of attachment point relative to riser
v_p_r_radial = dot(v_p_r_vec, e_p_r); % Velocity of attachment point along radial spring direction

F_spring = -(k * extension + c * v_p_r_radial) * e_p_r; % Force of the spring

F_spring_p = C_BE * F_spring; % Force of the spring in the body frame

% --- Body Moments ---

F_p = F_g + F_d + F_spring_p; % Body forces [N]
M_p = cross(P_a, F_spring_p); % Body moments [N m]

F_D_c = -0.78 * 1/2 * rho * 100 * V * V_c;

F_spring_c = -F_spring_p;
% F_spring_c(1) = -F_spring_c(1);

F_c = F_g + F_D_c + F_spring_c;

% ===========================
% --- Kinematics ---
% ===========================

[a_p, alpha_p] = particle_model([V_p; w_p], payload.mass, payload.I, F_p, M_p);

[a_c, ~] = particle_model([V_c; 0; 0; 0], m_p, 0, F_c, [0; 0; 0]);

% --- Euler-angle kinematics: convert body rates to Euler rates
% Avoid singularity near cos(theta) = 0
ct = cos(theta); st = sin(theta);
sp = sin(phi); cp = cos(phi);

if abs(ct) < 1e-6
    % handle near-singularity: you can saturate or use alternative representation
    warning('theta near +-pi/2: Euler kinematics near singularity');
    ct = sign(ct)*1e-6;
end

% Transform body rates [p;q;r] to Euler rates [phi_dot;theta_dot;psi_dot]
T = [ 1, sp.*tan(theta),   cp.*tan(theta);
      0,        cp,           -sp;
      0, sp./ct,        cp./ct ];

eul_dot = T * w_p;   % [phi_dot; theta_dot; psi_dot]

% ==============
% --- States ---
% ==============
x_dot = [
    V_p_e;
    a_p;

    eul_dot;

    alpha_p;

    V_c_e;
    a_c;
    ];
end