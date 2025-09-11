function x_dot = spring_drag_dynamic_model(~, x_curr, P0)
% ======================
% --- Current States ---
% ======================

V_b     = x_curr(1:3);   % Body axis velocity          [m   s^-1]
u = V_b(1); v = V_b(2); w = V_b(3);
V = norm(V_b);
V = max(V, 1e-20); % Avoid undefined 

w_b     = x_curr(4:6);   % Body angular rates          [rad s^-1]
p = w_b(1); q = w_b(2); r = w_b(3);

e       = x_curr(7:10);  % Orientation quaternion      []
P       = x_curr(11:13); % ECEF Position               [m]


% ==========================
% --- Physical Constants ---
% ==========================
g       = 9.81;       % Gravitational acceleration       [m  s^-2]
g_vec_e = [0; 0; -g]; % Gravity vector in ECEF           [m  s^-2]

m       = 1;         % Object mass                      [kg]
R       = 0.5;        % Object spherical radius          [m]

S       = pi*R^2;     % Cross-sectional area of a sphere [m^2]
Cd      = 0.07;       % Drag coefficient of a sphere     []

l0     = 2;        % Parachute offset length          [m]
l_r     = 2;          % Riser length                     [m]

rho     = 1.225;      % Density of air                   [kg m^-3]

% ===================
% --- Preliminary ---
% ===================
Q = 1/2*rho*V^2; % Dynamic pressure


% =================
% --- Rotations ---
% =================

C_BE  = ecef2body_rotm(e);                    % ROTM to Body from ECEF

alpha = atan(abs(w / u));                     % Angle of attack
beta  = asin(abs(v/V));                       % Side slip angle
gamma = flight_path_angle(C_BE, alpha, beta); % Flight path angle

% ==========================
% --- Parachute Dynamics ---
% ==========================

% [k, c] = parachute_spring_coefficients();

k = 1;
c = 0.5;

% ===============
% --- Inertia ---
% ===============

I_XX = 2/5*m*R^2; % Moment of inertia of a sphere [kg m^2]
I_YY = I_XX;      % Moment of inertia of a sphere [kg m^2]
I_ZZ = I_XX;      % Moment of inertia of a sphere [kg m^2]

I_XY = 0;         % Moment of inertia of a sphere [kg m^2]
I_XZ = 0;         % Moment of inertia of a sphere [kg m^2]
I_YZ = 0;         % Moment of inertia of a sphere [kg m^2]

% Inertia Tensor
I = [
    I_XX, -I_XY, -I_XZ;
    -I_XY, I_YY, -I_YZ;
    -I_XZ, -I_YZ, I_ZZ;
    ];

% ===========================
% --- Equations of Motion ---
% ===========================

% --- Body Forces ---

F_g = C_BE*g_vec_e * m; % Force of gravity

F_d = -Q*S*Cd * V_b/V;

P_R_P = P0 - P; % Position of payload relative to the riser
V_R_P = C_BE' * -V_b;  % Velocity of riser relative to the payload

e_spring = P_R_P / norm(P_R_P); % Unit vector along spring axis

x_spring = P_R_P - l0 * e_spring;
x_dot_spring = (e_spring' * V_R_P) * e_spring;

F_spring = k * x_spring + c * x_dot_spring;

F_spring_b = C_BE * F_spring;

% --- Body Moments ---

M = [0; 0; 0];

F_b = F_g + F_d + F_spring_b; % Body forces [N]
M_b = M;   % Body moments [N m]

% ===========================
% --- Kinematics ---
% ===========================

[a_b, alpha_b] = particle_model([V_b; w_b], m, I, F_b, M_b);

e_dot   = -1/2 * quat_kinematic_matrix(w_b) * e; % Quaternion rates
V_e     = C_BE' * V_b;                           % Velocity in ECEF

% ==============
% --- States ---
% ==============
x_dot = [
    a_b;

    alpha_b;

    e_dot;

    V_e;
    ];
end