function x_dot = basic_dynamic_model(t, x_curr)
% ======================
% --- Current States ---
% ======================

V_b     = x_curr(1:3);   % Body axis velocity     [m   s^-1]
w_b = x_curr(4:6);       % Body angular rates     [rad s^-1]
e       = x_curr(7:10);  % Orientation quaternion []
P       = x_curr(11:13); % ECEF Position          [m]

% ==========================
% --- Physical Constants ---
% ==========================
g = 9.81;              % Gravitational acceleration [m s^-2]
g_vec_e = [0; 0; -g];  % Gravity vector in ECEF     [m s^-2]

m = 10;  % Object mass             [kg]
R = 0.5; % Object spherical radius [m]

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
    I_XX, I_XY, I_XZ;
    I_XY, I_YY, I_YZ;
    I_XZ, I_YZ, I_ZZ;
    ];

% =================
% --- Rotations ---
% =================

C_BE = ecef2body_rotm(e); % ROTM to Body from ECEF
C_EB = C_BE';             % ROTM to ECEF from Body

g_vec_b = C_BE * g_vec_e; % Gravity vector in Body Frame

% ===========================
% --- Equations of Motion ---
% ===========================

F_b = g_vec_b * m; % Body forces [N]
M_b = [0; 0; 0];   % Body moments [N m]

% ===========================
% --- Kinematics ---
% ===========================

[a_b, alpha_b] = particle_model([V_b; w_b], m, I, F_b, M_b);
e_dot   = -1/2 * quat_kinematic_matrix(w_b) * e; % Quaternion rates
V_e     = C_EB * V_b;                            % Velocity in ECEF

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