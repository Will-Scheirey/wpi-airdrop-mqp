function x_dot = parachute_dynamic_model(~, x_curr, P0)
% ======================
% --- Current States ---
% ======================

V_b     = x_curr(1:3);   % Body axis velocity          [m   s^-1]
u = V_b(1); v = V_b(2); w = V_b(3);
V = norm(V_b);

w_b     = x_curr(4:6);   % Body angular rates          [rad s^-1]
p = w_b(1); q = w_b(2); r = w_b(3);

e       = x_curr(7:10);  % Orientation quaternion      []
P       = x_curr(11:13); % ECEF Position               [m]
sp      = x_curr(14);    % Parachute distance traveled [m]
V_p     = x_curr(15);    % Parachute velocity          [m   s^-1]


% ==========================
% --- Physical Constants ---
% ==========================
g       = 9.81;       % Gravitational acceleration       [m  s^-2]
% g = 0;
g_vec_e = [0; 0; -g]; % Gravity vector in ECEF           [m  s^-2]

m       = 10;         % Object mass                      [kg]
R       = 0.5;        % Object spherical radius          [m]

S       = pi*R^2;     % Cross-sectional area of a sphere [m^2]
Cd      = 0.07;       % Drag coefficient of a sphere     []

m_p     = 1;          % Mass of the parachute            [kg]
R_p     = 1;          % Parachute inflated radius        [m]
h_p     = 0.5;        % Parachute inflated height        [m]
d       = R*2;        % Parachute diameter               [m]
l_0     = 0.5;        % Parachute offset length          [m]
l_r     = 2;          % Riser length                     [m]

rho     = 1.225;      % Density of air                   [kg m^-3]
% rho = 0;

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

% Flow angle wrt +x_B
if V > eps
    u_flow_B = -V_b./V;
    c = max(-1,min(1, u_flow_B.'*[1;0;0]));
    aoa = acos(c);
else
    aoa = 0;
end
% Angle-dependent Cd using your Parachute_Rigid_Hemi convention
Cd0   = 1.40;       % or expose as parameters
Cedge = 0.05;
Cd    = Cd0*cos(aoa)^2 + Cedge*sin(aoa)^2;

S_p = pi*R_p^2;
D_P = 0.5 * rho * V_p^2 * S_p * Cd;   % replaces parachute_drag(...)

m_a    = parachute_added_mass(rho, R_p, h_p);
[k, c] = parachute_spring_coefficients();

sc     = norm(P - P0);

F_R    = riser_force(sc, sp, l_0, l_r, V, V_p, k, c);

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

% ================================
% --- Aerodynamic Coefficients ---
% ================================
% --- Coefficients of Force ---
C_xq = -0.1;
C_yr = 0;
C_zq = 0;

C_x = -1;
C_y = -1;
C_z = -1;

% --- Coefficients of Moment ---
C_Lp = -0.03;
C_Mq = -0.1;
C_Nr = -0.1;

C_L = -0.1;
C_M = 0.1;
C_N = 0;

% ===========================
% --- Equations of Motion ---
% ===========================

Q = 1/2*rho*V^2; % Dynamic pressure
Q1 = 1/2*rho*V*S;

F_R_b = F_R * [
    cos(alpha)*cos(beta);
    -sin(beta);
    sin(alpha)*cos(beta);
];

F_d = -Q*S*Cd * V_b/max(V, 0.00001);

% --- Body Moments ---
L = Q1 * d*(V*C_L + C_Lp * p*d);
M = Q1 * d*(V*C_M + C_Mq * q*d);
N = Q1 * d*(V*C_N + C_Nr * r*d);

M = [L; M; N];

F_g = C_BE*g_vec_e * m; % Force of gravity

F_b = F_g + F_d + F_R_b; % Body forces [N]
M_b = M;   % Body moments [N m]

sp_dot = V_p; % Parachute velocity
V_p_dot = (F_R - D_P - m_p * g * sin(gamma))/(m_p + m_a); % Parachute acceleration

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

    sp_dot;
    V_p_dot
    ];
end