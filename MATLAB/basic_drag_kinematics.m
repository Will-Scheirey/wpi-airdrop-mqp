clear; clc; close all;
addpath("Utils")

% ==========================
% --- Initial Conditions ---
% ==========================

initial_angle = pi/6;
quat = quaternion([cos(initial_angle), 0, -sin(initial_angle), 0]);

q0   = compact(quat)';   % Quaternion parts
V_b0 = [0; 0; 0];        % Body velocities    [m   s^-1]
w_b0 = [-0.3; 0.3; 0.8]; % Body angular rates [rad s^-1]
P0   = [0; 0; 1000];     % ECEF Position      [m]

x0   = [
    V_b0;

    w_b0;

    q0;

    P0
    ];


[t, y] = ode45(@(t, y) particle_model(t, y), 0:0.1:20, x0);

%% Plotting

figure(1)
clf
numsteps = height(y);

patch = poseplot(quaternion(y(1, 7), y(1, 8), y(1, 9), y(1, 10)));

patch.ScaleFactor = 50;
xlabel("X")
ylabel("Y")
zlabel("Z")

for i = 2:1:numsteps
    quat = quaternion(y(i, 7), y(i, 8), y(i, 9), y(i, 10));
    pos = [y(i, 11), y(i, 12), y(i, 13)];

    set(patch, Orientation=quat, Position=pos); hold on
    plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 10); hold on

    xlim([-200, 400]*2);
    ylim([-400, 200]*2);
    zlim([0, 1200]);

    set(gca,'ZDir','normal')
    title(sprintf("t = %0.2f", t(i)))
    drawnow
    pause(0.05)
end

figure(2)
plot(t, y(:, 1), 'DisplayName', 'X'); hold on;
plot(t, y(:, 2), 'DisplayName', 'Y');
plot(t, y(:, 3), 'DisplayName', 'Z');

plot(t, vecnorm(y(:,1:3), 2, 2), 'LineWidth', 3, 'DisplayName', 'Speed')

legend;
title("Body Frame Velocity vs. Time");

function x_dot = particle_model(~, x_curr)
% ======================
% --- Current States ---
% ======================

V_b     = x_curr(1:3);   % Body axis velocity     [m   s^-1]
w_b     = x_curr(4:6);   % Body angular rates     [rad s^-1]
e       = x_curr(7:10);  % Orientation quaternion []
P       = x_curr(11:13); % ECEF Position          [m]

% ==========================
% --- Physical Constants ---
% ==========================
g = 9.81;              % Gravitational acceleration [m s^-2]
g_vec_e = [0; 0; -g];  % Gravity vector in ECEF     [m s^-2]

m = 10;  % Object mass             [kg]
R = 0.5; % Object spherical radius [m]

S = 4*pi*R^2; % Cross-sectional area of a sphere
Cd = 0.07;    % Drag coefficient of a sphere

rho = 1.225;  % Density of air % [kg m^-3]

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

V = norm(V_b); % Speed of object;
Q = 1/2*rho*V^2; % Aerodynamic pressure;

F_d = -Q*Cd * V_b/max(V, 0.00001);

F_g = g_vec_b * m; % Force of gravity

F_b = F_g + F_d; % Body forces [N]
M_b = [0; 0; 0];   % Body moments [N m]

% ===========================
% --- Kinematics ---
% ===========================

a_b     = 1/m * (F_b) - cross(w_b, V_b);         % Body accelerations
alpha_b = I \ (M_b - cross(w_b, I*w_b));         % Angular accelerations
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