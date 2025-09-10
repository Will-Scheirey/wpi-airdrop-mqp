function x_dot = dynamic_model(t, x_curr)

    u     = x_curr(1); % Body axis velocity in X direction
    v     = x_curr(2); % Body axis velocity in Y direction
    w     = x_curr(3); % Body axis velocity in Z direction

    p     = x_curr(4); % Body axis roll
    q     = x_curr(5); % Body axis pitch
    r     = x_curr(6); % Body axis yaw

    e0    = x_curr(7); % Quaternion 0
    e1    = x_curr(8); % Quaternion 1
    e2    = x_curr(9); % Quaternion 2
    e3    = x_curr(10); % Quaternion 3

    e_norm = norm([e0, e1, e2, e3]);

    e0 = e0 / e_norm;
    e1 = e1 / e_norm;
    e2 = e2 / e_norm;
    e3 = e3 / e_norm;

    x     = x_curr(11); % ECEF X position
    y     = x_curr(12); % ECEF Y position
    h     = x_curr(13); % ECEF Z position

    sp   = x_curr(14); % Parachute absolute displacement
    Vp   = x_curr(15); % Parachute canopy total velocity

    % --- Constants ---
    g = 9.81; % Gravitational acceleration [m s^-2]
    m = 1e2; % Payload mass [kg]
    l = 1; % Payload cube side length [m]
    d = l/2;
    l0 = 0.5;

    S = 0.8*l^2;

    % --- Preliminary Calculations ---
    V = norm([u, v, w]);    % Velocity magnitude
    alpha = atan(abs(w/u)); % Angle of attack
    beta = asin(abs(v/V));  % Side slip angle
   
    C_BE = ecef2body_rotm([e0, e1, e2, e3]);

    C_WB = C1_rotm(0) * C2_rotm(alpha) * C3_rotm(-beta);
    C_BW = inv(C_WB);
    C_GW = C1_rotm(0) * C2_rotm(pi/2)  * C3_rotm(0);
    C_WE = C_WB * C_BE;
    C_GE = C_GW * C_WE;
    gamma = asin(-C_GE(1,3)) - pi/2;

    rho = 1.225; % CALCULATE BASED ON ALTITUDE!

    % --- Riser Force ---

    l_r = 2; % Riser length [m]

    n_b = 4;   % Number of bridles
    k_r = 0.8; % Riser stiffness
    k_b = 0.8; % Bridle stiffness

    m_p = 1;  % Mass of the parachute [kg]
    xi = 0.9;  % Damping ratio
    porosity = 0.2;    % Parachute porosity
    n = 1;     % Parachute efficiency

    R = 1; % Parachute inflated radius [m]
    height = 1; % Parachute inflated height [m]

    n_p = 1;   % Number of parachutes

    CD_SP_t1 = @(t, ti, t_fi, tau, n_i, CD_SP_0) CD_SP_0 * tau * ((t - t_i)/(t_fi))^n_i;
    CD_SP_t2 = @(tau, CD_SP_0) CD_SP_0 * tau;

    CD_SP = CD_SP_t2(4*pi*R*1.8, 2);

    D_P = 1/2 * rho * Vp^2 * CD_SP * n;

    k_a = 1.068 * (1.465*porosity - 0.25975*porosity^2 + 1.2626*porosity^3); % Added mass coefficient
    m_a = n_p*k_a* rho*4/3*pi*R^2 * height; % Added mass

    k = (n_b * k_r*k_b) / (k_r + n_b*k_b); % Stiffness
    c = 2*xi*m_p * sqrt(k/m_p); % Damping coefficient

    sc = sqrt(x^2 + y^2 + (h - 1000)^2);

    eps = (sc - sp - l0)/l_r;
    eps_dot = V - Vp;

    F_R =  k*l_r*eps + c*eps_dot; % Force in the riser
    
    % --- Moments of Inertia ---
    I_XX = 1/6*m*l^2;
    I_YY = 1/6*m*l^2;
    I_ZZ = 1/6*m*l^2;

    % --- Coefficients of Force ---
    C_xq = 0;
    C_yr = 0;
    C_zq = 0;

    C_x = 0;
    C_y = 0;
    C_z = 0;

    % --- Coefficients of Moment ---
    C_Lp = -0.1;
    C_Mq = -0.1;
    C_Nr = -0.1;

    C_L = 0;
    C_M = 0;
    C_N = 0;

    % --- Body Forces ---
    Q = 1/2*rho*V*S; % Dynamic pressure

    X = Q * (V*C_x + C_xq * q*d) - F_R * cos(alpha)*cos(beta);
    Y = Q * (V*C_y + C_yr * r*d) - F_R *            sin(beta);
    Z = Q * (V*C_z + C_zq * q*d) - F_R * sin(alpha)*cos(beta);

    % --- Body Moments ---
    L = Q * q*(V*C_L + C_Lp * p*d);
    M = Q * q*(V*C_M + C_Mq * q*d);
    N = Q * q*(V*C_N + C_Nr * r*d);

    % --- States ---
    u_dot = (X + m*g*C_BE(3,1))/m - q*w + r*v;
    v_dot = (Y + m*g*C_BE(3,2))/m - r*u + p*w;
    w_dot = (Z + m*g*C_BE(3,3))/m + q*u - p*v;

    p_dot = L/I_XX;
    q_dot = M/I_YY;
    r_dot = N/I_ZZ;

    % --- Quaternion Rates ---
    e0_dot = -1/2 * (e1*p + e2*q + e3*r);
    e1_dot = 1/2 *  (e0*p - e3*q + e2*r);
    e2_dot = 1/2 *  (e3*p + e0*q - e1*r);
    e3_dot = -1/2 * (e2*p - e1*q - e0*r);
    % ---^^ Checked ^^---

    x_dot =    C_BE(1,1)*u + C_BE(1,2)*v + C_BE(1,3)*w;
    y_dot =    C_BE(2,1)*u + C_BE(2,2)*v + C_BE(2,3)*w;
    h_dot =  -(C_BE(3,1)*u + C_BE(3,2)*v + C_BE(3,3)*w);

    sp_dot = Vp;
    
    Vp_dot = (F_R - D_P - m_p * g * sin(gamma))/(m_p + m_a);

    x_dot = [
      u_dot;
      v_dot;
      w_dot;

      p_dot;
      q_dot;
      r_dot;

      e0_dot;
      e1_dot;
      e2_dot;
      e3_dot;
      
      x_dot;
      y_dot;
      h_dot;

      sp_dot;
      Vp_dot;
    ];

    if any(imag(x_dot) ~= 0)
        disp("!");
    end
    
end