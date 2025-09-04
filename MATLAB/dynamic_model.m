function x_dot = dynamic_model(t, x_curr)

    u     = x_curr(1); % Body axis velocity in X direction
    v     = x_curr(2); % Body axis velocity in Y direction
    w     = x_curr(3); % Body axis velocity in Z direction

    p     = x_curr(4); % Body axis roll
    q     = x_curr(5); % Body axis pitch
    r     = x_curr(6); % Body axis yaw

    p_dot = x_curr(7); % Body axis roll rate
    q_dot = x_curr(8); % Body axis pitch rate
    r_dot = x_curr(9);%  Body axis yaw rate

    e0    = x_curr(10); % Quaternion 0
    e1    = x_curr(11); % Quaternion 1
    e2    = x_curr(12); % Quaternion 2
    e3    = x_curr(13); % Quaternion 3

    e_norm = norm([e0, e1, e2, e3]);

    e0 = e0 / e_norm;
    e1 = e1 / e_norm;
    e2 = e2 / e_norm;
    e3 = e3 / e_norm;

    x     = x_curr(14); % ECEF X position
    y     = x_curr(15); % ECEF Y position
    h     = x_curr(16); % ECEF Z position

    sp   = x_curr(17); % Parachute absolute displacement
    Vp   = x_curr(18); % Parachute canopy total velocity

    % --- Constants ---
    g = 9.81; % Gravitational acceleration [m s^-2]
    m = 1e2; % Payload mass [kg]
    l = 0.1; % Payload cube side length [m]
    d = l;

    S = 0.8*l^2;

    % --- Preliminary Calculations ---
    V = norm([u, v, w]);    % Velocity magnitude
    alpha = atan(abs(w/u)); % Angle of attack
    beta = asin(abs(v/V));  % Side slip angle
   
    C_BE = ecef2body_rotm(e0, e1, e2, e3);

    C_WB = C1_rotm(0) * C2_rotm(alpha) * C3_rotm(-beta);
    C_BW = inv(C_WB);
    C_GW = C1_rotm(0) * C2_rotm(pi/2)  * C3_rotm(0);
    C_WE = C_WB * C_BE;
    C_GE = C_GW * C_WE;
    gamma = asin(-C_GE(1,3)) - pi/2;

    rho = 1.225; % CALCULATE BASED ON ALTITUDE!

    % --- Riser Force ---

    l_r = 10; % Riser length [m]

    n_b = 4;   % Number of bridles
    k_r = 0.5; % Riser stiffness
    k_b = 0.3; % Bridle stiffness

    m_p = 10;  % Mass of the parachute [kg]
    xi = 0.8;  % Damping ratio
    p = 0.2;    % Parachute porosity
    n = 1;     % Parachute efficiency

    R = 5; % Parachute inflated radius [m]
    h = 3; % Parachute inflated height [m]

    n_p = 1;   % Number of parachutes

    CD_SP_t1 = @(t, ti, t_fi, tau, n_i, CD_SP_0) CD_SP_0 * tau * ((t - t_i)/(t_fi))^n_i;
    CD_SP_t2 = @(tau, CD_SP_0) CD_SP_0 * tau;

    CD_SP = CD_SP_t2(1, 30);

    D_P = 1/2 * rho * Vp^2 * CD_SP * n;

    k_a = 1.068 * (1.465*p - 0.25975*p^2 + 1.2626*p^3); % Added mass coefficient
    m_a = n_p*k_a* rho* 4/3*pi*R^2 * h; % Added mass

    k = (n_b * k_r*k_b) / (k_r + n_b*k_b); % Stiffness
    c = 2*xi*m_p * sqrt(k/m_p); % Damping coefficient

    eps = min(10, t / 10);
    eps_dot = 1/10;
    if eps == 10
        eps_dot = 0;
    end

    eps = 0;
    eps_dot = 0;

    F_R =  k*l_r*eps + c*eps_dot; % Force in the riser

    % --- Moments of Inertia ---
    I_XX = 1/6*m*l^2;
    I_YY = 1/6*m*l^2;
    I_ZZ = 1/6*m*l^2;
    
    I_XY = 0;
    I_YZ = 0;
    I_XZ = 0;

    % --- Coefficients of Force ---
    C_xy = 0.03;
    C_yr = 0.03;
    C_zq = 0.03;

    C_x = 0.05;
    C_y = 0.05;
    C_z = 0.05;

    % --- Coefficients of Moment ---
    C_lp = 0.02;
    C_Mr = 0.02;
    C_Nq = 0.02;

    C_l = 0.04;
    C_M = 0.04;
    C_N = 0.04;

    % --- Center of Mass Corrections ---

    x_cg = 0;
    x_p = 0;

    y_cg = 0;
    y_p = 0;

    z_cg = 0;
    z_p = 0;

    % --- Body Forces ---
    Q = 1/2*rho*V*S; % Dynamic pressure

    wind = 1000 * [1, 1, 1] * C_BE'; 

    X = Q * (V*C_x + C_xy * q*d) - F_R * cos(alpha)*cos(beta) + wind(1);
    Y = Q * (V*C_y + C_yr * r*d) - F_R * sin(beta) + wind(2);
    Z = Q * (V*C_z + C_zq * q*d) - F_R * sin(alpha)*cos(beta) + wind(3);

    % --- Body Moments ---
    L = Q * (V*C_l + C_lp * p*d) - F_R * sin(beta) * (z_cg - z_p)*d - F_R * sin(alpha)*sin(beta) * (y_cg - y_p)*d;
    M = Q * (V*C_M + C_Mr * q*d) - F_R * cos(alpha)*sin(beta) * (z_cg - z_p)*d + F_R * sin(alpha)*sin(beta) * (x_cg - x_p)*d;
    N = Q * (V*C_N + C_Nq * r*d) - F_R * sin(beta) * (x_cg - x_p)*d - F_R * cos(alpha)*cos(beta) * (y_cg - y_p)*d;

    % --- States ---
    u_dot = (X + m*g*C_BE(1,3))/m - q*w + r*v;
    v_dot = (Y + m*g*C_BE(2,3))/m - r*u + p*w;
    w_dot = (Z + m*g*C_BE(3,3))/m + q*u - p*v;

    p_dot_new = p_dot;
    q_dot_new = q_dot;
    r_dot_new = r_dot;

    p_ddot = (I_XY*q_dot + I_XZ*r_dot + I_XZ*p*q + (I_YY - I_ZZ)*q*r - I_XY*p*r + (q^2-r^2)*I_YZ + L)/I_XX;
    q_ddot = (I_XY*p_dot + I_YZ*r_dot + I_YZ*p*q + (I_ZZ - I_XX)*p*r + I_XY*q*r + (r^2-p^2)*I_XZ + M)/I_YY;
    r_ddot = (I_XZ*p_dot + I_YZ*q_dot - I_XZ*q*r + (I_XX - I_YY)*p*q + I_YZ*p*r + (p^2-q^2)*I_XY + N)/I_ZZ;

    e0_dot = -1/2 * (e1*p + e2*q + e3*r);
    e1_dot = -1/2 * (e0*p - e3*q + e2*r);
    e2_dot = -1/2 * (e3*p + e0*q - e1*r);
    e3_dot = -1/2 * (e2*p - e1*q - e0*r);

    x_dot =  -(C_BE(1,1)*u + C_BE(1,2)*v + C_BE(1,3)*w);
    y_dot =  C_BE(2,1)*u + C_BE(2,2)*v + C_BE(2,3)*w;
    h_dot =  -(C_BE(3,1)*u + C_BE(3,2)*v + C_BE(3,3)*w);

    sp_dot = Vp;
    
    Vp_dot = (F_R - D_P - m_p * g * sin(gamma))/(m_p + m_a);

    x_dot = [
      u_dot;
      v_dot;
      w_dot;

      p_dot_new;
      q_dot_new;
      r_dot_new;

      p_ddot;
      q_ddot;
      r_ddot;

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