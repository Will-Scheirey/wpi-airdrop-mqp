function [a_b, alpha_b] = particle_model(x, m, I, F_b, M_b)

    V_b = x(1:3); % Body axis velocity [m   s^-1]
    w_b = x(4:6); % Body angular rates [rad s^-1]

    a_b     = F_b/m - cross(w_b, V_b); % Body accelerations
    alpha_b = I \ (M_b - cross(w_b, I*w_b)); % Angular accelerations
end