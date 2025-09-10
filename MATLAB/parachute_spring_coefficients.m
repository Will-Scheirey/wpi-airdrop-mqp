function [k, c] = parachute_spring_coefficients()    
    n_b = 4;        % Number of bridles
    k_r = 0.8;      % Riser stiffness
    k_b = 0.8;      % Bridle stiffness
    
    m_p = 1;        % Mass of the parachute [kg]
    xi = 0.9;       % Damping ratio
    
    k = (n_b * k_r*k_b) / (k_r + n_b*k_b); % Stiffness
    c = 2*xi*m_p * sqrt(k/m_p);            % Damping coefficient
end