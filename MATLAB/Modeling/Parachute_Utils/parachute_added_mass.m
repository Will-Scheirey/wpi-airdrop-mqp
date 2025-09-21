function m_a = parachute_added_mass(rho, R_p, h_p)
    porosity = 0.2; % Parachute porosity    
    n_p = 1;        % Number of parachutes
    
    k_a = 1.068 * (1.465*porosity - 0.25975*porosity^2 + 1.2626*porosity^3); % Added mass coefficient
    m_a = n_p*k_a* rho*4/3*pi*R_p^2 * h_p; % Added mass
end