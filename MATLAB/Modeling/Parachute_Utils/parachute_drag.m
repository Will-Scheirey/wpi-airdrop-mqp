function D_P = parachute_drag(R_p, rho, Vp)
    CD_SP = pi*R_p^2 * 1.8;
    n = 1;          % Parachute efficiency
    D_P = 1/2 * rho * Vp^2 * CD_SP * n; % Parachute drag
end