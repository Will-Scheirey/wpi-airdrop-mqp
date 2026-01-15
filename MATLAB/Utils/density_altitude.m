function DA = density_altitude(P, T)

    P0    = 101325;    % Sea level pressure         [Pa]
    T0    = 288.15;    % Sea level temperature      [K]
    Gamma = 0.0065;    % Temperature lapse rate     [K/m]
    R     = 8.3144598; % Ideal Gas Constant         [J mol^-1 k^-1]
    g     = 9.80665;   % Gravitational acceleration [m s^-2]
    M     = 0.028964;  % Dry air molar mass         [kg/mol]

    r = (P .* T0) ./ (P0 .* T);
    n = (g * M) / (Gamma * R) - 1;

    theta = r .^ (1 ./ n);

    DA = (T0 ./ Gamma) .* (1 - theta);
end