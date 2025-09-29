function T = ThrustAtAltitude(T_sl, h)
    rho_sl = StandardAtmosphereModel.Density(0);
    rho = StandardAtmosphereModel.Density(h);
    T = T_sl * rho/rho_sl;
end

