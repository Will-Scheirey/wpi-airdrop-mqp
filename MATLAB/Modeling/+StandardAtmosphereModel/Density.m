function rho = Density(h)
    rho = 1.225 * (StandardAtmosphereModel.Temperature(h) / 286.16)^(-1 - 9.8/(-6.5e-03 * 287));
end

