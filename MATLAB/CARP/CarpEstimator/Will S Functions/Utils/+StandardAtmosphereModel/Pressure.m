function p = Pressure(h)
    p = 101.32 * (StandardAtmosphereModel.Temperature(h) / 286.16)^(-9.8/(-6.5e-03 * 287));
end

