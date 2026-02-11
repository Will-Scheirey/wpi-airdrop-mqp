function TAS_kt = IAS2TAS(IAS_KIAS, pressureAlt_ft)
% Simple conversion IAS->TAS using barometric formula approx
% TAS ≈ IAS * sqrt(rho0 / rho) ; approximate rho at pressureAlt using ISA
h_m = pressureAlt_ft * 0.3048;
T0 = 288.15; L = 0.0065; R = 287.05; g = 9.80665;
if h_m < 11000
    T = T0 - L*h_m;
    P = 101325 * (T/T0)^(g/(R*L));
else
    T = T0 - L*11000;
    P = 22632 * exp(-g*(h_m-11000)/(R*T));
end
rho = P/(R*T);
rho0 = 1.225;
TAS_kt = IAS_KIAS * sqrt(rho0 / rho);
end
