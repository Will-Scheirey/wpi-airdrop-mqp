% MATLAB/Modeling/Parachute_Utils/parachute_drag_dynamic.m
function F_D_B = parachute_drag_dynamic(parachute, rho, V_b)
% parachute: Parachute_Rigid_Hemi object
% rho: air density at current altitude [kg/m^3]
% V_b: body-axis velocity of canopy/payload [m/s]
    V = norm(V_b);
    if V < eps
        F_D_B = [0;0;0];
        return
    end
    aoa = parachute_flow_angle(V_b);
    Cd  = parachute.Cd(aoa);
    S   = parachute.S(aoa);    
    q   = 0.5*rho*V*V;
    u_flow_B = -V_b./V;
    F_D_B = q*Cd*S * u_flow_B; % drag opposes velocity
end
