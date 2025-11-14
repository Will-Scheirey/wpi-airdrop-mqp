function aoa = parachute_flow_angle(V_b)
% V_b: body-axis velocity [m/s] of the canopy/payload
% aoa: angle between +x_B and incoming flow direction
    V = norm(V_b);
    if V < eps
        aoa = 0;
        return
    end
    u_flow_B = -V_b./V;        % unit incoming flow direction
    xB       = [1;0;0];
    c = max(-1,min(1, dot(u_flow_B, xB)));
    aoa = acos(c);
end