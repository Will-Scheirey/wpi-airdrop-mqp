function [aoa, sideslip, flight_path] = flight_angles(V_b, C_EB)
    aoa = atan(abs(V_b(3) / V_b(1)));                     % Angle of attack
    sideslip  = asin(abs(V_b(2)/norm(V_b)));              % Side slip angle
    flight_path = flight_path_angle(C_EB, aoa, sideslip); % Flight path angle
end