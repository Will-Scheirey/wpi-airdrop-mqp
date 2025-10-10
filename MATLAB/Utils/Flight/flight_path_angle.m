function gamma = flight_path_angle(C_BE, alpha, beta)
    C_WB = C1_rotm(0) * C2_rotm(alpha) * C3_rotm(-beta);
    C_GW = C1_rotm(0) * C2_rotm(pi/2)  * C3_rotm(0);
    C_WE = C_WB * C_BE;
    C_GE = C_GW * C_WE;
    gamma = asin(-C_GE(1,3)) - pi/2;
end