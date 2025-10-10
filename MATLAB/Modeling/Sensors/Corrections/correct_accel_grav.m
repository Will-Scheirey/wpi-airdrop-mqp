function accel_out = correct_accel_grav(accel_list, e_list)
    
    num_steps = height(accel_list);

    accel_out = zeros(num_steps, 3);

    g_e = [0; 0; -9.8];

    for i = 1:num_steps
        C_BE = ecef2body_rotm(e_list(i, :)');
        g_b = C_BE * g_e;

        accel_out(i, :) = accel_list(i, :) - g_b';
    end
end