function accel_out = correct_meas_accel(accel_list, w_list, e_list, alpha_list, r_s_b)

    if nargin < 5
        r_s_b = [0; 0; 0];
    end

    accel_grav_corr = correct_accel_grav(accel_list, e_list);
    accel_rot_corr  = correct_accel_rot(accel_grav_corr, w_list, alpha_list, r_s_b);

    accel_out = accel_rot_corr;
end