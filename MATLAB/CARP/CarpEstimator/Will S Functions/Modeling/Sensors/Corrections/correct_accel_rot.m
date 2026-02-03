function accel_out = correct_accel_rot(accel_list, w_list, alpha_list, r_s_b)

    num_steps = height(accel_list);
    accel_out = zeros(num_steps,3);

    for i = 1:num_steps
        w     = w_list(i,:)';
        alpha = alpha_list(i,:)';

        a_meas = accel_list(i,:)' ...
                   + cross(alpha, r_s_b) ...
                   + cross(w, cross(w, r_s_b));

        accel_out(i,:) = a_meas';
    end
end