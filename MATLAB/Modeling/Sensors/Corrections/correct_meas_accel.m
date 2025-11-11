function accel_out = correct_meas_accel(a_list, v_list, w_list, e_list, alpha_list, r_s_b)
    if nargin < 6, r_s_b = [0;0;0]; end

    n = size(a_list,1);
    accel_out = zeros(n,3);
    g_e = [0;0;-9.81];

    for i = 1:n
        C_BE = ecef2body_rotm(e_list(i,:)');
        g_b  = C_BE * g_e;

        a_dot_b = a_list(i,:)';
        v_b     = v_list(i,:)';
        w_b     = w_list(i,:)';
        alpha_b = alpha_list(i,:)';
        
        a_b = a_dot_b - g_b ...
              + cross(alpha_b, r_s_b) + cross(w_b, cross(w_b, r_s_b));
        accel_out(i,:) = a_b';
    end
end