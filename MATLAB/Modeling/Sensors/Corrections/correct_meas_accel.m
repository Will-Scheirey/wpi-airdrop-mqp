function accel_out = correct_meas_accel(a_list, v_list, w_list, e_list, alpha_list, r_s_b)
    % CORRECT_ACCEL_GRAV Converts acceleration to specific force
    %   This function subtracts the gravity vector from body accelerations
    %   and accounts for rotational effects to convert them into specific 
    %   force
    %
    % INPUTS:
    %   a_list : List of raw body accelerations
    %   v_list : List of body velocities
    %   w_list : List of angular velocities
    %   e_list : List of orientation quaternions
    %   alpha_list : List of body angular accelerations
    %   r_s_b : (Optional) Sensor position relative to body COM
    %
    % OUTPUTS:
    %   accel_out : Specific force

    % Make r_s_b optional
    if nargin < 6, r_s_b = [0;0;0]; end

    n = size(a_list,1);
    accel_out = zeros(n,3);
    g_e = [0;0;-9.81];

    % Apply corrections
    for i = 1:n
        C_BE = body2enu_rotm(e_list(i,:)');
        g_b  = C_BE * g_e;

        v_dot_b = a_list(i,:)';
        v_b     = v_list(i, :)';
        w_b     = w_list(i,:)';
        alpha_b = alpha_list(i,:)';
        
        a_b = v_dot_b ...
            + cross(w_b, v_b) ...
            - g_b ...
            + cross(alpha_b, r_s_b) ...
            + cross(w_b, cross(w_b, r_s_b));

        accel_out(i,:) = a_b';
    end
end