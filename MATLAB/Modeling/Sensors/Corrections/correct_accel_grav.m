function accel_out = correct_accel_grav(accel_list, e_list)
    % CORRECT_ACCEL_GRAV Converts acceleration to specific force
    %   This function subtracts the gravity vector from body accelerations
    %   to convert them into specific force
    %
    % INPUTS:
    %   accel_list : List of raw body accelerations
    %   e_list     : List of orientation quaternions
    %
    % OUTPUTS:
    %   accel_out : Specific force
    
    num_steps = height(accel_list);
    accel_out = zeros(num_steps, 3);

    % Gravity vector in ENU
    g_e = [0; 0; -9.8];

    for i = 1:num_steps
        % Rotate the gravity vector into the body frame
        C_BE = body2enu_rotm(e_list(i, :)');
        g_b = C_BE * g_e;

        % Subtract gravity
        accel_out(i, :) = accel_list(i, :) - g_b';
    end
end