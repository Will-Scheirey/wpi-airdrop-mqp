function eul_dot = body_w_to_ecef(phi, theta, w_p)
% --- Euler-angle kinematics: convert body rates to Euler rates
    % Avoid singularity near cos(theta) = 0
    ct = cos(theta); st = sin(theta);
    sp = sin(phi); cp = cos(phi);
    
    if abs(ct) < 1e-6
        % handle near-singularity: you can saturate or use alternative representation
        % warning('theta near +-pi/2: Euler kinematics near singularity');
        ct = sign(ct)*1e-6;
    end
    
    % Transform body rates [p;q;r] to Euler rates [phi_dot;theta_dot;psi_dot]
    T = [ 1, sp.*tan(theta),   cp.*tan(theta);
          0,        cp,           -sp;
          0, sp./ct,        cp./ct ];
    
    eul_dot = T * w_p;   % [phi_dot; theta_dot; psi_dot]
end