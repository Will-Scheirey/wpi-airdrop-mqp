function F_s1 = spring_force(obj1, obj2, spring)
    % obj.V, obj.omega, obj.P_attach (absolute), obj.P_attach_rel (COM->attach)
    % All vectors must be expressed in the same frame (here: ECEF)
    
    % --- Velocity of Attachment Points (use relative vector for cross) ---
    V_attach_1 = obj1.V + cross(obj1.omega, obj1.P_attach_rel);
    V_attach_2 = obj2.V + cross(obj2.omega, obj2.P_attach_rel);
    
    % --- Vector from B2 Attachment to B1 Attachment (absolute positions) ---
    r_vec = obj1.P_attach - obj2.P_attach;
    r = norm(r_vec);
    if r < 1e-9
        F_s1 = [0;0;0];
        return
    end
    e_r = r_vec / r;
    
    % extension: choose cable or spring behavior
    extension = max(r - spring.l0, 0); % cable (no compression)
    
    v_vec = V_attach_1 - V_attach_2;
    v_radial = dot(v_vec, e_r);
    
    F_s1 = -(spring.k * extension + spring.c * v_radial) * e_r;
end