function F_s1 = spring_force(obj1, obj2, spring, cable)
    % obj.V, obj.omega, obj.P_attach (absolute), obj.P_attach_rel (COM->attach)
    % All vectors must be expressed in the same frame (here: ECEF)
    
    % --- Velocity of Attachment Points (use relative vector for cross) ---
    V_attach_1 = obj1.V + cross(obj1.omega, obj1.P_attach_rel);
    V_attach_2 = obj2.V + cross(obj2.omega, obj2.P_attach_rel);
    
    % --- Vector from B2 Attachment to B1 Attachment ---
    r_vec = obj1.P_attach - obj2.P_attach;
    r = norm(r_vec);
    if r < 1e-9
        F_s1 = [0;0;0];
        return
    end
    e_r = r_vec / r;
    
    % Relative radial speed
    v_vec = V_attach_1 - V_attach_2;
    v_radial = dot(v_vec, e_r);
    
    extension = r - spring.l0;
    
    % IMPORTANT: slack cable transmits no force (spring OR damper)
    if nargin == 4 && cable && extension <= 0
        F_s1 = [0;0;0];
        return
    end
    
    % If taut (extension > 0), apply spring+damper, but cable cannot push
    tension = spring.k * extension + spring.c * v_radial;
    
    if nargin == 4 && cable
        tension = max(tension, 0);
    end
    
    F_s1 = -tension * e_r;
end 