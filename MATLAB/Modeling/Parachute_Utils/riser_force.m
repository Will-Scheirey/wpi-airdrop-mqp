function F_R = riser_force(sc, sp, l0, l_r, V, Vp, k, c)
    eps = (sc - sp - l0)/l_r;
    eps_dot = V - Vp;
    
    F_R =  k*l_r*eps + c*eps_dot; % Force in the riser
end