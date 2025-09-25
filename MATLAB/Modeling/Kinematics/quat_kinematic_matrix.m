function E = quat_kinematic_matrix(pqr)
    p = pqr(1);
    q = pqr(2);
    r = pqr(3);

    E = [  0    p    q    r;
          -p    0   -r    q;
          -q    r    0   -p;
          -r   -q    p    0 ];
end