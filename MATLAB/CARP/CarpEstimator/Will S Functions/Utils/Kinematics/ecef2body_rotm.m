function rotm = ecef2body_rotm(e)
    e0=e(1); e1=e(2); e2=e(3); e3=e(4);
    rotm = [
        e0^2 + e1^2 - e2^2 - e3^2, 2*(e1*e2 + e0*e3),         2*(e1*e3 - e0*e2);
        2*(e1*e2 - e0*e3),         e0^2 - e1^2 + e2^2 - e3^2, 2*(e2*e3 + e0*e1);
        2*(e0*e2 + e1*e3),         2*(e2*e3 - e0*e1),         e0^2 - e1^2 - e2^2 + e3^2
    ];
end