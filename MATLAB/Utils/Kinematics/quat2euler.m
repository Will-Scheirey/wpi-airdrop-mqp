function [psi, theta, phi] = quat2euler(q)
    e0 = q(1);
    e1 = q(2);
    e2 = q(3);
    e3 = q(4);

    % Roll (phi)
    phi = atan2( 2*(e0*e1 + e2*e3), 1 - 2*(e1^2 + e2^2) );

    % Pitch (theta)
    theta = asin( 2*(e0*e2 - e3*e1) );

    % Yaw (psi)
    psi = atan2( 2*(e0*e3 + e1*e2), 1 - 2*(e2^2 + e3^2) );
end
