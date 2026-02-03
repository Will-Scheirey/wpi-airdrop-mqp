function q = euler2quat(psi, theta, phi)
    e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
    e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
    e2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
    e3 = sin(psi/2)*cos(theta/2)*cos(phi/2) + cos(psi/2)*sin(theta/2)*sin(phi/2);

    q = [e0; e1; e2; e3];
end