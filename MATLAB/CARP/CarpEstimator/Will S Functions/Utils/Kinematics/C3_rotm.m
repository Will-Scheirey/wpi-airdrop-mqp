function rotm = C3_rotm(psi)
    rotm = [
      cos(psi),  sin(psi), 0;
      -sin(psi), cos(psi), 0;
      0,         0,        1
    ];
end