function rotm = C2_rotm(theta)
    rotm = [
      cos(theta), 0, -sin(theta);
      0,          1,  0;
      sin(theta), 0,  cos(theta)
    ];
end