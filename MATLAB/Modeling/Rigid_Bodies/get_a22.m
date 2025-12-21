function a22 = get_a22()
        a22_width  = in2m(48);    % [m]
        a22_length = in2m(83);    % [m]
        a22_height = in2m(43);    % [m]
        a22_mass   = lb2kg(2200); % [kg]
        
        a22 = Box(a22_width, ...
            a22_length, ...
            a22_height, ...
            a22_mass);
end