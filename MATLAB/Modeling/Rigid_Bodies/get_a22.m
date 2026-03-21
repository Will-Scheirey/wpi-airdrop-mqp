function a22 = get_a22()
% GET_A22 Creates a Box object with the A22 container specifications
% 
% OUTPUTS:
%   a22 : The A22 Box object

% Container specifications
a22_width  = in2m(48);    % [m]
a22_length = in2m(83);    % [m]
a22_height = in2m(43);    % [m]
a22_mass   = lb2kg(2200); % [kg]

% Create the object
a22 = Box(a22_width, ...
    a22_length, ...
    a22_height, ...
    a22_mass);
end