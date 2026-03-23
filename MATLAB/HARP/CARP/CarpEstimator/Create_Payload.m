% CREATE_PAYLOAD Instantiate a Box payload object from imperial dimensions and mass.
%   Converts payload dimensions from inches to meters and mass from pounds
%   to kilograms, then constructs a Box object for use in the propagator.
%
% INPUTS:
%   w : Payload width (inches)
%   l : Payload length (inches)
%   h : Payload height (inches)
%   m : Payload mass (lbs)
%
% OUTPUTS:
%   payload : Box object with dimensions in meters and mass in kilograms,
%             with drag enabled (true)
%

function payload = Create_Payload(w, l, h, m)
    
    payload = Box(in2m(w), in2m(l), in2m(h), lb2kg(m), true);

end