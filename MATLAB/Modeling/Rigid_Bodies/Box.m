classdef Box < Rigid_Body
    % BOX A box rigidbody with a specified riser attachment point

    properties
        % L Length (x)
        l
        % W Width  (y)
        w
        % H Height (z)
        h
        
        % P_ATTACH_B Attachment point of the riser in the body frame
        P_attach_B

        % CD_0 Flow-on drag coefficient (at 0 AOA)
        Cd_0
    end

    methods
        function obj = Box(l, w, h, m, drag)
            % BOX Constructs a new instance of this class
            % 
            % INPUTS:
            %   l    : length
            %   w    : width
            %   h    : height
            %   m    : mass
            %   drag : Whether to use drag
            %
            % OUTPUTS:
            %   obj : The new Box object

            % Standard moment of inertia of a uniform box
            I = 1/12 * m * [
                w^2 + h^2, 0,         0;
                0,         l^2 + h^2, 0;
                0          0,         l^2 + w^2
            ];

            obj = obj@Rigid_Body(m, I);

            obj.l = l;
            obj.w = w;
            obj.h = h;

            obj.P_attach_B = [h; 0; 0];

            if nargin == 5
                if drag
                    obj.Cd_0 = 0.9; % Rough approx.
                else
                    obj.Cd_0 = 0;
                end
            else
                obj.Cd_0 = 0.9;
            end
        end

        % --- Overrided Functions ---

        % Assume face-on flow
        function S_out = S(obj, aoa); S_out = obj.l * obj.w; end

        % Assume constant Cd
        function Cd_out = Cd(obj, ~); Cd_out = obj.Cd_0; end
    end
end