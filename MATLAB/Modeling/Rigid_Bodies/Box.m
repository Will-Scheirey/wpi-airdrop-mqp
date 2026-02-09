classdef Box < Rigid_Body
    properties
        l
        w
        h
        
        P_attach_B
        Cd_0
    end

    methods
        function obj = Box(l, w, h, m, drag)
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
                    obj.Cd_0 = 0.9;
                else
                    obj.Cd_0 = 0;
                end
            else
                obj.Cd_0 = 0.9;
            end
        end

        function S_out = S(obj, aoa); S_out = obj.l * obj.w; end
        function Cd_out = Cd(obj, ~); Cd_out = obj.Cd_0; end
    end
end