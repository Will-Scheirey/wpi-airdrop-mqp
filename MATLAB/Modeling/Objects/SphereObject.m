classdef SphereObject < AerodynamicObject
    %SPHEREOBJECT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R
        A
    end

    methods
        function obj = SphereObject(m, R)
            I_XX = 2/5*m*R^2; % Moment of inertia of a sphere [kg m^2]
            I_YY = I_XX;      % Moment of inertia of a sphere [kg m^2]
            I_ZZ = I_XX;      % Moment of inertia of a sphere [kg m^2]
            
            I_XY = 0;         % Moment of inertia of a sphere [kg m^2]
            I_XZ = 0;         % Moment of inertia of a sphere [kg m^2]
            I_YZ = 0;         % Moment of inertia of a sphere [kg m^2]
            
            % Inertia Tensor
            I = [
             I_XX, -I_XY, -I_XZ;
            -I_XY,  I_YY, -I_YZ;
            -I_XZ, -I_YZ,  I_ZZ;
            ];
            obj = obj@AerodynamicObject(m, I);
            obj.R = R;
            obj.A = pi*R^2;
        end

        function S_out = S(obj, ~); S_out = obj.A; end
        function Cd_out = Cd(~, ~); Cd_out = 0.;5; end
    end
end