classdef Parachute_Rigid_Hemi < AerodynamicObject
    %PARACHUTE_RIGID_HEMI Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R
        A
        V
        l0
        P_attach_B
    end

    methods
        function obj = Parachute_Rigid_Hemi(R, mc, rho, l0)
            V = 2/3*pi*R^3;
            m = mc + V*rho;

            I_XX = 83/320*m*R^2; % Moment of inertia of a hemisphere [kg m^2]
            I_YY = I_XX;         % Moment of inertia of a hemisphere [kg m^2]
            I_ZZ = 2/5*m*R^2;    % Moment of inertia of a hemisphere [kg m^2]
            
            I_XY = 0;         % Moment of inertia of a hemisphere [kg m^2]
            I_XZ = 0;         % Moment of inertia of a hemisphere [kg m^2]
            I_YZ = 0;         % Moment of inertia of a hemisphere [kg m^2]
            
            % Inertia Tensor
            I = [
             I_XX, -I_XY, -I_XZ;
            -I_XY,  I_YY, -I_YZ;
            -I_XZ, -I_YZ,  I_ZZ;
            ];

            obj = obj@AerodynamicObject(m, I);
            obj.R = R;
            obj.A = pi*R^2;
            obj.V = V;
            obj.l0 = l0;

            obj.P_attach_B = [R*1.5; 0; 0];
        end

        function Cd_out = Cd(~, ~); Cd_out = 0.78; end
        function S_out = S(obj, ~); S_out = obj.A; end
    end
end