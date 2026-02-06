classdef Parachute < Rigid_Body
    %PARACHUTE_RIGID_HEMI Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R          % Radius
        A          % Area
        V          % Volume
        l0         % Riser rest length
        k_riser    % Riser spring stiffness
        c_riser    % Riser spring damping coefficient
        P_attach_B % Attachment point of riser in body frame
        porosity   % Parachute porosity
        eta % Parachute efficiency

        mc         % Canopy mass

        Cd_0
        variable_ma
        is_deployed    % NEW: deployment flag
        t_deploy
    end

    methods
        function obj = Parachute(R, mc, l0, k, c, eta, porosity, drag, variable_ma)
            obj = obj@Rigid_Body();
            obj.R = R;
            obj.A = pi*R^2;
            obj.V = 2/3*pi*R^3;
            obj.l0 = l0;

            obj.P_attach_B = [R*1.5; 0; 0];

            obj.k_riser = k;
            obj.c_riser = c;

            obj.mc = mc;

            obj.eta = eta;

            if nargin >= 7
                obj.porosity = porosity;
            else
                obj.porosity = 1;
            end

            if nargin >= 8
                if drag
                    obj.Cd_0 = 0.78;
                else
                    obj.Cd_0 = 0;
                end
            else
                obj.Cd_0 = 0.78;
            end

            if nargin >= 9
                obj.variable_ma = variable_ma;
            else
                obj.variable_ma = false;
            end
            % Default to deployed immediately
            obj.is_deployed = false;
            obj.t_deploy = 0;
        end

        function Cd_out = Cd(obj, ~)
            if obj.is_deployed
                Cd_out = obj.Cd_0 * obj.eta;
            else
                Cd_out = 0;  % No drag before deployment
            end
        end

        function S_out = S(obj, ~)
            if obj.is_deployed
                S_out = obj.A;
            else
                S_out = 0;  % No area before deployment
            end
        end

        function ma_out = added_mass(obj, rho)
            p = obj.porosity;

            ka = 1.068 * (1.465*p - 0.25975*p^2 + 1.2626*p^3); % Added mass coefficient
            ma_out = ka* rho * obj.V; % Added mass
        end

        function m_out = m(obj, rho)
            if ~obj.variable_ma
                rho = 1.225;
            end

            m_out = obj.mc + obj.added_mass(rho);
        end

        function I_out = I(obj, rho)
            if ~obj.variable_ma
                rho = 1.225;
            end
            m = obj.m(rho);

            I_XX = 83/320*m*obj.R^2; % Moment of inertia of a hemisphere [kg m^2]
            I_YY = I_XX;             % Moment of inertia of a hemisphere [kg m^2]
            I_ZZ = 2/5*m*obj.R^2;    % Moment of inertia of a hemisphere [kg m^2]

            I_XY = 0;         % Moment of inertia of a hemisphere [kg m^2]
            I_XZ = 0;         % Moment of inertia of a hemisphere [kg m^2]
            I_YZ = 0;         % Moment of inertia of a hemisphere [kg m^2]

            % Inertia Tensor
            I_out = [
                I_XX, -I_XY, -I_XZ;
                -I_XY,  I_YY, -I_YZ;
                -I_XZ, -I_YZ,  I_ZZ;
                ];
        end

    end
end