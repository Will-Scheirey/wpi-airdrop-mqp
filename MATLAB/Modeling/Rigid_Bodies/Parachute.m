classdef Parachute < Rigid_Body
    % PARACHUTE A parachute rigidbody with riser and aerodynamic properties
    %  This class assumes a perfectly-rigid, hemispherical parachute

    properties
        % R Parachute Radius
        R
        % A Parachute drag area
        A
        % V Inflated parachute volume
        V
        % L0 Riser rest length
        l0
        % K_RISER Riser spring stiffness
        k_riser
        % C_RISER Riser spring damping coefficient
        c_riser
        % P_ATTACH_B  Attachment point of riser in body frame
        P_attach_B
        % POROSITY Parachute porosity
        porosity
        % ETA Parachute efficiency
        eta 
        % MC Canopy mass
        mc

        % CD0_FLOW Face-on drag coefficient
        Cd0_flow 
        % CD_EDGE Edge-on drag coefficient
        Cd_edge  
        % VARIABLE_MASS Whether to use variable mass
        variable_mass
        % IS_DEPLOYED Whether the parachute is deployed / should have drag
        is_deployed
        % T_DEPLOY The time in flight when the parachute should be deployed
        t_deploy
        % T_CUT The time in flight when the parachute should be cut
        t_cut
    end

    methods
        function obj = Parachute(R, mc, l0, k, c, eta, porosity, drag, variable_mass)
            % PARACHUTE Constructs a new instance of this class
            % 
            % INPUTS:
            %   R             : Parachute radius
            %   mc            : Canopy mass
            %   l0            : Resting riser spring length
            %   k             : Riser spring stiffness
            %   c             : Riser damping coefficient
            %   eta           : Canopy efficiency
            %   porosity      : Canopy porosity
            %   drag          : Whether to use drag
            %   variable_mass : Whether to use variable mass (air density)
            %
            % OUTPUTS:
            %   obj : The new Parachute object

            obj = obj@Rigid_Body();
            obj.R = R;
            % Fabric surface area for a hemisphere
            obj.A = 2*pi*R^2;
            % Hemisphere volume
            obj.V = 2/3*pi*R^3;

            obj.l0 = l0;

            % Relatively arbitrary
            obj.P_attach_B = [R*1.5; 0; 0];

            obj.k_riser = k;
            obj.c_riser = c;

            obj.mc = mc;

            obj.eta = eta;

            % Tuneable
            obj.Cd0_flow = 0.78;
            obj.Cd_edge = 0.2832;

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
                obj.variable_mass = variable_mass;
            else
                obj.variable_mass = false;
            end

            % Default to deployed immediately
            obj.is_deployed = false;
            obj.t_deploy = 0;
            obj.t_cut = inf;
        end

        % --- Overrided functions ---

        function Cd_out = Cd(obj, aoa)
            if obj.is_deployed

                c = cos(aoa);
                s = sin(aoa);
                Cd_out = obj.eta * (obj.Cd0_flow * (s.^2) + obj.Cd_edge * (c.^2));
              
            else
                Cd_out = 0;  % No drag before deployment
            end
        end

        function Cl_out = Cl(obj, aoa)
              Cl_out = obj.Cd(aoa) .* tan(aoa);
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

            % Standard equation (source ?)
            ka = 1.068 * (1.465*p - 0.25975*p^2 + 1.2626*p^3); % Added mass coefficient
            ma_out = ka* rho * obj.V; % Added mass
        end

        function m_out = m(obj, rho)
            if ~obj.variable_mass
                rho = 1.225;
            end

            m_out = obj.mc + obj.added_mass(rho);
        end

        function I_out = I(obj, rho)
            if ~obj.variable_mass
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