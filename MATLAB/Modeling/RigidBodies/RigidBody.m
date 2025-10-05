classdef RigidBody
    %AERODYNAMICOBJECT Class for a rigidbody experiencing aerodynamics
    %   This class defines the most basic physical object that experiences
    %   aerodynamic forces and moments.

    properties
        % --- Mass and Inertia ---
        mass % Object mass           [kg]
        inertia_tensor    % Object inertia tensor [N m]
    end

    methods
        function obj = RigidBody(mass, I)
            if nargin < 2
                obj.mass = 0;
                obj.inertia_tensor = 0;
            else
                obj.mass = mass;
                obj.inertia_tensor = I;
            end
        end
    end

    methods
        % --- Basic Properties ---
        function m_out = m(obj, rho); m_out = obj.mass; end
        function I_out = I(obj, rho); I_out = obj.inertia_tensor; end
        
        % --- Basic Aerodynamic Properties ---
        function S_out = S(obj, aoa); S_out = 0; end
        function Cd_out = Cd(obj, aoa); Cd_out = 0; end
        function Cl_out = Cl(obj, aoa); Cl_out = 0; end
        function ClS_out = ClS(obj, aoa); ClS_out = obj.S(aoa)*obj.Cl(aoa); end
        function CdS_out = CdS(obj, aoa); CdS_out = obj.S(aoa)*obj.Cd(aoa); end

        % --- Aerodynamic Coefficients of Force ---
        function C_out = C_xq(obj, aoa); C_out = 0; end
        function C_out = C_yr(obj, aoa); C_out = 0; end
        function C_out = C_zq(obj, aoa); C_out = 0; end
        
        function C_out = C_x(obj, aoa); C_out = 0; end
        function C_out = C_y(obj, aoa); C_out = 0; end
        function C_out = C_z(obj, aoa); C_out = 0; end
        
        % --- Aerodynamic Coefficients of Moment ---
        function C_out = C_Lp(obj, aoa); C_out = 0; end
        function C_out = C_Mq(obj, aoa); C_out = 0; end
        function C_out = C_Nr(obj, aoa); C_out = 0; end
        
        function C_out = C_L(obj, aoa); C_out = 0; end
        function C_out = C_M(obj, aoa); C_out = 0; end
        function C_out = C_N(obj, aoa); C_out = 0; end
    end
end