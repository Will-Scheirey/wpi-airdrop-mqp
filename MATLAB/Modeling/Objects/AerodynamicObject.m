classdef AerodynamicObject
    %AERODYNAMICOBJECT Class for an object experiencing aerodynamics
    %   This class defines the most basic physical object that experiences
    %   aerodynamic forces and moments.

    properties
        % --- Mass and Inertia ---
        mass % Object mass           [kg]
        I    % Object inertia tensor [N m]
    end

    methods
        function obj = AerodynamicObject(mass, I)
            obj.mass = mass;
            obj.I = I;
        end
    end

    methods
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