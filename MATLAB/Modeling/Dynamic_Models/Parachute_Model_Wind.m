classdef Parachute_Model_Wind < Parachute_Model_Simple
    %PARACHUTE_MODEL_WIND Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods
        function obj = Parachute_Model_Wind(payload, parachute, x0)
            %PARACHUTE_MODEL_SIMPLE Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Parachute_Model_Simple(payload, parachute, x0);

            obj.parachute = parachute;
            obj.payload   = payload;
        end

        function get_states(obj, x)
            % ======================
            % --- Current States ---
            % ======================

            % if (x(3) >  2000)
            %     worc_wind = [10, 270]; % Average wind data in Worcester, MA ([mph], [deg])
            % elseif (x(3) > 1000)
            %     worc_wind = [6, 250]; % Average wind data in Worcester, MA ([mph], [deg])
            %     disp("Switch");
            % else
            %     worc_wind = [3, 210]; % Average wind data in Worcester, MA ([mph], [deg])
            %     disp("Switch")
            % end

            worc_wind = [10,0]; % [m s]

            mag = worc_wind(1); dir = worc_wind(2);

            e = mag*sind(dir);
            n = mag*cosd(dir);

            wind = [e; n; 0];              % Wind,             ECEF [m   s]

            % --- Payload ---
            obj.e_p = x(7:10) / norm(x(7:10));   % Orientation,      ECEF

            obj.P_p   = x(1:3);            % Position,         ECEF [m]
            wind = ecef2body_rotm(obj.e_p)*wind;
            obj.V_p = wind + x(4:6);             % Velocity,         Body [m   s^-1]

            obj.w_p = x(11:13);            % Angular Velocity, Body [rad s^-1]

            % --- Parachute ---
            obj.e_c = x(20:23) / norm(x(20:23)); % Orientation,      ECEF

            obj.P_c = x(14:16);            % Position,         ECEF [m]
            wind = ecef2body_rotm(obj.e_c)*wind;
            obj.V_c = wind + x(17:19);           % Velocity,         Body [m   s^-1]

            obj.w_c = x(24:26);            % Angular Velocity, Body [rad s^-1]

            % --- Additional States ---

            obj.rho = StandardAtmosphereModel.Density(obj.P_p(3)); % Density of air  [kg m^-3]

            obj.m_payload   = obj.payload.  m(obj.rho);
            obj.m_parachute = obj.parachute.m(obj.rho);

            % =================
            % --- Rotations ---
            % =================

            obj.C_EB_p   = ecef2body_rotm(obj.e_p);                 % ROTM from ECEF to Body
            obj.C_EB_c   = ecef2body_rotm(obj.e_c);                 % ROTM from ECEF to Body
        end

        function [F_p, F_c, M_p, M_c] = equations_of_motion(obj)
            % ===========================
            % --- Equations of Motion ---
            % ===========================
            % --- Body Forces ---

            [F_g_p, F_g_c] = obj.calc_gravity();
            [F_d_p, F_d_c] = obj.calc_drag();
            [F_r_p, F_r_c] = obj.calc_riser_force();
            % [F_w_p, F_w_c] = obj.calc_wind_force();

            % --- Body Moments ---

            F_p = F_g_p + F_d_p + F_r_p; % Payload body forces [N]
            F_c = F_g_c + F_d_c + F_r_c; % Parachute body forces [N]
            % F_p = F_g_p + F_d_p + F_r_p + F_w_p; % Payload body forces [N]
            % F_c = F_g_c + F_d_c + F_r_c + F_w_c; % Parachute body forces [N]

            M_p = cross(obj.payload.  P_attach_B, F_r_p); % Payload body moments   [N m]
            M_c = cross(obj.parachute.P_attach_B, F_r_c); % Parachute body moments [N m]
        end

        function [f_p, f_c] = calc_drag(obj)
            aoa_p = flight_angles(obj.V_p, obj.C_EB_p);
            aoa_c = flight_angles(obj.V_c, obj.C_EB_c);

            f_p = -0.5 * obj.rho * obj.payload.CdS(aoa_p)   * obj.V_p * norm(obj.V_p);
            f_c = -0.5 * obj.rho * obj.parachute.CdS(aoa_c) * obj.V_c * norm(obj.V_c);

            if any(isnan(f_p))
                f_p = [0; 0; 0];
            end
            if any(isnan(f_c))
                f_c = [0; 0; 0];
            end

            obj.aoa_p_curr = aoa_p;
            obj.aoa_c_curr = aoa_c;

            obj.drag_force_p = f_p;
            obj.drag_force_c = f_c;
        end
    end
end