classdef Parachute_Model_Simple < Dynamic_Model
    %PARACHUTE_MODEL_SIMPLE Summary of this class goes here
    %   Detailed explanation goes here

    properties
        parachute
        payload

        m_payload
        m_parachute

        P_p
        P_c

        e_p
        e_c

        V_p
        V_c

        w_p
        w_c

        C_EB_p
        C_EB_c

        rho

        drag_force_p
        drag_force_c

        a_p_curr
        a_c_curr

        alpha_p_curr
        alpha_c_curr

        aoa_p_curr
        aoa_c_curr
    end

    methods
        function obj = Parachute_Model_Simple(payload, parachute, x0)
            %PARACHUTE_MODEL_SIMPLE Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Dynamic_Model(x0);

            obj.parachute = parachute;
            obj.payload   = payload;
        end

        function x_dot = ode_fcn(obj, t, x)
            
            % Check if parachute should deploy
            if t >= obj.parachute.t_deploy && ~obj.parachute.is_deployed
                obj.parachute.is_deployed = true;
                fprintf('*** PARACHUTE DEPLOYED at t = %.2f seconds ***\n', t);
            end

            obj.get_states(x);

            [F_p, F_c, M_p, M_c] = obj.equations_of_motion();

            x_dot = obj.dxdt(F_p, F_c, M_p, M_c);
end

        function [F_p, F_c, M_p, M_c] = equations_of_motion(obj)
            % ===========================
            % --- Equations of Motion ---
            % ===========================
            % --- Body Forces ---

            [F_g_p, F_g_c] = obj.calc_gravity();
            [F_d_p, F_d_c] = obj.calc_drag();
            [F_r_p, F_r_c] = obj.calc_riser_force();

            % --- Body Moments ---

            F_p = F_g_p + F_d_p + F_r_p; % Payload body forces [N]
            F_c = F_g_c + F_d_c + F_r_c; % Parachute body forces [N]

            M_p = cross(obj.payload.  P_attach_B, F_r_p); % Payload body moments   [N m]
            M_c = cross(obj.parachute.P_attach_B, F_r_c); % Parachute body moments [N m]
        end

        function x_dot = dxdt(obj, F_p, F_c, M_p, M_c)

            % ===========================
            % --- Kinematics ---
            % ===========================

            [a_p, alpha_p] = obj.calc_accel(obj.V_p, obj.w_p, obj.m_payload,   obj.payload.I(obj.rho),   F_p, M_p);
            [a_c, alpha_c] = obj.calc_accel(obj.V_c, obj.w_c, obj.m_parachute, obj.parachute.I(obj.rho), F_c, M_c);

            obj.a_p_curr = a_p;
            obj.a_c_curr = a_c;

            obj.alpha_p_curr = alpha_p;
            obj.alpha_c_curr = alpha_c;

            e_p_dot   = -1/2 * quat_kinematic_matrix(obj.w_p) * obj.e_p; % Quaternion rates
            e_c_dot   = -1/2 * quat_kinematic_matrix(obj.w_c) * obj.e_c; % Quaternion rates

            % ==============
            % --- States ---
            % ==============
            x_dot = [
                obj.C_EB_p' * obj.V_p;
                a_p;

                e_p_dot
                alpha_p;

                obj.C_EB_c' * obj.V_c;
                a_c;

                e_c_dot;
                alpha_c;
                ];
        end
            
        function get_states(obj, x)
            % ======================
            % --- Current States ---
            % ======================

            % --- Payload ---
            obj.P_p   = x(1:3);            % Position,         ECEF [m]
            obj.V_p = x(4:6);              % Velocity,         Body [m   s^-1]

            obj.e_p = x(7:10) / norm(x(7:10));   % Orientation,      ECEF
            obj.w_p = x(11:13);            % Angular Velocity, Body [rad s^-1]

            % --- Parachute ---
            obj.P_c = x(14:16);            % Position,         ECEF [m]
            obj.V_c = x(17:19);            % Velocity,         Body [m   s^-1]

            obj.e_c = x(20:23) / norm(x(20:23)); % Orientation,      ECEF
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

        function calc_mass(obj)
            obj.m_payload = obj.payload.m(obj.rho);
            obj.m_parachute = obj.parachute.m(obj.rho);
        end

        function [f_p, f_c] = calc_gravity(obj)
            f_p = obj.C_EB_p * obj.g_vec_e * obj.m_payload; % Force of gravity
            f_c = obj.C_EB_c * obj.g_vec_e * obj.m_parachute; % Force of gravity
        end

        function [f_p, f_c] = calc_drag(obj)
    % Payload drag (disabled)
    aoa_p = flight_angles(obj.V_p, obj.C_EB_p);
    f_p = [0; 0; 0];  % Payload drag disabled
    
    % VERTICAL-ONLY PARACHUTE DRAG
    % Convert parachute velocity to ENU frame
    V_c_ENU = obj.C_EB_c' * obj.V_c;
    
    % Extract vertical component only
    V_vertical = V_c_ENU(3);  % m/s (positive = up)
    
    % Calculate parachute CdS
    aoa_c = flight_angles(obj.V_c, obj.C_EB_c);
    CdS = obj.parachute.CdS(aoa_c);
    
    % Vertical drag force (opposes vertical motion)
    F_drag_vertical = -0.5 * obj.rho * CdS * V_vertical * abs(V_vertical);
    
    % Apply only in vertical direction (ENU frame)
    F_drag_ENU = [0; 0; F_drag_vertical];
    
    % Convert back to body frame
    f_c = obj.C_EB_c * F_drag_ENU;
    
    % Handle NaN
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

        function [f_p, f_c] = calc_riser_force(obj)
            % --- Velocity of attachment point ---
            % compute relative attach vectors (ECEF)
            r_attach_p_e = obj.C_EB_p' * obj.payload.P_attach_B;
            r_attach_c_e = obj.C_EB_c' * obj.parachute.P_attach_B;

            obj1 = struct( ...
                'V',            obj.C_EB_p' * obj.V_p, ... % COM velocity, ECEF
                'omega',        obj.C_EB_p' * obj.w_p, ...         % omega in ECEF
                'P_attach_rel', r_attach_p_e, ... % relative vector COM->attach in ECEF
                'P_attach',     obj.P_p + r_attach_p_e ...  % absolute attach pos in ECEF
                );

            obj2 = struct( ...
                'V',            obj.C_EB_c' * obj.V_c, ... % COM velocity, ECEF
                'omega',        obj.C_EB_c' * obj.w_c, ...         % omega in ECEF
                'P_attach_rel', r_attach_c_e, ... % relative vector COM->attach in ECEF
                'P_attach',     obj.P_c + r_attach_c_e ...  % absolute attach pos in ECEF
                );

            spring = struct('l0', obj.parachute.l0, ...
                'k', obj.parachute.k_riser, ...
                'c', obj.parachute.c_riser);

            F_spring_e = spring_force(obj1, obj2, spring, true);

            f_p = obj.C_EB_p   *  F_spring_e; % Force of the spring in the body frame
            f_c = obj.C_EB_c * -F_spring_e;
        end
    end
end